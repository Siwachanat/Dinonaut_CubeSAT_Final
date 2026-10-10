#!/usr/bin/env python3
"""Compile the firmware's yaw EKF for the PC and check it against a simulated spacecraft.

    python3 tools/hosttest_yaw/run.py

The same C++ that runs on the board (src/Module_YawEKF.h) is driven sample by
sample, next to the organisers' estimator (src/Module_Estimator.h) on identical
data. The spacecraft and sensors are a MODEL (tools/hosttest_yaw/yawmodel.py):
the numbers show that the code behaves as designed, not what the real board does.
"""
import os, shutil, subprocess, sys, tempfile
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.normpath(os.path.join(here, "..", ".."))
sys.path.insert(0, here)
import yawmodel as ym

def build(exe, *defs):
    cmd = ["g++", "-std=c++17", "-O2", "-w", "-I", here, "-I", os.path.join(root, "include"), "-I", os.path.join(root, "src"),
           *["-D" + d for d in defs], os.path.join(here, "yawsim.cpp"), "-o", exe]
    subprocess.check_call(cmd)

def run(exe, text_lines):
    out = subprocess.run([exe], input="\n".join(text_lines) + "\n", capture_output=True, text=True, check=True).stdout
    notes = [l for l in out.splitlines() if l.startswith("#")]
    a = np.array([l.split() for l in out.splitlines() if not l.startswith("#")], dtype=float)
    return {c: a[:, i] for i, c in enumerate(ym.COLS)}, notes

def rms(x): return float(np.sqrt(np.mean(np.square(x))))

fails = 0
def check(ok, text):
    global fails
    fails += 0 if ok else 1
    print("  %-4s %s" % ("ok" if ok else "FAIL", text))

tmp = tempfile.mkdtemp(prefix="sunseek_yawtest_")
exe = os.path.join(tmp, "yawsim");            build(exe)
exe_neg = os.path.join(tmp, "yawsim_neg");    build(exe_neg, "YAWSIM_SIGN=-1.0f", "YAWSIM_OFFSET_DEG=37.0f")
exe_tilt = os.path.join(tmp, "yawsim_tilt");  build(exe_tilt, "YAWSIM_SCALE_Z=1.0f")
MISSION = [(0, 0), (6, 60), (14, -90), (24, 150), (36, 0)]

print("== 1. at rest, gyro bias 0.8 deg/s left uncorrected ==")
d = ym.simulate([(0, 0)], 60.0, seed=3, gyro_bias=0.8)
r, _ = run(exe, ym.lines(d))
late = d["t"] > 20
e = ym.wrap(r["ekf_abs"] - d["heading_true"]); s = ym.wrap(r["std_est"] - d["heading_true"]); w = ym.wrap(r["std_raw"] - d["heading_true"])
print("       after 20 s      mean error   noise (1 sigma)")
print("       magnetometer      %+6.2f       %5.3f deg" % (w[late].mean(), w[late].std()))
print("       organisers' est.  %+6.2f       %5.3f deg" % (s[late].mean(), s[late].std()))
print("       EKF               %+6.2f       %5.3f deg     bias estimate %.3f (true %.3f) deg/s" % (e[late].mean(), e[late].std(), r["bias"][-1], d["bias"][-1]))
check(abs(e[late].mean()) < 0.15 and e[late].std() < 0.15, "EKF holds the heading with no offset from the gyro bias")
check(abs(r["bias"][-1] - d["bias"][-1]) < 0.08, "gyro bias found to better than 0.08 deg/s")
i5 = np.searchsorted(d["t"], 5.0)
check(abs(r["bias"][i5] - d["bias"][i5]) < 0.2, "bias within 0.2 deg/s after 5 s (%.2f)" % r["bias"][i5])

print("== 2. mission profile 0 -> +60 -> -90 -> +150 -> 0 at 40 deg/s, 8 random runs ==")
res = []
for seed in range(1, 9):
    d = ym.simulate(MISSION, 50.0, seed=seed)
    r, _ = run(exe, ym.lines(d))
    sel = d["t"] > 3; mov = np.abs(d["w"]) > 5
    e = ym.wrap(r["ekf_abs"] - d["heading_true"]); s = ym.wrap(r["std_est"] - d["heading_true"]); w = ym.wrap(r["std_raw"] - d["heading_true"])
    res.append([rms(e[sel]), np.abs(e[sel]).max(), rms(e[sel & mov]), rms(e[sel & ~mov]),
                rms(s[sel]), np.abs(s[sel]).max(), rms(s[sel & mov]), rms(s[sel & ~mov]),
                rms(w[sel]), np.abs(w[sel]).max(), rms(w[sel & mov]), rms(w[sel & ~mov]), r["reacq"][-1]])
m = np.mean(res, axis=0)
print("                         rms     max   turning   holding   [deg]")
print("       magnetometer     %5.2f  %5.2f   %5.2f     %5.2f" % tuple(m[8:12]))
print("       organisers' est. %5.2f  %5.2f   %5.2f     %5.2f" % tuple(m[4:8]))
print("       EKF              %5.2f  %5.2f   %5.2f     %5.2f" % tuple(m[0:4]))
check(m[0] < 0.7 and m[0] < m[4], "EKF error below 0.7 deg rms and below the organisers' estimator")
check(m[2] < 1.0, "no lag while turning (EKF %.2f deg rms at 40 deg/s)" % m[2])
check(m[12] == 0, "no re-acquisition needed")
mission_ref = m[0]

print("== 3. other mounting: MAG_HEADING_SIGN -1 and a 37 deg heading offset ==")
res = []
for seed in range(1, 5):
    d = ym.simulate(MISSION, 50.0, seed=seed, z_down=True)
    r, _ = run(exe_neg, ym.lines(d))
    sel = d["t"] > 3
    res.append([rms(ym.wrap(r["ekf_abs"] - 37.0 - d["heading_true"])[sel]), rms(ym.wrap(r["ekf_abs"] - r["std_raw"])[sel]), r["reacq"][-1]])
m = np.mean(res, axis=0)
check(m[0] < 0.7 and m[2] == 0, "same accuracy with the opposite sign (%.2f deg rms)" % m[0])

print("== 4. three full turns at 30 deg/s through 0/360 ==")
d = ym.simulate([(0, 0), (3, 1080)], 45.0, seed=5)
r, _ = run(exe, ym.lines(d))
e = ym.wrap(r["ekf_abs"] - d["heading_true"])[d["t"] > 3]
check(np.abs(e).max() < 2.5 and r["reacq"][-1] == 0, "no jump at the wrap (max error %.2f deg)" % np.abs(e).max())

print("== 5. YAW_ZERO and the +/-180 wrap of the relative angle ==")
d = ym.simulate([(0, 40), (10, 240)], 25.0, seed=6)     # starts at 40, zero there, then +200 more
L = ym.lines(d); k = int(np.searchsorted(d["t"], 8.0))
r, _ = run(exe, L[:k] + ["zero"] + L[k:])
check(abs(r["ekf_rel"][k + 5]) < 0.5, "relative yaw reads 0 after YAW_ZERO (%.2f)" % r["ekf_rel"][k + 5])
check(abs(r["ekf_rel"][0]) < 3.0, "before YAW_ZERO the datum is the boot attitude (%.2f)" % r["ekf_rel"][0])
check(abs(ym.wrap(r["ekf_rel"][-1] - (-160.0))) < 2.0, "+200 deg from the datum reads -160 (%.1f)" % r["ekf_rel"][-1])

print("== 6. a 3 s gap in sampling while the body turned 50 deg ==")
d = ym.simulate([(0, 0), (5, 50)], 14.0, seed=7, jitter=False)
L = ym.lines(d); k0 = int(np.searchsorted(d["t"], 4.9)); k1 = int(np.searchsorted(d["t"], 7.9))
gap = L[k1].split(); gap[0] = "3.000000"
r, _ = run(exe, L[:k0] + [" ".join(gap)] + L[k1 + 1:])
tt = np.concatenate([d["heading_true"][:k0], d["heading_true"][k1:]])
e = ym.wrap(r["ekf_abs"] - tt)
check(np.abs(e[k0 + 10:]).max() < 3.0, "heading re-acquired within 10 samples after the gap (%.2f deg)" % np.abs(e[k0 + 10:]).max())

print("== 7. magnetic disturbances ==")
SMALL, BIG = (-5.0, -1.8, 0.0), (-20.0, -7.0, 0.0)       # fields across the Earth field at heading 110: about 7 and 27 deg
for name, vec in (("small", SMALL), ("large", BIG)):
    d = ym.simulate([(0, 0)], 20.0, seed=8, pulse=(8.0, 8.4, vec))                # lasts 0.4 s
    r, _ = run(exe, ym.lines(d))
    sel = (d["t"] > 7.9) & (d["t"] < 10.0)
    e = ym.wrap(r["ekf_abs"] - d["heading_true"]); s = ym.wrap(r["std_est"] - d["heading_true"]); w = ym.wrap(r["std_raw"] - d["heading_true"])
    print("       0.4 s %s field pulse: magnetometer moves %.0f deg, organisers' estimate %.1f deg, EKF %.2f deg" % (name, np.abs(w[sel]).max(), np.abs(s[sel]).max(), np.abs(e[sel]).max()))
    if name == "small": check(np.abs(e[sel]).max() < 1.6 and np.abs(e[sel]).max() < 0.6 * np.abs(s[sel]).max() and r["rejected"][-1] == 0, "inside the gate: used, with a small weight")
    else: check(np.abs(e[sel]).max() < 0.6 and r["rejected"][-1] >= 15 and r["reacq"][-1] == 0, "outside the gate: not used at all (%d samples dropped)" % r["rejected"][-1])
d = ym.simulate([(0, 0)], 30.0, seed=9, pulse=(8.0, 18.0, BIG))                   # 10 s: the field really changed
r, _ = run(exe, ym.lines(d))
e = ym.wrap(r["ekf_abs"] - d["heading_true"]); f = ym.wrap(r["ekf_abs"] - r["mag_abs"])
hold = (d["t"] > 8.0) & (d["t"] < 8.9); after = (d["t"] > 9.6) & (d["t"] < 17.5); back = d["t"] > 20.5
check(np.abs(e[hold]).max() < 1.0, "a lasting change is held off for 1 s (%.2f deg)" % np.abs(e[hold]).max())
check(np.abs(f[after]).max() < 6.0 and r["reacq"][-1] == 2, "then the filter re-acquires the magnetometer, and again when it ends (%d)" % r["reacq"][-1])
check(np.abs(e[back]).max() < 1.5, "back on the true heading afterwards (%.2f deg)" % np.abs(e[back]).max())
d = ym.simulate(MISSION, 50.0, seed=2, mag_glitch_p=0.01, mag_glitch=25.0)
r, _ = run(exe, ym.lines(d))
e = ym.wrap(r["ekf_abs"] - d["heading_true"])[d["t"] > 3]
check(rms(e) < mission_ref + 0.15 and r["rejected"][-1] > 5, "single-sample spikes of 25 uT are dropped (%d dropped, %.2f deg rms)" % (r["rejected"][-1], rms(e)))

print("== 8. accelerometer: tilt, and tilt compensation ==")
d = ym.simulate([(0, 0)], 6.0, seed=10)
r, notes = run(exe, ["tilt 1"] + ym.lines(d))
check(notes == ["# tilt 0"] and r["tilt_used"].max() == 0, "refused with the present calibration (MAG_SCALE_Z %.2f is not a real Z calibration)" %
      float(open(os.path.join(root, "include", "Config_Sensor.h")).read().split("#define MAG_SCALE_Z")[1].split("f")[0]))
kw = dict(seed=11, b_h=38.0, b_v=22.0, tilt_static=(8.0, -6.0), motor_field=(0, 0, 0), gyro_scale=0.0)
d = ym.simulate([(0, 0), (3, 360)], 20.0, **kw)
r0, _ = run(exe_tilt, ["tilt 0"] + ym.lines(d)); r1, n1 = run(exe_tilt, ["tilt 1"] + ym.lines(d))
sel = d["t"] > 3
e0 = ym.wrap(r0["ekf_abs"] - d["heading_true"])[sel]; e1 = ym.wrap(r1["ekf_abs"] - d["heading_true"])[sel]
print("       10 deg tilt, dip 30 deg, one full turn: error without %.1f deg max, with compensation %.1f deg max" % (np.abs(e0).max(), np.abs(e1).max()))
check(abs(r1["tilt"][-1] - 10.0) < 0.5, "tilt angle measured: %.1f deg (true 10.0)" % r1["tilt"][-1])
check(np.abs(e1).max() < 1.2 and np.abs(e0).max() > 3.0, "compensation removes the tilt error (needs a 3-D magnetometer calibration)")
d = ym.simulate(MISSION, 50.0, seed=1)
r0, _ = run(exe_tilt, ["tilt 0"] + ym.lines(d)); r1, _ = run(exe_tilt, ["tilt 1"] + ym.lines(d))
check(np.abs(ym.wrap(r0["ekf_abs"] - r1["ekf_abs"])).max() < 0.15, "level spacecraft: compensation changes nothing (%.2f deg)" % np.abs(ym.wrap(r0["ekf_abs"] - r1["ekf_abs"])).max())
d = ym.simulate([(0, 0)], 30.0, seed=12, swing_deg=4.0, b_v=22.0)
r, _ = run(exe, ym.lines(d))
late = d["t"] > 10
e = ym.wrap(r["ekf_abs"] - d["heading_true"])[late]; w = ym.wrap(r["std_raw"] - d["heading_true"])[late]; s = ym.wrap(r["std_est"] - d["heading_true"])[late]
print("       swinging 4 deg at 0.8 Hz: magnetometer %.2f, organisers' est. %.2f, EKF %.2f deg rms" % (rms(w), rms(s), rms(e)))
check(rms(e) < 0.5 * rms(w), "a swing is averaged out (the accelerometer cannot see it; the other gyro axes do)")

print("== 9. bad input ==")
L = ym.lines(ym.simulate([(0, 0)], 4.0, seed=13))
bad = ["0.02 nan 0 0 1 1 1 1 0 0 16393", "0.02 0 0 0 nan 1 1 1 0 0 16393", "0.02 0 0 0 0 0 0 1 0 0 0", "-1 0 0 0 30 20 -15 1 0 0 16393", "0.02 0 0 0 30 20 -15 1 nan nan nan"]
r, _ = run(exe, L[:100] + bad + L[100:])
check(np.isfinite(np.array([r[c] for c in ym.COLS if c not in ("std_est", "std_filt", "std_raw")])).all() and abs(ym.wrap(r["ekf_abs"][-1] - 110.0)) < 1.5,
      "NaN, zero field and negative dt leave the estimate finite and correct")

print("== 10. cost on this PC (the board reports its own figure in YAW_STATUS: EKF_US) ==")
import time
L = ym.lines(ym.simulate(MISSION, 50.0, seed=1)) * 20
t0 = time.time(); run(exe, L); t1 = time.time()
print("       %.2f us per cycle here for BOTH estimators plus text parsing and printing" % ((t1 - t0) / len(L) * 1e6))

shutil.rmtree(tmp, ignore_errors=True)
print("\nRESULT:", "PASS" if not fails else "FAIL (%d)" % fails)
sys.exit(1 if fails else 0)
