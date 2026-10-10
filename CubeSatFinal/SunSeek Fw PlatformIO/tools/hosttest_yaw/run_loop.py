#!/usr/bin/env python3
"""Run the WHOLE firmware on the PC, closed loop, against a simulated spacecraft.

    python3 tools/hosttest_yaw/run_loop.py

src/main.cpp with every module and YOUR Config_*.h is compiled with g++ (fwsim.cpp
supplies a fake I2C bus with a GY-89 on it, a reaction wheel and a body on one axis).
Telecommands are typed in from a script exactly as the ground station sends them.

What this proves: the commands, modes, signs and the control loop hang together with
the configuration on disk. What it does not prove: anything about the real rig. The
spacecraft is a model (see the top of fwsim.cpp) built on three facts from the board
notes: gyro Z negative turning right, atan2(magY, magX) rising turning right, positive
wheel command turning the body left. If one of those is different on the board, change
the Config sign for it and this test will show whether the set is still consistent.
"""
import os, shutil, subprocess, sys, tempfile
import numpy as np
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.normpath(os.path.join(here, "..", ".."))
COLS = ["t", "psi", "om", "cmd", "est", "err", "rel", "abs", "mag", "th", "bias", "rej", "reacq"]

tmp = tempfile.mkdtemp(prefix="sunseek_fwsim_")
exe = os.path.join(tmp, "fwsim")
subprocess.check_call(["g++", "-std=c++17", "-O2", "-w", "-I", os.path.join(here, "fake"), "-I", os.path.join(root, "include"),
                       "-I", os.path.join(root, "src"), os.path.join(here, "fwsim.cpp"), "-o", exe])

def run(script):
    out = subprocess.run([exe], input="\n".join(script) + "\n", capture_output=True, text=True, check=True).stdout
    st = np.array([l.split()[1:] for l in out.splitlines() if l.startswith("ST ")], dtype=float)
    tm = [(float(l.split(" ", 2)[1]), l.split(" ", 2)[2]) for l in out.splitlines() if l.startswith("TM ")]
    return {c: st[:, i] for i, c in enumerate(COLS)}, tm

def at(d, ms, col): return float(d[col][min(np.searchsorted(d["t"], ms), len(d["t"]) - 1)])
def has(tm, text, t0=0, t1=1e12): return any(text in x for (t, x) in tm if t0 <= t <= t1)
def wrap(x): return (np.asarray(x) + 180.0) % 360.0 - 180.0

def step_metrics(d, t0, start, target, band=3.0, T=10.0):
    sel = (d["t"] >= t0) & (d["t"] <= t0 + T * 1000)
    t = (d["t"][sel] - t0) / 1000.0; y = d["psi"][sel]; frac = (y - start) / (target - start)
    rise = float(t[np.argmax(frac >= 0.9)]) if (frac >= 0.9).any() else float("nan")
    over = float((frac.max() - 1.0) * abs(target - start))
    out = np.where(np.abs(y - target) > band)[0]
    settle = 0.0 if not len(out) else (float(t[out[-1] + 1]) if out[-1] + 1 < len(t) else float("nan"))
    return rise, over, settle

fails = 0
def check(ok, text):
    global fails
    fails += 0 if ok else 1
    print("  %-4s %s" % ("ok" if ok else "FAIL", text))

print("== 1. boot, status, tilt compensation guard ==")
d, tm = run(["0 @seed 1", "2000 YAW_STATUS", "2500 YAW_TILT,ON", "2600 YAW_MAG_SIGMA,99", "2700 YAW_MAG_SIGMA,4", "2800 YAW_MAG_SIGMA,3", "2900 HELP", "3000 @end"])
check(has(tm, "YAW_VALID,1") and has(tm, "TILT_COMP,OFF") and has(tm, "ACC_OK,1"), "EKF valid 1 s after boot, accelerometer accepted")
check(has(tm, "ERR,YAW_TILT_NEEDS_3D_MAG_CAL"), "YAW_TILT,ON refused with the present magnetometer calibration")
check(has(tm, "ERR,YAW_MAG_SIGMA_RANGE") and has(tm, "ACK,YAW_MAG_SIGMA,4.00"), "YAW_MAG_SIGMA range checked")
check(has(tm, "TM,HELP,YAW_ZERO"), "HELP lists the new commands")
longest = max(len(x) for (t, x) in tm if x.startswith("TM,YAW"))
check(longest <= 180, "YAW telemetry lines fit one BLE packet (longest %d of 182 bytes)" % longest)

print("== 2. sign set: turn the body to the RIGHT by hand (MANUAL) ==")
d, tm = run(["0 @seed 2", "2000 @kick 40", "3500 @end"])
k0, k1 = np.searchsorted(d["t"], 2000), np.searchsorted(d["t"], 3000)
turned = d["psi"][k1] - d["psi"][k0]
check(turned > 20, "the model turned right by %.0f deg" % turned)
check(abs((d["rel"][k1] - d["rel"][k0]) - turned) < 2.0, "YAW_REL rose by the same amount (%.1f)" % (d["rel"][k1] - d["rel"][k0]))
check(abs(wrap(d["mag"][k1] - d["mag"][k0]) - turned) < 4.0, "MAG heading rose too (%.1f): gyro and magnetometer signs agree" % wrap(d["mag"][k1] - d["mag"][k0]))
check(d["reacq"][-1] == 0 and d["rej"][-1] < 5, "no rejected samples (a climbing YAW_REJECTED while turning = sign mismatch)")

print("== 3. step of +60 deg: MAG reference against the new YAW reference (model without rig drag) ==")
rows = {"MAG": [], "YAW": []}
for seed in range(1, 6):
    mag = ["0 @seed %d" % seed, "0 @tau_f 1000000", "3000 ADCS_REFERENCE,MAG", "3100 SET_TARGET,170", "5000 ADCS_MODE,AUTO", "16000 @end"]
    yaw = ["0 @seed %d" % seed, "0 @tau_f 1000000", "3000 ADCS_REFERENCE,YAW", "3050 YAW_ZERO", "3100 SET_TARGET,60", "5000 ADCS_MODE,AUTO", "16000 @end"]
    for name, sc in (("MAG", mag), ("YAW", yaw)):
        d, tm = run(sc)
        late = d["t"] > 11000
        rows[name].append(list(step_metrics(d, 5000, 0.0, 60.0)) + [float(np.mean(d["psi"][late]) - 60.0), float(d["reacq"][-1]), float(has(tm, "ERR,"))])
m = {k: np.mean(v, axis=0) for k, v in rows.items()}
print("       reference   90 % rise    overshoot   inside 3 deg after   final offset")
for k in ("MAG", "YAW"):
    print("       %-9s   %5.2f s     %5.1f deg     %5.2f s           %+5.2f deg" % (k, m[k][0], m[k][1], m[k][2], m[k][3]))
check(m["YAW"][5] == 0 and m["YAW"][4] == 0, "YAW: no command errors, no re-acquisition")
check(m["YAW"][0] <= 1.15 * m["MAG"][0], "YAW is as fast as MAG (rise time within 15 %)")
check(m["YAW"][1] <= m["MAG"][1] + 1.0, "YAW does not overshoot more than MAG")
check(abs(m["YAW"][3]) < 2.1, "YAW ends inside the controller's 2 deg dead band")
d, tm = run(["0 @seed 1", "3000 ADCS_REFERENCE,YAW", "3050 YAW_ZERO", "3100 SET_TARGET,60", "5000 ADCS_MODE,AUTO", "16000 @end"])
print("       with rig drag (4 s time constant) the same step stops %.1f deg short, on MAG as well: the wheel must keep" % abs(float(np.mean(d["psi"][d["t"] > 11000]) - 60.0)))
print("       turning to hold the body against the momentum the rig took, and u = Kp * error needs an error to do that.")

print("== 4. the short way round through +/-180 ==")
d, tm = run(["0 @seed 3", "0 @tau_f 1000000", "2000 ADCS_REFERENCE,YAW", "2050 YAW_ZERO", "2100 SET_TARGET,-150", "2200 ADCS_MODE,AUTO",
             "12000 ADCS_MODE,MANUAL", "12100 SET_TARGET,170", "12200 ADCS_MODE,AUTO", "22000 @end"])
a, b = at(d, 11900, "psi"), at(d, 21900, "psi")
check(abs(a + 150) < 3, "target -150 reached (true yaw %.1f)" % a)
check(abs(b + 190) < 3, "target +170 reached by turning 40 deg further left, not 320 deg right (true yaw %.1f = +170)" % b)

print("== 5. command rules ==")
d, tm = run(["0 @seed 4", "2000 ADCS_REFERENCE,YAW", "2100 SET_TARGET,181", "2200 SET_TARGET,-180", "2300 SET_TARGET,20", "2400 ADCS_MODE,AUTO",
             "4000 YAW_ZERO", "4100 YAW_RESET", "4200 ADCS_REFERENCE,MAG", "6000 ADCS_MODE,MANUAL", "6100 ADCS_REFERENCE,MAG",
             "6200 SET_TARGET,150", "6300 ADCS_MODE,AUTO", "12000 YAW_ZERO", "12500 YAW_STATUS", "13000 @end"])
check(has(tm, "ERR,TARGET_OUT_OF_RANGE_OR_AUTO", 2100, 2150) and has(tm, "ACK,SET_TARGET,-180.00"), "YAW targets limited to -180..+180")
check(has(tm, "ERR,YAW_ZERO_REFUSED_IN_AUTO_ON_YAW", 4000, 4050), "YAW_ZERO refused while AUTO is flying on YAW")
check(has(tm, "ERR,YAW_RESET_REFUSED_IN_AUTO_ON_YAW", 4100, 4150), "YAW_RESET refused while AUTO is flying on YAW")
check(has(tm, "ERR,REFERENCE_CHANGE_REQUIRES_MANUAL", 4200, 4250), "reference change still needs MANUAL")
check(has(tm, "ACK,YAW_ZERO,", 12000, 12050) and has(tm, "YAW_ZEROED,1", 12400), "YAW_ZERO accepted while MAG holds the target in AUTO")

print("== 6. take the datum while MAG holds a heading, then move relative to it ==")
d, tm = run(["0 @seed 5", "0 @tau_f 1000000", "2000 ADCS_REFERENCE,MAG", "2100 SET_TARGET,150", "2200 ADCS_MODE,AUTO", "10000 YAW_ZERO",
             "10200 ADCS_MODE,MANUAL", "10300 ADCS_REFERENCE,YAW", "10400 SET_TARGET,-30", "10500 ADCS_MODE,AUTO",
             "20000 ADCS_MODE,MANUAL", "20100 SET_TARGET,45", "20200 ADCS_MODE,AUTO", "30000 @end"])
z = at(d, 10000, "psi")
check(abs(at(d, 19900, "psi") - (z - 30)) < 3, "-30 deg from the datum: true yaw %.1f, datum %.1f" % (at(d, 19900, "psi"), z))
check(abs(at(d, 29900, "psi") - (z + 45)) < 3, "+45 deg from the datum: true yaw %.1f" % at(d, 29900, "psi"))

print("== 7. mission on the YAW reference: 3 relative targets (no camera connected: capture times out, mission continues) ==")
d, tm = run(["0 @seed 6", "0 @tau_f 1000000", "2000 ADCS_REFERENCE,YAW", "2050 YAW_ZERO", "2100 MISSION_TARGET,45,5,2", "2200 MISSION_TARGET,-60,5,2",
             "2300 MISSION_TARGET,120,5,2", "2400 MISSION_PREPARE", "2500 MISSION_START", "75000 MISSION_STATUS", "76000 @end"])
held = [x for (t, x) in tm if x.startswith("EVT,MISSION_TARGET_HELD")]
check(len(held) == 3 and has(tm, "EVT,MISSION_COMPLETE"), "all three targets held, mission COMPLETE (%s)" % "; ".join(h.split("HELD,")[1] for h in held))
check(abs(at(d, 75000, "psi") - 120) < 3, "spacecraft still holding the last target (true yaw %.1f)" % at(d, 75000, "psi"))

print("== 8. gyro bias wrong by 1.5 deg/s (GYRO_OFFSET not run), holding a target ==")
res = {}
for name, pre in (("MAG", ["3000 ADCS_REFERENCE,MAG", "3100 SET_TARGET,170"]), ("YAW", ["3000 ADCS_REFERENCE,YAW", "3050 YAW_ZERO", "3100 SET_TARGET,60"])):
    d, tm = run(["0 @seed 7", "0 @gyro_resid 1.5", "0 @tau_f 1000000"] + pre + ["9000 ADCS_MODE,AUTO", "22000 @end"])
    late = d["t"] > 17000
    truth = d["th"] if name == "MAG" else d["psi"] - at(d, 3050, "psi")
    res[name] = float(np.mean(wrap(d["est"] - truth)[late]))
print("       error of the angle the controller is flying on: MAG %+.2f deg, YAW %+.2f deg" % (res["MAG"], res["YAW"]))
check(abs(res["MAG"]) > 1.0 and abs(res["YAW"]) < 0.4, "the complementary filter carries bias x 1 s as an offset; the EKF estimates the bias and removes it")

print("== 9. a magnet near the spacecraft for 0.5 s while holding (moves the magnetometer heading about 26 deg) ==")
out = {}
for name, pre in (("MAG", ["3000 ADCS_REFERENCE,MAG", "3100 SET_TARGET,110"]), ("YAW", ["3000 ADCS_REFERENCE,YAW", "3050 YAW_ZERO", "3100 SET_TARGET,0"])):
    d, tm = run(["0 @seed 8", "0 @tau_f 1000000"] + pre + ["4000 ADCS_MODE,AUTO", "10000 @ext_x -14", "10000 @ext_y -5", "10500 @ext_x 0", "10500 @ext_y 0", "16000 @end"])
    sel = (d["t"] > 10000) & (d["t"] < 14000)
    out[name] = float(np.abs(d["psi"][sel] - at(d, 9990, "psi")).max())
print("       how far the spacecraft was pulled off its attitude: MAG %.1f deg, YAW %.1f deg" % (out["MAG"], out["YAW"]))
check(out["YAW"] < 0.5 * out["MAG"] and out["YAW"] < 2.5, "the gate keeps the disturbance out of the control loop")

print("== 10. mission: capture chosen per target (a fake camera answers CAPTURE) ==")
d, tm = run(["0 @seed 9", "0 @tau_f 1000000", "0 @camera 1", "2000 ADCS_REFERENCE,YAW", "2050 YAW_ZERO",
             "2100 MISSION_TARGET,30,5,1,0", "2200 MISSION_TARGET,-40,5,1,1", "2300 MISSION_TARGET,60,5,1,0",
             "2350 MISSION_TARGET,10,5,1,2", "2370 MISSION_TARGET,10,5,1,x", "2390 MISSION_TARGET,10,5,1,1,1",
             "2400 MISSION_PREPARE", "2500 MISSION_START", "40000 MISSION_STATUS", "41000 @end"])
ev = [x for (t, x) in tm if x.startswith("EVT,MISSION_") or x.startswith("ACK,MISSION_TARGET") or x.startswith("ERR,MISSION")]
check([x for x in ev if x.startswith("ACK,MISSION_TARGET")] == ["ACK,MISSION_TARGET,1,CAPTURE,0", "ACK,MISSION_TARGET,2,CAPTURE,1", "ACK,MISSION_TARGET,3,CAPTURE,0"],
      "MISSION_TARGET,<angle>,<tol>,<hold>,<0|1> accepted and acknowledged with the capture choice")
check(ev.count("ERR,MISSION_TARGET_INVALID") == 3, "capture field other than 0 or 1, or a 5th field, is refused")
skipped = [x for x in ev if x.startswith("EVT,MISSION_CAPTURE_SKIPPED")]; shot = [x for x in ev if x.startswith("EVT,MISSION_CAPTURE,")]
img = [x for x in ev if x.startswith("EVT,MISSION_IMAGE")]; done = [x for x in ev if x.startswith("EVT,MISSION_COMPLETE")]
check(skipped == ["EVT,MISSION_CAPTURE_SKIPPED,1", "EVT,MISSION_CAPTURE_SKIPPED,3"] and shot == ["EVT,MISSION_CAPTURE,2,1"],
      "targets 1 and 3: no CAPTURE sent; target 2: one CAPTURE")
check(len(img) == 1 and img[0].startswith("EVT,MISSION_IMAGE,2,/IMG_0001.JPG") and len(done) == 1 and done[0].split(",")[2] == "1",
      "one picture, from target 2; mission COMPLETE reports 1 image (%s)" % (done[0] if done else "no COMPLETE"))
d, tm = run(["0 @seed 9", "0 @tau_f 1000000", "0 @camera 1", "2000 ADCS_REFERENCE,YAW", "2050 YAW_ZERO",
             "2100 MISSION_TARGET,30,5,1", "2200 MISSION_TARGET,-40,5,1", "2400 MISSION_PREPARE", "2500 MISSION_START", "30000 @end"])
ev = [x for (t, x) in tm if x.startswith("EVT,MISSION_") or x.startswith("ACK,MISSION_TARGET")]
check([x for x in ev if x.startswith("ACK,MISSION_TARGET")] == ["ACK,MISSION_TARGET,1", "ACK,MISSION_TARGET,2"] and
      len([x for x in ev if x.startswith("EVT,MISSION_IMAGE")]) == 2 and not any("SKIPPED" in x for x in ev),
      "old 3-field MISSION_TARGET unchanged: a picture at every target")

shutil.rmtree(tmp, ignore_errors=True)
print("\nRESULT:", "PASS" if not fails else "FAIL (%d)" % fails)
sys.exit(1 if fails else 0)
