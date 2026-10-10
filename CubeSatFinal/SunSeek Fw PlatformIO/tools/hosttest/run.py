#!/usr/bin/env python3
"""Compile the firmware's sun-sensor module for the PC and check it.

    python3 tools/hosttest/run.py [sweep.csv]

1. every calibration point must read back as its own angle (algorithm 1)
2. the board code and tools/suncal_fit.py must agree on random inputs
3. algorithm 0 (organisers' NDV + team LUT) must still return the LUT angles
"""
import os, random, shutil, subprocess, sys, tempfile
here = os.path.dirname(os.path.abspath(__file__))
root = os.path.normpath(os.path.join(here, "..", ".."))
sys.path.insert(0, os.path.join(root, "tools"))
import suncal_fit as sf

def build(alg, exe):
    cmd = ["g++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Wno-unused-function", "-Wno-unused-variable",
           "-I", here, "-I", os.path.join(root, "include"), "-I", os.path.join(root, "src"),
           "-DSUN_ALGORITHM_OVERRIDE=%d" % alg, os.path.join(here, "hosttest.cpp"), "-o", exe]
    subprocess.check_call(cmd)

def run(exe, lines):
    out = subprocess.run([exe], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True).stdout
    return [l.split() for l in out.strip().splitlines()]

csv = sys.argv[1] if len(sys.argv) > 1 else os.path.join(root, "tools", "suncal_2026-10-06.csv")
pts, dark = sf.read_sweep(csv)
fails = 0

# what the header on disk was built with
hdr = open(os.path.join(root, "include", "Config_SunCal.h")).read()
vcc = float(hdr.split("SUNCAL_COUNTS_AT_VCC")[1].split("f")[0])
cal, info = sf.fit(pts, vcc, None, "hosttest")
z0, z1 = info["zone"]
mid = pts[(z0 + z1) // 2]

tmp = tempfile.mkdtemp(prefix="sunseek_hosttest_")     # binaries never land in the project
exe1 = os.path.join(tmp, "hosttest_alg1")
build(1, exe1)
print("== 1. calibration points through the board code (algorithm 1) ==")
lines = []
for (a, L, R) in pts:
    lines += ["reset", "%g %g" % (mid[1], mid[2]), "%g %g" % (L, R)]
res = run(exe1, lines)
for k, (a, L, R) in enumerate(pts):
    ang, br = float(res[2 * k + 1][0]), res[2 * k + 1][1]
    inside = cal.ang_min <= a <= cal.ang_max
    bad = inside and abs(ang - a) > 0.02
    fails += bad
    print("  %+5.0f deg  L %5.0f R %5.0f -> %+8.2f  %-5s %s" % (a, L, R, ang, br, "FAIL" if bad else ("ok" if inside else "outside coverage")))

print("== 2. board code vs suncal_fit.py on 5000 random inputs, reference carried along ==")
random.seed(7)
est = sf.Estimator(cal)
lines, exp = [], []
for i in range(5000):
    L, R = random.randint(300, 4090), random.randint(300, 4090)
    lines.append("%d %d" % (L, R))
    exp.append(est.step(L, R))
res = run(exe1, ["reset"] + lines)
worst = 0.0
for r, e in zip(res, exp):
    worst = max(worst, abs(float(r[0]) - e[0]))
    if r[1] != e[1]:
        fails += 1
print("  worst angle difference %.4f deg, branch mismatches %d" % (worst, sum(1 for r, e in zip(res, exp) if r[1] != e[1])))
fails += worst > 0.02

exe0 = os.path.join(tmp, "hosttest_alg0")
build(0, exe0)
print("== 3. algorithm 0 (raw NDV + team LUT in Config_Sensor.h) ==")
res = run(exe0, ["%g %g" % (L, R) for (a, L, R) in pts])
for (a, L, R), r in zip(pts, res):
    ang = float(r[0])
    inside = -50 <= a <= 60
    bad = inside and abs(ang - a) > 0.05
    fails += bad
    print("  %+5.0f deg -> %+8.2f %s" % (a, ang, "FAIL" if bad else ("ok" if inside else "outside the LUT (folds back)")))

shutil.rmtree(tmp, ignore_errors=True)
print("\nRESULT:", "PASS" if not fails else "FAIL (%d)" % fails)
sys.exit(1 if fails else 0)
