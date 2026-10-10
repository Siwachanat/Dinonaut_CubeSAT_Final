#!/usr/bin/env python3
"""
suncal_fit.py - build include/Config_SunCal.h for SUN_ALGORITHM 1
(conductance ratio + level branch, the regional-round method).

Input : a sweep CSV. Either the file saved by the SunSeek Console "Sun cal" tab
        (columns angle_deg, sun_l, sun_r, ..., use) or any CSV whose first three
        columns are angle, left counts, right counts.
Output: Config_SunCal.h (overwrite the whole file) and a report on the terminal.

Angle convention (SunSeek platform): POSITIVE = CubeSat rotated to the RIGHT
(clockwise seen from above), which puts the lamp on the left LDR (SUN_L > SUN_R).
Type the angles that way in the console. Lines starting with # are comments.

    python3 tools/suncal_fit.py tools/suncal_2026-10-06.csv
    python3 tools/suncal_fit.py sweep.csv --label "final arena" --vcc-counts 4300

No third-party packages. The estimator in this file is the same arithmetic as
src/Module_SunSensor.h, so the "check" columns are what the board will report.
"""
import argparse
import csv
import datetime
import math
import os
import re
import sys

NEG = -99.0  # "threshold disabled" marker written to the header


# ----------------------------------------------------------------------------
# input
# ----------------------------------------------------------------------------
def read_sweep(path):
    """-> (points [(angle, L, R)], dark (L, R) or None)"""
    with open(path, newline="") as f:
        rows = [r for r in csv.reader(f)
                if r and any(c.strip() for c in r) and not r[0].lstrip().startswith("#")]
    if not rows:
        sys.exit("empty file: " + path)
    head = [c.strip().lower() for c in rows[0]]
    has_head = any(re.search(r"[a-z]", c) for c in head[:3])
    ia, il, ir, iu = 0, 1, 2, -1
    dark = None
    if has_head:
        def find(pat, default):
            for i, h in enumerate(head):
                if re.search(pat, h) and not re.search(r"fw|dark", h):
                    return i
            return default
        ia = find(r"angle", 0)
        il = find(r"sun_l|^l$|left", 1)
        ir = find(r"sun_r|^r$|right", 2)
        iu = find(r"^use$", -1)
        dl = dr = None
        for h in head:
            m = re.match(r"dark_l=([\d.]+)", h)
            if m:
                dl = float(m.group(1))
            m = re.match(r"dark_r=([\d.]+)", h)
            if m:
                dr = float(m.group(1))
        if dl is not None and dr is not None:
            dark = (dl, dr)
        rows = rows[1:]
    pts = []
    for r in rows:
        try:
            a, L, R = float(r[ia]), float(r[il]), float(r[ir])
        except (ValueError, IndexError):
            continue
        if iu >= 0 and iu < len(r) and r[iu].strip() == "0":
            continue
        pts.append((a, L, R))
    pts.sort()
    return pts, dark


# ----------------------------------------------------------------------------
# the maths (mirrors Module_SunSensor.h)
# ----------------------------------------------------------------------------
def conductance(counts, vcc_counts, gdark=0.0):
    c = min(max(counts, 1.0), vcc_counts - 1.0)
    return max(c / (vcc_counts - c) - gdark, 1e-6)


def interp(xs, ys, x):
    """-> (value, clip) clip = -1 below the table, +1 above, 0 inside"""
    n = len(xs)
    if x <= xs[0]:
        return ys[0], (-1 if x < xs[0] else 0)
    if x >= xs[n - 1]:
        return ys[n - 1], (1 if x > xs[n - 1] else 0)
    i = 1
    while i < n - 1 and xs[i] < x:
        i += 1
    f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
    return ys[i - 1] + f * (ys[i] - ys[i - 1]), 0


class Cal:
    """Everything that goes into Config_SunCal.h."""
    def __init__(self):
        self.label = ""
        self.vcc = 4300.0
        self.gdL = self.gdR = 0.0
        self.X = []
        self.ANG = []
        self.S = []
        self.ln_edge_L = NEG
        self.ln_edge_R = NEG
        self.lvlL_x = []
        self.lvlL_a = []
        self.lvlR_x = []
        self.lvlR_a = []
        self.ln_min_sum = NEG
        self.ang_min = self.ang_max = 0.0


class Estimator:
    """Run-time state: the brightness reference."""
    def __init__(self, cal, alpha=0.02):
        self.cal, self.alpha = cal, alpha
        self.amp, self.valid = 0.0, False

    def step(self, cL, cR):
        c = self.cal
        gL, gR = conductance(cL, c.vcc, c.gdL), conductance(cR, c.vcc, c.gdR)
        lnL, lnR = math.log(gL), math.log(gR)
        X, S = lnL - lnR, math.log(gL + gR)
        amp = self.amp if self.valid else 0.0
        darkL = c.ln_edge_L > -90.0 and (lnL - amp) < c.ln_edge_L
        darkR = c.ln_edge_R > -90.0 and (lnR - amp) < c.ln_edge_R
        n = len(c.X)
        in_table = c.X[0] <= X <= c.X[n - 1]
        if in_table and not darkL and not darkR:
            ang, _ = interp(c.X, c.ANG, X)
            err = S - interp(c.X, c.S, X)[0]
            if not self.valid:
                self.amp, self.valid = err, True
            else:
                self.amp += self.alpha * (err - self.amp)
            branch = "RATIO"
        else:
            if darkL != darkR:
                lit_is_L = darkR
            elif X > c.X[n - 1]:
                lit_is_L = True
            elif X < c.X[0]:
                lit_is_L = False
            else:
                lit_is_L = gL >= gR
            if lit_is_L:
                xs, ys, x, fallback = c.lvlL_x, c.lvlL_a, lnL - amp, c.ANG[n - 1]
            else:
                xs, ys, x, fallback = c.lvlR_x, c.lvlR_a, lnR - amp, c.ANG[0]
            if len(xs) >= 2:
                ang, clip = interp(xs, ys, x)
                branch = "EDGE" if clip < 0 else "LEVEL"
            else:
                ang, branch = fallback, "EDGE"
        if c.ln_min_sum > -90.0 and (S - amp) < c.ln_min_sum:
            branch = "NOSRC"
        return ang, branch, X, S


# ----------------------------------------------------------------------------
# fit
# ----------------------------------------------------------------------------
def fit(pts, vcc, dark=None, label=""):
    notes = []
    cal = Cal()
    cal.label, cal.vcc = label, vcc
    if dark:
        cal.gdL = dark[0] / (vcc - dark[0])
        cal.gdR = dark[1] / (vcc - dark[1])
    A = [p[0] for p in pts]
    gL = [conductance(p[1], vcc, cal.gdL) for p in pts]
    gR = [conductance(p[2], vcc, cal.gdR) for p in pts]
    X = [math.log(a / b) for a, b in zip(gL, gR)]
    S = [math.log(a + b) for a, b in zip(gL, gR)]
    n = len(pts)
    if n < 5:
        sys.exit("need at least 5 points")

    # direction of X against angle: majority vote of the steps
    up = sum(1 for i in range(1, n) if X[i] > X[i - 1])
    sgn = 1 if up * 2 >= (n - 1) else -1

    # longest strictly monotonic run in that direction
    best = (0, 0)
    i = 0
    while i < n - 1:
        j = i
        while j < n - 1 and (X[j + 1] - X[j]) * sgn > 0:
            j += 1
        if j - i > best[1] - best[0]:
            best = (i, j)
        i = max(j, i + 1)
    r0, r1 = best
    fold_lo, fold_hi = r0 > 0, r1 < n - 1          # a fold was seen on that side
    z0 = r0 + 1 if fold_lo else r0                 # the turning point itself is not
    z1 = r1 - 1 if fold_hi else r1                 # trusted for the ratio table
    if z1 - z0 < 2:
        sys.exit("ratio zone has fewer than 3 points - check the sweep")

    order = list(range(z0, z1 + 1))
    if sgn < 0:
        order.reverse()                            # rows ascending in X
    cal.X = [X[i] for i in order]
    cal.ANG = [A[i] for i in order]
    cal.S = [S[i] for i in order]

    def side(zone_i, step, fold):
        """One end of the zone. step = -1 towards lower angle, +1 towards higher."""
        lit_is_L = gL[zone_i] >= gR[zone_i]
        g_lit, g_sh = (gL, gR) if lit_is_L else (gR, gL)
        name = "L" if lit_is_L else "R"
        ln_edge = NEG
        xs, ys = [], []
        if fold:
            nxt = zone_i + step
            if g_sh[nxt] < g_sh[zone_i]:
                ln_edge = 0.5 * (math.log(g_sh[zone_i]) + math.log(g_sh[nxt]))
            else:
                notes.append("WARNING: at %+g deg the shadowed cell does not get darker "
                             "past the zone edge; darkness test disabled on that side" % A[zone_i])
            k = zone_i
            tab = [(math.log(g_lit[k]), A[k])]
            while 0 <= k + step < n and g_lit[k + step] < g_lit[k]:
                k += step
                tab.append((math.log(g_lit[k]), A[k]))
            if len(tab) >= 2:
                tab.sort()
                xs, ys = [t[0] for t in tab], [t[1] for t in tab]
            else:
                notes.append("WARNING: no level table past %+g deg (lit cell not falling)" % A[zone_i])
        else:
            notes.append("note: the sweep ends at %+g deg with the ratio still rising - "
                         "no level table on that side, readings beyond clamp there" % A[zone_i])
        return lit_is_L, name, ln_edge, xs, ys

    for zone_i, step, fold in ((z0, -1, fold_lo), (z1, +1, fold_hi)):
        lit_is_L, name, ln_edge, xs, ys = side(zone_i, step, fold)
        if lit_is_L:                               # L lit -> R is the shadowed one
            cal.ln_edge_R, cal.lvlL_x, cal.lvlL_a = ln_edge, xs, ys
        else:
            cal.ln_edge_L, cal.lvlR_x, cal.lvlR_a = ln_edge, xs, ys

    if sgn < 0:
        notes.append("WARNING: X = ln(G_L/G_R) FALLS as the angle rises, i.e. positive angles have the lamp "
                     "on the RIGHT cell. The platform convention is + = CubeSat rotated right = lamp on the "
                     "LEFT cell. If the sheet was written the other way round, rerun with --negate-angle.")
    cal.ln_min_sum = min(S) - math.log(2.0)
    cov = list(cal.ANG) + cal.lvlL_a + cal.lvlR_a
    cal.ang_min, cal.ang_max = min(cov), max(cov)
    info = dict(A=A, gL=gL, gR=gR, X=X, S=S, zone=(z0, z1), sgn=sgn, notes=notes)
    return cal, info


# ----------------------------------------------------------------------------
# report
# ----------------------------------------------------------------------------
def counts_at(g_true, vcc_true):
    return vcc_true * g_true / (1.0 + g_true)


def report(pts, cal, info, out=sys.stdout):
    w = out.write
    A, X, S = info["A"], info["X"], info["S"]
    z0, z1 = info["zone"]
    w("Calibration '%s'  -  %d points, counts at 3V3 = %.0f\n" % (cal.label, len(pts), cal.vcc))
    w("Ratio zone  : %+g .. %+g deg  (%d rows, X = ln(G_L/G_R) %+.3f .. %+.3f)\n"
      % (A[z0], A[z1], len(cal.X), cal.X[0], cal.X[-1]))
    for nm, xs, ys in (("L", cal.lvlL_x, cal.lvlL_a), ("R", cal.lvlR_x, cal.lvlR_a)):
        if len(xs) >= 2:
            w("Level table : cell %s lit, %+g .. %+g deg (%d rows)\n" % (nm, min(ys), max(ys), len(xs)))
        else:
            w("Level table : cell %s lit - none\n" % nm)
    w("Coverage    : %+g .. %+g deg\n" % (cal.ang_min, cal.ang_max))
    for t in info["notes"]:
        w(t + "\n")

    w("\n angle     L      R        X        S   branch   board says   error\n")
    worst = 0.0
    for (a, L, R) in pts:
        est = Estimator(cal)
        mid = pts[(z0 + z1) // 2]
        est.step(mid[1], mid[2])                   # reference learned in the zone
        ang, br, x, s = est.step(L, R)
        e = ang - a
        inside = cal.ang_min <= a <= cal.ang_max
        if inside:
            worst = max(worst, abs(e))
        w(" %+5.0f  %5.0f  %5.0f  %+7.4f  %7.4f  %-6s  %+8.2f   %+6.2f%s\n"
          % (a, L, R, x, s, br, ang, e, "" if inside else "  (outside coverage)"))
    w("Worst error at the calibration points inside the coverage: %.2f deg\n" % worst)
    if worst > 0.05:
        w("WARNING: a calibration point does not read back as itself - inspect the rows above\n")

    # leave-one-out: how much the straight lines between rows are worth
    loo = []
    for k in range(1, len(cal.X) - 1):
        xs = cal.X[:k] + cal.X[k + 1:]
        ys = cal.ANG[:k] + cal.ANG[k + 1:]
        loo.append(interp(xs, ys, cal.X[k])[0] - cal.ANG[k])
    if loo:
        w("Ratio zone leave-one-out (interpolation error estimate): max %.2f, rms %.2f deg\n"
          % (max(abs(v) for v in loo), math.sqrt(sum(v * v for v in loo) / len(loo))))

    # What a brightness change costs. Model: LDR power law, both cells alike, so a
    # brighter or dimmer lamp multiplies both conductances by the same factor. The
    # counts-at-3V3 constant is taken 100 counts wrong each way, because it is not
    # known exactly; with the exact constant the ratio zone would not move at all.
    w("\nSimulated lamp change (conductance x k; counts-at-3V3 really %.0f or %.0f, not %.0f):\n"
      % (cal.vcc - 100, cal.vcc + 100, cal.vcc))
    mid = pts[(z0 + z1) // 2]

    def shifted(c, k, vt):
        return counts_at(conductance(c, vt) * k, vt)

    for k in (0.75, 0.9, 1.15, 1.3):
        worst_r = worst_l = worst_c = 0.0
        for vt in (cal.vcc - 100.0, cal.vcc + 100.0):
            for (a, L, R) in pts:
                if not (cal.ang_min <= a <= cal.ang_max):
                    continue
                l2, r2 = shifted(L, k, vt), shifted(R, k, vt)
                if max(l2, r2) > 4090:
                    continue
                est = Estimator(cal)
                for _ in range(200):               # reference settled inside the zone
                    est.step(shifted(mid[1], k, vt), shifted(mid[2], k, vt))
                ang, br, _, _ = est.step(l2, r2)
                if br == "RATIO":
                    worst_r = max(worst_r, abs(ang - a))
                else:
                    worst_l = max(worst_l, abs(ang - a))
                worst_c = max(worst_c, abs(Estimator(cal).step(l2, r2)[0] - a))
        w("  k = %.2f : ratio zone worst %.1f deg | level branch worst %.1f deg | "
          "no reference yet (first reading outside the zone) worst %.1f deg\n" % (k, worst_r, worst_l, worst_c))


# ----------------------------------------------------------------------------
# header
# ----------------------------------------------------------------------------
def carr(name, vals, fmt):
    vals = list(vals) if len(vals) else [0.0, 0.0]          # C++ has no zero-length arrays
    body = ", ".join((fmt % v) + "f" for v in vals)
    return "static const float %s[] = { %s };\n" % (name, body)


def header(cal, src, pts):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    h = []
    h.append("#pragma once\n")
    h.append("/* Config_SunCal.h - GENERATED by tools/suncal_fit.py, %s\n" % now)
    h.append("   Source: %s (%d points). Overwrite this whole file; do not edit by hand.\n"
             % (os.path.basename(src), len(pts)))
    h.append("   Used when SUN_ALGORITHM is 1 (Config_Sensor.h). Valid for one baffle and one\n")
    h.append("   lamp position; the ratio zone tolerates brightness changes, the level branch\n")
    h.append("   relies on the brightness reference learned inside the ratio zone.\n")
    h.append("   Coverage %+g .. %+g deg; ratio zone %+g .. %+g deg. */\n\n"
             % (cal.ang_min, cal.ang_max, min(cal.ANG), max(cal.ANG)))
    safe = re.sub(r"[^A-Za-z0-9._+-]+", "_", cal.label).strip("_")      # goes into comma-separated telemetry
    h.append('#define SUNCAL_LABEL          "%s"\n' % safe)
    h.append("#define SUNCAL_COUNTS_AT_VCC  %.1ff   /* ADC counts the 3V3 rail would read */\n" % cal.vcc)
    h.append("#define SUNCAL_GDARK_L        %.6ef\n" % cal.gdL)
    h.append("#define SUNCAL_GDARK_R        %.6ef\n" % cal.gdR)
    h.append("\n/* Ratio zone. X = ln(G_L) - ln(G_R), rows ascending in X.\n")
    h.append("   SUNCAL_S = ln(G_L + G_R) at calibration brightness (brightness reference). */\n")
    h.append("#define SUNCAL_N %d\n" % len(cal.X))
    h.append(carr("SUNCAL_X", cal.X, "%.5f"))
    h.append(carr("SUNCAL_ANG", cal.ANG, "%.2f"))
    h.append(carr("SUNCAL_S", cal.S, "%.5f"))
    h.append("\n/* Darkness thresholds, ln(G) at calibration brightness. A cell below its\n")
    h.append("   threshold is in shadow: the ratio has folded back and must not be used.\n")
    h.append("   -99 = test disabled. */\n")
    h.append("#define SUNCAL_LN_EDGE_L %.5ff\n" % cal.ln_edge_L)
    h.append("#define SUNCAL_LN_EDGE_R %.5ff\n" % cal.ln_edge_R)
    h.append("\n/* Level branch: ln(G) of the lit cell -> angle, rows ascending in ln(G). */\n")
    h.append("#define SUNCAL_LVL_L_N %d\n" % len(cal.lvlL_x))
    h.append(carr("SUNCAL_LVL_L_X", cal.lvlL_x, "%.5f"))
    h.append(carr("SUNCAL_LVL_L_ANG", cal.lvlL_a, "%.2f"))
    h.append("#define SUNCAL_LVL_R_N %d\n" % len(cal.lvlR_x))
    h.append(carr("SUNCAL_LVL_R_X", cal.lvlR_x, "%.5f"))
    h.append(carr("SUNCAL_LVL_R_ANG", cal.lvlR_a, "%.2f"))
    h.append("\n/* ln(G_L + G_R) below this = no light source in view. */\n")
    h.append("#define SUNCAL_LN_MIN_SUM %.5ff\n" % cal.ln_min_sum)
    h.append("#define SUNCAL_ANG_MIN %.2ff\n" % cal.ang_min)
    h.append("#define SUNCAL_ANG_MAX %.2ff\n" % cal.ang_max)
    return "".join(h)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out", default=None, help="default: ../include/Config_SunCal.h next to this script")
    ap.add_argument("--label", default=None)
    ap.add_argument("--vcc-counts", type=float, default=4300.0,
                    help="ADC counts the 3V3 rail would read (SUN_STATUS prints an estimate). Default 4300")
    ap.add_argument("--use-dark", action="store_true", help="subtract the dark reading stored in the CSV header")
    ap.add_argument("--negate-angle", action="store_true",
                    help="flip the sign of every angle in the CSV (sheet taken with + = CubeSat rotated LEFT)")
    ap.add_argument("--dry-run", action="store_true", help="report only, write nothing")
    a = ap.parse_args()

    pts, dark = read_sweep(a.csv)
    if a.negate_angle:
        pts = sorted((-ang, L, R) for (ang, L, R) in pts)
    label = a.label or os.path.splitext(os.path.basename(a.csv))[0]
    top = max(max(p[1], p[2]) for p in pts)
    if top >= a.vcc_counts - 100:
        sys.exit("a reading (%.0f) is within 100 counts of --vcc-counts: the cell is saturated or the constant is too low" % top)
    cal, info = fit(pts, a.vcc_counts, dark if a.use_dark else None, label)
    report(pts, cal, info)
    if top >= 4000:
        print("WARNING: readings up to %.0f counts - too close to the ADC ceiling (4095). Dim the lamp, "
              "move it back or lower the series resistor, then sweep again." % top)
    elif top > 3500:
        print("note: readings up to %.0f counts. The procedure's target is 2500-3500 at the peak; "
              "above that the divider is compressed and the board is less sure of its own brightness." % top)
    if a.dry_run:
        return
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "include", "Config_SunCal.h")
    with open(out, "w") as f:
        f.write(header(cal, a.csv, pts))
    print("\nwrote", os.path.normpath(out))


if __name__ == "__main__":
    main()
