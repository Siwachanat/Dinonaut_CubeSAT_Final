"""Synthetic spacecraft + GY-89 for testing the yaw estimators on a PC.

Everything here is a MODEL with assumed numbers (datasheet noise, guessed motor
field). It is good for checking the code and comparing estimators on equal
terms. It is not a measurement of the real board.

Conventions (same as the firmware after 2026-10-07):
  yaw  : degrees, positive = spacecraft rotated to the right (clockwise from above)
  frame: sensor X forward, Y left, Z up (z_down=True flips Y and Z, which is the
         mounting that needs MAG_HEADING_SIGN -1)
"""
import math
import numpy as np

G1 = 16393.0  # accelerometer counts per g


def slew_profile(t, steps, w_max=40.0, a_max=80.0):
    """Rate/acceleration-limited moves between hold angles. steps = [(time, target_deg), ...]."""
    dt = t[1] - t[0]
    psi = np.zeros_like(t)
    w = np.zeros_like(t)
    p, v, tgt, k = steps[0][1], 0.0, steps[0][1], 0
    for i, ti in enumerate(t):
        while k + 1 < len(steps) and ti >= steps[k + 1][0]:
            k += 1
            tgt = steps[k][1]
        e = tgt - p
        v_des = math.copysign(min(w_max, math.sqrt(2 * a_max * abs(e))), e)   # time-optimal braking curve
        dv = max(-a_max * dt, min(a_max * dt, v_des - v))
        v += dv
        if abs(e) < 1e-3 and abs(v) < a_max * dt:
            v = 0.0
            p = tgt
        p += v * dt
        psi[i], w[i] = p, v
    return psi, w


def simulate(steps, T, seed=1, dt=0.02,
             heading0=110.0,            # compass heading of the X axis at yaw 0 [deg]
             b_h=40.0, b_v=15.0,        # horizontal / vertical (down) field [uT]: dip about 20 deg
             gyro_bias=0.4,             # [deg/s] left after GYRO_BIAS_Z_DPS
             gyro_bias_drift=0.002,     # [deg/s per sqrt(s)] random walk
             gyro_noise=0.10,           # [deg/s] rms per sample
             gyro_scale=0.02,           # +2 % scale-factor error
             mag_noise=0.5,             # [uT] rms per axis per sample (LSM303D: 5 mgauss)
             mag_glitch_p=0.003, mag_glitch=4.0,
             hard_iron=(0.0, 0.0, 0.0), soft_iron=0.0,
             motor_field=(1.6, 1.1, 0.4),   # [uT] at full wheel torque, body-fixed (ASSUMED)
             tilt_static=(0.0, 0.0),    # roll, pitch [deg]
             swing_deg=0.0, swing_hz=0.8, swing_seen_by_accel=False,
             acc_noise=400.0,           # counts rms (wheel vibration), about 24 mg
             jitter=True, dup_p=0.05, z_down=False, pulse=None):
    rng = np.random.default_rng(seed)
    n = int(round(T / dt))
    # control-cycle period: 20 ms, one longer cycle every 250 ms when the telemetry stream blocks
    dts = np.full(n, dt)
    if jitter:
        dts[::12] = 0.032
    t = np.cumsum(dts) - dts[0]
    psi, w = slew_profile(np.arange(n) * dt, steps)
    psi = np.interp(t, np.arange(n) * dt, psi)
    w = np.gradient(psi, t)
    acc = np.gradient(w, t)                      # [deg/s^2] -> wheel torque -> motor current
    torque = np.clip(acc / 80.0, -1.0, 1.0)

    # gyro (platform sign)
    bias = gyro_bias + np.cumsum(rng.normal(0, gyro_bias_drift * math.sqrt(dt), n))
    rate = (1 + gyro_scale) * w + bias + rng.normal(0, gyro_noise, n)
    rate = np.round(rate / 0.00875) * 0.00875

    # attitude: yaw about the vertical, then tilt about body x (roll) and y (pitch)
    roll = np.full(n, tilt_static[0]) + swing_deg * np.sin(2 * np.pi * swing_hz * t)
    pitch = np.full(n, tilt_static[1]) + 0.6 * swing_deg * np.cos(2 * np.pi * swing_hz * t)
    hc = np.radians(heading0 + psi)
    m_w = np.array([b_h, 0.0, -b_v])             # world = (north, west, up)
    up_w = np.array([0.0, 0.0, 1.0])
    mag = np.zeros((n, 3)); up = np.zeros((n, 3))
    for i in range(n):
        c, s = math.cos(hc[i]), math.sin(hc[i])
        r_level = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])   # columns = body x, y, z in world
        cr, sr = math.cos(math.radians(roll[i])), math.sin(math.radians(roll[i]))
        cp, sp = math.cos(math.radians(pitch[i])), math.sin(math.radians(pitch[i]))
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        r_wb = r_level @ rx @ ry
        mag[i] = r_wb.T @ m_w
        if swing_seen_by_accel or swing_deg == 0.0:
            up[i] = r_wb.T @ up_w
        else:  # a freely swinging body feels its weight along the string: the accelerometer sees only the static tilt
            crs, srs = math.cos(math.radians(tilt_static[0])), math.sin(math.radians(tilt_static[0]))
            cps, sps = math.cos(math.radians(tilt_static[1])), math.sin(math.radians(tilt_static[1]))
            rxs = np.array([[1, 0, 0], [0, crs, -srs], [0, srs, crs]])
            rys = np.array([[cps, 0, sps], [0, 1, 0], [-sps, 0, cps]])
            up[i] = (rxs @ rys).T @ up_w
    # magnetometer errors
    a_soft = np.array([[1 + soft_iron, 0.3 * soft_iron, 0], [0.3 * soft_iron, 1 - soft_iron, 0], [0, 0, 1]])
    mag = mag @ a_soft.T + np.array(hard_iron)
    mag += np.outer(torque, np.array(motor_field))
    if pulse:                                    # (t0, t1, (dx, dy, dz)) an external field for a while
        sel = (t >= pulse[0]) & (t < pulse[1])
        mag[sel] += np.array(pulse[2])
    mag += rng.normal(0, mag_noise, (n, 3))
    gl = rng.random(n) < mag_glitch_p
    mag[gl, rng.integers(0, 2, gl.sum())] += rng.choice([-1, 1], gl.sum()) * mag_glitch
    # the magnetometer runs at its own 50 Hz: now and then the loop reads the same sample twice
    new = np.ones(n, dtype=int)
    for i in range(1, n):
        if rng.random() < dup_p:
            mag[i] = mag[i - 1]; new[i] = 0
    accel = up * G1 + rng.normal(0, acc_noise, (n, 3))
    sw_x = np.gradient(roll, t) + 0.5 + rng.normal(0, 0.1, n)      # the other two gyro axes, with a bias
    sw_y = np.gradient(pitch, t) - 0.3 + rng.normal(0, 0.1, n)
    if z_down:
        mag[:, 1] *= -1; mag[:, 2] *= -1; accel[:, 1] *= -1; accel[:, 2] *= -1
    return dict(t=t, dt=dts, psi=psi, w=w, rate=rate, mag=mag, new=new, accel=accel, sw_x=sw_x, sw_y=sw_y,
                heading_true=(heading0 + psi) % 360.0, torque=torque, bias=bias)


def lines(d):
    out = []
    for i in range(len(d["t"])):
        m, a = d["mag"][i], d["accel"][i]
        out.append("%.6f %.5f %.4f %.4f %.4f %.4f %.4f %d %.1f %.1f %.1f" % (
            d["dt"][i] if i else 0.0, d["rate"][i], d["sw_x"][i], d["sw_y"][i], m[0], m[1], m[2], d["new"][i], a[0], a[1], a[2]))
    return out


COLS = ["ok", "ekf_abs", "ekf_rel", "mag_abs", "bias", "sigma", "innov", "gain", "varscale", "tilt", "used", "rejected",
        "reacq", "std_est", "std_filt", "std_raw", "ekf_rate", "tilt_used", "sigma_bias"]


def wrap(x):
    return (np.asarray(x) + 180.0) % 360.0 - 180.0
