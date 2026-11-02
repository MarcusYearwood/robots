"""Per-joint step/ramp tracking at several speeds. Output: data/001_dynamics/{raw.json,summary.json}.
Measures: tracking error during ramp (setpoint-present), settle time, steady-state error, overshoot, peak load."""
import sys, json, time, pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "lib"), str(ROOT / "scripts")]
from so101 import Arm
import numpy as np

OUT = ROOT / "data/001_dynamics"; a = Arm()
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=15)
a.goto(shoulder_lift=-10, elbow_flex=59, wrist_flex=-42, speed=15)
a.telemetry(True)
AMP = {"shoulder_pan": 15, "shoulder_lift": 12, "elbow_flex": 12, "wrist_flex": 15, "wrist_roll": 20, "gripper": 20}
raw, summ = {}, {}
for j in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]:
    if j == "gripper": a.goto(gripper=30, speed=30)
    for speed in [20, 40, 80]:
        for sign in (+1, -1):
            p0 = a.pos()[j]; tgt = p0 + sign * AMP[j]
            t0 = time.time(); a.call("trace", clear=True)
            a.call("goto", joints={j: tgt}, speed=speed)
            time.sleep(AMP[j] / speed + 1.0)     # ramp + settle window
            tr = a.trace(since=t0)
            if not tr: continue
            t = np.array([r[0] for r in tr]) - t0; pres = np.array([r[1][j] for r in tr]); sp = np.array([r[2][j] for r in tr])
            load = np.abs(np.array([(r[3][j] & 0x3FF) * (-1 if r[3][j] & 0x400 else 1) if r[3] else 0 for r in tr]))
            ramp_end = AMP[j] / speed
            err = np.abs(sp - pres)
            fin = pres[-5:].mean(); settled = np.where(np.abs(pres - tgt) < 1.5)[0]
            key = f"{j}|{speed}|{'+' if sign>0 else '-'}"
            summ[key] = dict(ramp_s=round(ramp_end, 2), max_track_err=round(float(err.max()), 2), mean_track_err_ramp=round(float(err[t < ramp_end].mean()), 2),
                             t_settle_after_ramp=round(float(t[settled[0]] - ramp_end), 2) if len(settled) else None,
                             ss_err=round(float(fin - tgt), 2), overshoot=round(float(((pres - tgt) * sign).max()), 2), peak_load=int(load.max()),
                             rate_hz=round(len(t) / (t[-1] - t[0]), 1))
            raw[key] = dict(t=t.tolist(), pres=pres.tolist(), sp=sp.tolist(), load=load.tolist())
    st = a.state(); print(j, "temp", st["health"]["temp"][j], flush=True)
a.telemetry(False)
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, gripper=1, speed=20)
a.park()
json.dump(raw, open(OUT / "raw.json", "w")); json.dump(summ, open(OUT / "summary.json", "w"), indent=1)
for k, v in summ.items(): print(k, v)
