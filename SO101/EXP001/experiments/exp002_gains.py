"""Does a higher servo P coefficient remove gravity sag on lift/elbow/wrist_flex without jitter? Output data/002_gains/summary.json"""
import sys, json, time, pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "lib"), str(ROOT / "scripts")]
from so101 import Arm
import numpy as np
a = Arm(); out = {}
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=15)
a.goto(shoulder_lift=-10, elbow_flex=59, wrist_flex=-42, speed=15)
a.telemetry(True)
try:
    for P in (16, 24, 32):
        for j in ("shoulder_lift", "elbow_flex", "wrist_flex"): a.call("gain", joint=j, p=P)
        for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
            for sign in (-1, +1):
                p0 = a.pos()[j]; tgt = p0 + sign * 12; a.call("trace", clear=True); t0 = time.time()
                a.call("goto", joints={j: tgt}, speed=40); time.sleep(12 / 40 + 1.5)
                tr = a.trace(since=t0); pres = np.array([r[1][j] for r in tr]); t = np.array([r[0] for r in tr]) - t0
                hold = pres[t > t[-1] - 0.8]
                out[f"P{P}|{j}|{'+' if sign>0 else '-'}"] = dict(ss_err=round(float(hold.mean() - tgt), 2), hold_jitter_std=round(float(hold.std()), 3),
                    hold_pp=round(float(hold.max() - hold.min()), 2), overshoot=round(float(((pres - tgt) * sign).max()), 2))
        print(P, a.state()["health"]["temp"], flush=True)
finally:
    for j in ("shoulder_lift", "elbow_flex", "wrist_flex"): a.call("gain", joint=j, p=16)   # restore lerobot default
    a.telemetry(False); a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=20); a.park()
json.dump(out, open(ROOT / "data/002_gains/summary.json", "w"), indent=1)
for k, v in out.items(): print(k, v)
