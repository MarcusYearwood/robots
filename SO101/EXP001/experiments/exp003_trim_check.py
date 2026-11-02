"""Verify P=32 + integral trim: negative moves on gravity joints, error vs time after arrival. data/003_trim/summary.json"""
import sys, json, time, pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "lib"), str(ROOT / "scripts")]
from so101 import Arm
a = Arm(); res = {}
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=15)
a.goto(shoulder_lift=-10, elbow_flex=59, wrist_flex=-42, speed=15)
for j in ("shoulder_lift", "elbow_flex", "wrist_flex"):
    for sign in (-1, +1):
        tgt = a.pos()[j] + sign * 12; t0 = time.time(); a.call("goto", joints={j: tgt}, speed=40); errs = {}
        for tt in (0.5, 1.0, 2.0, 3.0):
            time.sleep(max(0, t0 + 12/40 + tt - time.time())); errs[f"+{tt}s"] = round(a.pos()[j] - tgt, 2)
        res[f"{j}|{'+' if sign>0 else '-'}"] = errs; print(j, sign, errs, flush=True)
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=20); a.park()
import os; os.makedirs(ROOT/"data/003_trim", exist_ok=True); json.dump(res, open(ROOT/"data/003_trim/summary.json","w"), indent=1)
