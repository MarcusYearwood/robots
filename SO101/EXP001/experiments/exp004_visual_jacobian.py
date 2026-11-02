"""Image-space response of each joint at the look pose: dx,dy (px) and rotation per unit via phase correlation / ECC. Also camera latency.
Output data/004_visual_jacobian/{summary.json, *.jpg}"""
import sys, json, time, pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "lib"), str(ROOT / "scripts")]
from so101 import Arm, Camera
import cv2, numpy as np
OUT = ROOT / "data/004_visual_jacobian"
a = Arm(); cam = Camera()
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=15)
a.goto(shoulder_lift=-10, elbow_flex=59, wrist_flex=-35, speed=15)
def grab():
    time.sleep(0.4); return cv2.cvtColor(cam.frame(), cv2.COLOR_BGR2GRAY).astype(np.float32)
def shift(g0, g1):
    win = cv2.createHanningWindow(g0.shape[::-1], cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(g0, g1, win); return dx, dy, resp
res = {}; cam.save(OUT / "base.jpg")
for j, d in [("shoulder_pan", 6), ("wrist_flex", 6), ("wrist_roll", 8), ("shoulder_lift", 4), ("elbow_flex", 4)]:
    p0 = a.pos()[j]; g0 = grab()
    a.goto({j: p0 + d}, speed=20); g1 = grab(); a.goto({j: p0 - d}, speed=20); g2 = grab(); a.goto({j: p0}, speed=20)
    dx, dy, r = shift(g2, g1); n = 2 * d
    res[j] = dict(dx_per_unit=round(dx / n, 2), dy_per_unit=round(dy / n, 2), confidence=round(r, 2))
    cv2.imwrite(str(OUT / f"{j}_plus.jpg"), g1.astype(np.uint8)); print(j, res[j], flush=True)
# latency: command a fast small pan step and watch image start to change
g0 = grab(); a.call("goto", joints={"shoulder_pan": a.pos()["shoulder_pan"] + 10}, speed=60); t0 = time.time(); lat = None
while time.time() - t0 < 2:
    g = cv2.cvtColor(cam.frame(), cv2.COLOR_BGR2GRAY).astype(np.float32)
    if np.abs(g - g0).mean() > 3 and lat is None: lat = time.time() - t0; break
    time.sleep(0.01)
res["camera_cmd_to_image_change_s"] = lat; print("latency", lat)
a.wait(); a.goto(shoulder_pan=0, speed=30); cam.close()
a.goto(shoulder_lift=-50, elbow_flex=73, wrist_flex=30, speed=20); a.park()
json.dump(res, open(OUT / "summary.json", "w"), indent=1)
