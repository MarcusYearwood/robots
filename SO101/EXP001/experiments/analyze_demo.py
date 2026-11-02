"""Summarize a teleop demo: python analyze_demo.py <name> [t1 t2 ...] -> stats + timeline + contact_sheet.jpg (frames at given seconds)."""
import sys, json, pathlib, cv2, numpy as np
d = pathlib.Path(__file__).resolve().parent.parent / "data/demos" / sys.argv[1]
ts = [float(x) for x in sys.argv[2:]] or list(range(5, 120, 10))[:12]
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
rows = [json.loads(l) for l in open(d / "states.jsonl")]; t = np.array([r["t"] for r in rows])
F = np.array([[r["follower_obs"][j + ".pos"] for j in J] for r in rows])
print(f"{len(t)} steps, {t[-1]:.0f}s, {len(t)/t[-1]:.1f} Hz"); print("min", F.min(0).round(), "max", F.max(0).round())
v = np.abs(np.diff(F, axis=0)) / np.diff(t)[:, None]; print("p95 speed u/s", np.percentile(v, 95, axis=0).round(), "max", v.max(0).round())
tiles = []
for x in ts:
    i = int(np.argmin(abs(t - x))); im = cv2.rotate(cv2.imread(str(d / "frames" / f"{(i//6)*6:06d}.jpg")), cv2.ROTATE_90_CLOCKWISE)
    im = cv2.resize(im, (240, 320)); cv2.putText(im, f"t={x:g}s", (5, 20), 0, 0.6, (0, 255, 255), 2); tiles.append(im)
while len(tiles) % 6: tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(str(d / "contact_sheet.jpg"), np.vstack([np.hstack(tiles[i:i+6]) for i in range(0, len(tiles), 6)]))
