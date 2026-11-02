"""Same teleop as `python -m lerobot.teleoperate`, but logs everything so Claude can review the demo afterwards.
Run with the armd daemon STOPPED (it owns the port + camera).  Usage:
  python scripts/teleop_record.py <demo_name> [--seconds 120]
Writes data/demos/<demo_name>/{states.jsonl, frames/NNNNN.jpg (~5 Hz), meta.json}. Ctrl-C to stop.
states.jsonl rows: {t, leader:{joint.pos}, follower_obs:{joint.pos}, action_sent:{joint.pos}}   (normalized units, same as armd)."""
import argparse, json, pathlib, time, cv2
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

ap = argparse.ArgumentParser(); ap.add_argument("name"); ap.add_argument("--seconds", type=float, default=120); ap.add_argument("--fps", type=int, default=30)
a = ap.parse_args()
out = pathlib.Path(__file__).resolve().parent.parent / "data/demos" / a.name; (out / "frames").mkdir(parents=True, exist_ok=True)
cam = {"handeye": OpenCVCameraConfig(index_or_path=0, width=640, height=480, fps=30)}
robot = SO101Follower(SO101FollowerConfig(port="/dev/tty.usbmodem5C821086721", id="my_awesome_follower_arm", cameras=cam))
leader = SO101Leader(SO101LeaderConfig(port="/dev/tty.usbmodem5C821102991", id="my_awesome_leader_arm"))
robot.connect(); leader.connect()
json.dump({"name": a.name, "start": time.time(), "fps": a.fps, "camera": "handeye idx0 640x480, raw (unrotated)"}, open(out / "meta.json", "w"))
f = open(out / "states.jsonl", "w"); t0 = time.time(); n = 0
try:
    while time.time() - t0 < a.seconds:
        t = time.perf_counter()
        act = leader.get_action(); obs = robot.get_observation(); sent = robot.send_action(act)
        row = {"t": round(time.time() - t0, 3), "leader": act, "follower_obs": {k: v for k, v in obs.items() if k.endswith(".pos")}, "action_sent": sent}
        f.write(json.dumps(row) + "\n")
        if n % 6 == 0: cv2.imwrite(str(out / "frames" / f"{n:06d}.jpg"), cv2.cvtColor(obs["handeye"], cv2.COLOR_RGB2BGR))
        n += 1; time.sleep(max(0, 1 / a.fps - (time.perf_counter() - t)))
except KeyboardInterrupt: pass
finally:
    f.close(); print(f"saved {n} steps to {out}")
    robot.disconnect(); leader.disconnect()
