"""Rerun monitor for the arm daemon: camera, joints (present/target/leader), torque, faults, plus an override hotkey.

Talks to armd_override.py over its unix socket (read-only `state`, plus `override`). It never touches the serial ports.

    o   toggle override (ON = leader arm drives the follower, OFF = hold and give control back to the agent)
    Ctrl+C  quit the monitor (does not change daemon state)

Run (lerobot env), with armd_override.py already running:
    python SO101/devtools/monitor.py [--camera 0] [--no-camera]
"""
import argparse, json, socket, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "EXP001" / "scripts"))
import safety as S  # noqa: E402  (for SOCK / JOINTS)

import rerun as rr  # noqa: E402

HZ = 10


def call(op, **kw):
    s = socket.socket(socket.AF_UNIX); s.settimeout(2); s.connect(S.SOCK)
    f = s.makefile("rw"); f.write(json.dumps({"op": op, **kw}) + "\n"); f.flush()
    r = json.loads(f.readline()); s.close()
    if not r["ok"]: raise RuntimeError(r["error"])
    return r["result"]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--camera", type=int, default=0); ap.add_argument("--no-camera", action="store_true")
    a = ap.parse_args()

    rr.init("so101_monitor", spawn=True)
    cap = None
    if not a.no_camera:
        import cv2
        cap = cv2.VideoCapture(a.camera)
        if not cap.isOpened(): print("WARNING: camera not available (is another process holding it?); continuing without."); cap = None

    toggle = {"want": None}
    try:
        from pynput import keyboard
        def on_press(key):
            if getattr(key, "char", None) == "o": toggle["want"] = "toggle"
        keyboard.Listener(on_press=on_press).start()
    except Exception as e:
        print(f"WARNING: hotkey unavailable ({e}); use `armctl`-style calls to toggle override.")

    last_fault, last_ov = None, None
    while True:
        t0 = time.time()
        try:
            st = call("state")
        except Exception as e:
            print(f"daemon not reachable: {e}"); time.sleep(1); continue

        if toggle["want"]:
            toggle["want"] = None
            try: call("override", on=not st.get("override", False)); st = call("state")
            except Exception as e: print("override failed:", e); rr.log("events", rr.TextLog(f"override failed: {e}"))

        for j in S.JOINTS:
            rr.log(f"joints/{j}/present", rr.Scalar(st["present"][j]))
            if st["target"].get(j) is not None: rr.log(f"joints/{j}/target", rr.Scalar(st["target"][j]))
            if st.get("leader", {}).get(j) is not None: rr.log(f"joints/{j}/leader", rr.Scalar(st["leader"][j]))
        rr.log("status/torque", rr.Scalar(1.0 if st["torque"] else 0.0))
        rr.log("status/override", rr.Scalar(1.0 if st.get("override") else 0.0))
        for j, t in st["health"].get("temp", {}).items(): rr.log(f"health/temp_C/{j}", rr.Scalar(t))
        if st["fault"] != last_fault:
            last_fault = st["fault"]; rr.log("events", rr.TextLog(f"fault: {last_fault}" if last_fault else "fault cleared"))
        if st.get("override") != last_ov:
            last_ov = st.get("override"); rr.log("events", rr.TextLog("override ON (leader in control)" if last_ov else "override OFF (agent may act)"))

        if cap is not None:
            ok, frame = cap.read()
            if ok: rr.log("camera/handeye", rr.Image(frame[:, :, ::-1]))

        time.sleep(max(0, 1 / HZ - (time.time() - t0)))


if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: pass
