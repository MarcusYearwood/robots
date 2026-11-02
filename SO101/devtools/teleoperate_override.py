"""lerobot.teleoperate (Rerun viewer) plus an override toggle. Stock lerobot is untouched; this reuses its config and imports.

Hotkey (terminal must be focused; macOS may ask for Input Monitoring permission for pynput):
    o   toggle override.  ON  = leader arm drives the follower (eases in from the follower's current pose, no jump)
                          OFF = follower holds its last commanded pose; leader is ignored
    Ctrl+C  quit

Starts with override OFF, so nothing moves until you press `o`. Do NOT run while EXP001's armd.py is running: both would own the follower port.
Note: on exit lerobot disables torque, so a raised arm will drop. Fold it or hold it first.

Run (same flags as lerobot.teleoperate):
    python SO101/devtools/teleoperate_override.py \
      --robot.type=so101_follower --robot.port=/dev/tty.usbmodem5C821086721 --robot.id=my_awesome_follower_arm \
      --robot.cameras="{ handeye: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
      --teleop.type=so101_leader --teleop.port=/dev/tty.usbmodem5C821102991 --teleop.id=my_awesome_leader_arm \
      --display_data=true
"""

import logging
import threading
import time
from dataclasses import asdict
from pprint import pformat

import draccus
import rerun as rr

# Importing the stock module registers all robot/teleop config types with draccus and gives us the config dataclass.
from lerobot.teleoperate import TeleoperateConfig
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import _init_rerun, log_rerun_data

TOGGLE_KEY = "o"
RAMP_SPEED = 40.0  # units/s while easing from the follower's pose to the leader's pose
RAMP_TOL = 2.0     # units; once every joint is this close, follow the leader directly


def step_toward(cur, tgt, speed, dt):
    step = speed * dt
    d = tgt - cur
    return tgt if abs(d) <= step else cur + step * (1 if d > 0 else -1)


def start_key_listener(override: threading.Event):
    try:
        from pynput import keyboard
    except Exception as e:  # pragma: no cover
        print(f"WARNING: pynput unavailable ({e}); override can't be toggled and will stay OFF.")
        return None

    def on_press(key):
        if getattr(key, "char", None) == TOGGLE_KEY:
            override.clear() if override.is_set() else override.set()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    return listener


def teleop_loop(teleop, robot, fps, display_data=False, duration=None):
    override = threading.Event()
    listener = start_key_listener(override)

    # Hold wherever the follower is right now before anything else, so there's no jump when torque is on.
    cmd = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
    robot.send_action(cmd)

    dt = 1 / fps
    display_len = max(len(k) for k in robot.action_features)
    start = time.perf_counter()
    was_on, ramping = False, False
    try:
        while True:
            t0 = time.perf_counter()
            action = teleop.get_action()
            on = override.is_set()

            if on and not was_on:
                ramping = True
                logging.info("override ON: easing follower to leader pose")
                rr.log("override/events", rr.TextLog("override ON")) if display_data else None
            elif was_on and not on:
                logging.info("override OFF: holding")
                rr.log("override/events", rr.TextLog("override OFF")) if display_data else None
            was_on = on

            if on:
                if ramping:
                    cmd = {k: step_toward(cmd[k], action[k], RAMP_SPEED, dt) for k in action}
                    ramping = any(abs(cmd[k] - action[k]) > RAMP_TOL for k in action)
                else:
                    cmd = dict(action)
                robot.send_action(cmd)
            # OFF: send nothing; the follower keeps its last goal.

            if display_data:
                log_rerun_data(robot.get_observation(), action)
                rr.log("override/on", rr.Scalar(1.0 if on else 0.0))

            busy_wait(dt - (time.perf_counter() - t0))
            loop_s = time.perf_counter() - t0

            state = ("ON (easing in)" if ramping else "ON") if on else f"OFF  (press '{TOGGLE_KEY}' to take over)"
            print("\n" + "-" * (display_len + 10))
            print(f"{'NAME':<{display_len}} | {'LEADER':>7} | {'SENT':>7}")
            for k, v in action.items():
                print(f"{k:<{display_len}} | {v:>7.2f} | {cmd.get(k, float('nan')):>7.2f}")
            print(f"\noverride: {state:<40} time: {loop_s * 1e3:.2f}ms ({1 / loop_s:.0f} Hz)")

            if duration is not None and time.perf_counter() - start >= duration:
                return
            move_cursor_up(len(action) + 6)
    finally:
        if listener:
            listener.stop()


@draccus.wrap()
def main(cfg: TeleoperateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        _init_rerun(session_name="teleoperation_override")

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)
    teleop.connect()
    robot.connect()
    try:
        teleop_loop(teleop, robot, cfg.fps, display_data=cfg.display_data, duration=cfg.teleop_time_s)
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()
