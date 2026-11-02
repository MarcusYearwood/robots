"""Hard guards for the SO-101 follower. Every motion command passes through these; nothing else writes Goal_Position.

Units: lerobot normalized (RANGE_M100_100 for arm joints, RANGE_0_100 for gripper) which map linearly onto the
calibrated raw range [range_min, range_max] in my_awesome_follower_arm.json. Motors also have those same limits burned
into Min/Max_Position_Limit (verified), so the servos themselves refuse to go beyond calibration.
"""
import json
from pathlib import Path

PORT = "/dev/tty.usbmodem5C821086721"
CAL_PATH = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/my_awesome_follower_arm.json"
EXP_DIR = Path(__file__).resolve().parent.parent
SOCK = "/tmp/so101_armd.sock"
ESTOP_FILE = EXP_DIR / "ESTOP"          # `touch` this file => daemon holds then parks immediately

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Soft limits (normalized units), tighter than calibration. Lift/elbow rest pose sits at ~-99/+99 so they get less margin.
SOFT = {
    "shoulder_pan":  (-92.0, 92.0),
    "shoulder_lift": (-99.0, 90.0),
    "elbow_flex":    (-97.0, 99.5),
    "wrist_flex":    (-92.0, 92.0),
    "wrist_roll":    (-92.0, 92.0),
    "gripper":       (1.0, 99.0),
}
# Folded rest pose the arm was found in (all joints within soft limits). Park target; torque is cut only once here.
REST = {"shoulder_pan": 0.0, "shoulder_lift": -99.0, "elbow_flex": 99.0, "wrist_flex": 69.0, "wrist_roll": 0.0, "gripper": 1.5}

MAX_SPEED = 80.0        # units/s hard ceiling per joint (full range 200 units => 2.5 s minimum)
DEFAULT_SPEED = 30.0    # units/s when caller doesn't say
TICK_HZ = 60
SERVO_ACCEL = 40        # RAM reg Acceleration (0-254; default 254 = abrupt)
SERVO_GOAL_VELOCITY = 1000   # RAM reg steps/s cap (~88 deg/s)
SERVO_TORQUE_LIMIT = 500     # RAM reg, 50% of 1000
GRIPPER_TORQUE_LIMIT = 300
TEMP_STOP_C = 62        # hold+park above this
TEMP_WARN_C = 55
VOLT_MIN, VOLT_MAX = 10.0, 13.0   # volts (reg is 0.1 V units)
STALL_ERR = 12.0        # |setpoint - present| in units
STALL_SECONDS = 1.5
IDLE_PARK_S = 600       # no client commands for this long => park + torque off


def load_cal():
    return json.loads(CAL_PATH.read_text())


def clamp_target(joint, value):
    lo, hi = SOFT[joint]
    return min(max(float(value), lo), hi)


def validate_targets(targets: dict) -> dict:
    """Reject unknown joints / non-finite numbers; clamp the rest into soft limits."""
    out = {}
    for j, v in targets.items():
        if j not in SOFT:
            raise ValueError(f"unknown joint {j!r}")
        v = float(v)
        if v != v or abs(v) == float("inf"):
            raise ValueError(f"non-finite target for {j}")
        out[j] = clamp_target(j, v)
    return out


def step_toward(current, target, speed, dt):
    """Rate limiter: move `current` toward `target` by at most min(speed, MAX_SPEED)*dt."""
    step = min(abs(speed), MAX_SPEED) * dt
    d = target - current
    return target if abs(d) <= step else current + step * (1 if d > 0 else -1)

# Servo P coefficient (lerobot default 16). 32 halves gravity sag, no jitter measured [exp002]. Not applied to gripper/pan/roll.
P_GAIN = {"shoulder_lift": 32, "elbow_flex": 32, "wrist_flex": 32}
# Software integral trim for residual gravity sag. Never on the gripper (would squeeze objects harder).
TRIM_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
TRIM_GAIN, TRIM_MAX, TRIM_DEADBAND = 0.12, 4.0, 0.5
