"""Read-only probe of the SO-101 follower. Never enables torque or writes any register."""
import json, sys
from pathlib import Path
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

PORT = "/dev/tty.usbmodem5C821086721"
CAL = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/my_awesome_follower_arm.json"
cal = {k: MotorCalibration(**v) for k, v in json.loads(CAL.read_text()).items()}
names = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
motors = {n: Motor(i+1, "sts3215", MotorNormMode.RANGE_0_100 if n=="gripper" else MotorNormMode.RANGE_M100_100) for i,n in enumerate(names)}
bus = FeetechMotorsBus(port=PORT, motors=motors, calibration=cal)
bus.connect()
try:
    raw = bus.sync_read("Present_Position", normalize=False)
    norm = bus.sync_read("Present_Position")
    print(f"{'joint':14}{'raw':>6}{'min':>6}{'max':>6}{'span°':>8}{'norm':>8}")
    for n in names:
        c = cal[n]
        print(f"{n:14}{raw[n]:6}{c.range_min:6}{c.range_max:6}{(c.range_max-c.range_min)*360/4096:8.1f}{norm[n]:8.1f}")
    for reg in ["Torque_Enable","Present_Voltage","Present_Temperature","Present_Load","Present_Current","Max_Torque_Limit","Acceleration","Maximum_Acceleration","P_Coefficient","Min_Position_Limit","Max_Position_Limit","Operating_Mode","Lock"]:
        try: print(reg, bus.sync_read(reg, normalize=False))
        except Exception as e: print(reg, "ERR", e)
finally:
    bus.disconnect(disable_torque=False)
