# SO-101 knowledge base (living model). One fact per bullet, tagged with the experiment that proved it. Fix in place if contradicted.

## Hardware / setup
- Follower on `/dev/tty.usbmodem5C821086721`, leader `/dev/tty.usbmodem5C821102991`; calibration ids `my_awesome_{follower,leader}_arm` in `~/.cache/huggingface/lerobot/calibration/` (read-only for us). Wrist camera = OpenCV index 0, 640x480, raw image is rotated 90° (lib.Camera rotates). Mac webcam is index 1. [setup, 000]
- Units: lerobot normalized: joints −100..100 across the calibrated range, gripper 0..100. Calibrated spans (deg): pan 223, lift 208, elbow 193, wrist_flex 193, roll 336, gripper 128. Servo Min/Max_Position_Limit registers equal the calibration range (hardware backstop). [probe]
- Supply ≈12.2 V; motors idle 48–50 °C, ~52–55 °C after a few minutes of experiments. Torque OFF at start; arm rests folded (lift −99, elbow +99, wrist_flex ~69). [probe, 001]

## Scene (kitchen counter, grey speckled granite look) [000, pick_gum, random1]
- Arm base on counter next to sink. Straight ahead (pan≈0, wrist_flex≈−40): faucet+sink, stainless fridge, stove, wall oven. Dish-soap bottle (clear) left-of-center on an orange mat; blue pump bottle on an orange mat at pan≈+35; wall at pan≈+70; bonsai/toaster at pan≈−35. Power brick + cable loop beside the base. Extra Spearmint gum pack (flat) lay near the brick (pick_gum, again visible in random1).
- Hazards for reaching: faucet/sink edge, bottles, cables near the base.

## Joint conventions
- Lifting arm: lift ↑, elbow ↓. wrist_flex ↓ tilts camera up/away from counter. pan ↑ toward blue-pump side. Fully extended reach (human): lift +85…91, elbow −96, wrist_flex ~7; pan swept −14…+87. [000, random1]
- Soft limits in safety.py: pan ±92, lift −99…90, elbow −97…99.5 (widened from −92 after random1), wflex ±92, roll ±92, gripper 1…99.

## Dynamics (control-relevant) [001–003]
- Loop 60 Hz (tick ≈1.3 ms, so >100 Hz possible). Servo tracking lag ≈0.15 s at any speed: mean |setpoint−present| ≈ 0.15·speed; settle 0.1–0.4 s after ramp. Camera cmd→visible change ≈0.06 s.
- Gravity sag: at servo P=16 lift/elbow/wrist_flex stop 2–4 units short on negative-direction moves. P=32 (applied at torque-on, `P_GAIN`) halves it with no jitter; software integral trim (`TRIM_*`, gravity joints only, never gripper) → ≤0.3 units within ~1 s. pan/roll/gripper are accurate (±0.5) at defaults.
- Peak servo load ≤ ~370/1000 at 80 u/s; torque cap 500 (gripper 300). Load register: bit10 = direction, low 10 bits = magnitude.
- Human teleop envelope: p95 30–65 u/s, peaks 120–200 u/s. Our MAX_SPEED 80 covers normal motion; goto(duration=…) gives synchronized joint arrival.
- Stall threshold is speed-aware (12 + 0.25·speed units sustained 1.5 s). wait() = setpoint reached AND arm still; raises if >8 units short.

## Task knowledge
- **Gum pack grasp** (pick_gum): open gripper to 60; camera sees pack from above; grasp pose ≈ pan 25, lift 13, elbow 23, wrist_flex 73, roll ~0 (arm extended); closing stalls at gripper ≈ **36** (pack width). Carried to pan 21–27, lift −41, elbow −23, wflex 59, released ~81 s. Final resting place of the pack is unknown.
- random1 was free arm motion with gripper closed (2) the whole time; no grasp. User's phrase "picked it up" is ambiguous (likely the leader arm).
- Visual Jacobian (px per unit, upright-unrotated frame) is LOW CONFIDENCE (phase-corr 0.02–0.13, smooth scene): pan ≈(4.6,1.4), wflex ≈(−1.4,7.4), roll ≈(−1.9,−2.5), lift ≈(−2.5,10), elbow unusable. Redo on a textured target (the gum pack). [004]
- Demo recording runs only ~16 Hz (camera read in loop); frames saved ~5 Hz. Fine for review, thin for training. [pick_gum, random1]
