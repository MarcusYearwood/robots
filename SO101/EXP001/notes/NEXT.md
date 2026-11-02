# Next steps (ordered). Update/remove as done.
1. Ask user: where was the gum pack released in pick_gum; what was random1 meant to show; where should "place" targets be.
2. Redo visual Jacobian (exp004) on the gum pack at a pre-grasp pose with textured scene; verify with phase-corr confidence > 0.3. Then build a visual-servo loop: center target in image via pan/wrist_flex/lift, 0.15 s servo lag ⇒ gain ≤ ~0.5/iteration.
3. Autonomous pick of the gum pack: search (pan sweep at look pose) → detect pack (green/white colour blob, cv2) → pre-grasp above it → descend → close gripper until stall (~36) → lift → place. First attempt slow (≤20 u/s), user watching, ESTOP file ready.
4. Contact/collision detection from load/current: record load vs pose baseline (telemetry on, free motion), flag spikes > baseline+Δ; use as guard before faster moves. Also map unsafe zones (faucet, counter surface) as extra guard in safety.py.
5. Optional: forward kinematics (no URDF locally; see `LeRobotMaterials/3. Technical Specifications & Drawings/urdf model download link.txt`) for Cartesian goals and a table-plane guard.
6. Optional: raise tick to 100 Hz; add `snap`/camera into armd so vision+control share timing; log demos at higher Hz for learning (policy/record) via lerobot dataset tools.
