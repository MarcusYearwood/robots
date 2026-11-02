# Journal (append-only, newest last). One entry per experiment: goal → result → what changed.

- **000 look-around**: wrist camera views at pans -35/0/35/70. Kitchen counter. Images in data/000_look_around/.
- **001 dynamics**: per-joint ramps at 20/40/80 u/s ±amp, telemetry on. Lag 0.15 s; gravity sag on lift/elbow/wrist_flex. Changes: speed-aware stall threshold, TICK_HZ 60, `gain` op, wait() = reached-or-settled. Data: data/001_dynamics/.
- **002 gains**: P 16/24/32 on lift/elbow/wrist_flex → P32 halves sag, no jitter. **003 trim**: P32 + integral trim → ≤0.3 units. Both now default in safety.py (P_GAIN, TRIM_*).
- **004 visual jacobian**: low-confidence numbers (smooth scene); camera latency 0.06 s. TODO redo on textured target.
- **demo prep**: user will teleop demos; daemon stopped; added scripts/teleop_record.py.
- **demo pick_gum**: user teleoperated picking up a gum pack. Reviewed states + frames. Added experiments/analyze_demo.py and facts above. Daemon restarted after.
- **rule**: I must leave armd STOPPED whenever the user may run teleop/record (the port is exclusive). Restart only when I'm about to move the arm, and say so. (random1 failed once because I'd restarted it.)
- **demo random1**: free-motion demo, gripper closed. Revealed full reach envelope; widened elbow soft limit to −97.
- **wrap-up (session end)**: consolidated KNOWLEDGE/NEXT/README. Daemon stopped, arm parked/torque off, port free. Nothing modified outside SO101/EXP001 (only /tmp/so101_armd.sock created at runtime; calibration files only read).
