# EXP001 — operating the SO-101 follower arm (Claude-driven)

START HERE. Then read `notes/KNOWLEDGE.md` (what is known) and `notes/NEXT.md` (what to do next). Log new work in `notes/JOURNAL.md`.

## Layout (keep it this way; nothing outside this folder is modified)
- `scripts/safety.py` — ALL limits/gains/thresholds. Single source of truth for guards. Edit here, nowhere else.
- `scripts/armd.py` — daemon, sole owner of the follower serial port; every motion passes through it (soft limits, speed cap, rate limiter, health/stall/ESTOP, park on exit).
- `scripts/armctl.py` — CLI client (`state | goto j=v … [--speed N|--wait] | park | hold | clear | relax`).
- `scripts/teleop_record.py` — user's teleop command + logging to `data/demos/<name>/` (needs daemon STOPPED).
- `scripts/probe_readonly.py` — read-only register dump.
- `lib/so101.py` — Python client for experiments: `Arm` (goto/wait/trace/telemetry/park) and `Camera` (threaded, rotated upright).
- `experiments/expNNN_*.py` — one script per experiment; writes `data/NNN_*/`. `analyze_demo.py` summarizes demos.
- `data/` — raw outputs (000 look-around, 001 dynamics, 002 gains, 003 trim, 004 visual jacobian, demos/pick_gum, demos/random1).
- `notes/` — KNOWLEDGE (facts), JOURNAL (append-only log), NEXT (open items). `logs/armd.log` — daemon log.

## Run
Python: `/opt/miniconda3/envs/lerobot/bin/python` (3.10, lerobot installed editable from ../lerobot; run experiments with `-I`, they add lib/ and scripts/ to sys.path themselves; `armctl.py` runs without `-I`).
Start daemon (torque stays OFF until the first `goto`):
`cd scripts && nohup /opt/miniconda3/envs/lerobot/bin/python -I -c "import sys;sys.path.insert(0,'.');import armd;armd.main()" > ../logs/armd.log 2>&1 &`
Stop (parks, torque off, frees port): `kill $(pgrep -f armd.main)` and wait ~15 s.

## Hard rules
1. The serial port is exclusive. Daemon must be STOPPED whenever the user may teleop/record (it fails with "Incorrect status packet"). Tell the user before starting it.
2. Never write Goal_Position outside armd. Never bypass safety.py. Never cut torque away from the folded pose except via `park` (arm falls under gravity).
3. Start every session with `state`, read temps; stop if any ≥ 55 °C and let it cool (idle after experiments was ~52-54 °C).
4. Test with tiny moves first; an out-of-range target is clamped but still starts a real move toward the clamp.
5. Add a fact to KNOWLEDGE.md only with an experiment/demo tag; fix wrong facts in place and note it in JOURNAL.
