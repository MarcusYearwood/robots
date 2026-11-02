# SO-101 Setup Notes

Running notes on config for the Hiwonder/HiRobot SO-101 kit. Source material is in `SO101/LeRobotMaterials/`. The manual is `1. Tutorials/SO-ARM101 Open-Source 6-Axis Robotic Arm User Manual.pdf`. It only covers Windows and Ubuntu, so macOS differences are noted here.

## Kit

- Pre-assembled kit, so servo ID setting is skipped (manual section 3.5).
- One camera only (wrist/arm camera). No second fixed camera.
- The accessory pack of spare horns, screws and standoffs is not needed. Keep it for repairs.

## Environment (macOS, Apple Silicon)

- Miniconda installed with the `.pkg` installer.
- Conda env: `lerobot` (`python=3.10.18`, `ffmpeg=7.1.1`, from conda-forge).
- Source: `lerobot.zip` from `2. Software Tools & Source Code/Source Code/`, unzipped and installed with `pip install -e ".[feetech]"`.
- Activate before every command: `conda activate lerobot`.

## Ports

Found with `python -m lerobot.find_port`. Re-run if you change USB ports or computers.

| Device | Port | Status |
| --- | --- | --- |
| Follower arm | `/dev/tty.usbmodem5C821086721` | confirmed |
| Leader arm | `/dev/tty.usbmodem5C821102991` | confirmed |

`/dev/cu.usbmodem...` is the same device as `/dev/tty.usbmodem...`.

## Cameras

- Camera index is not fixed. Confirm it after replugging or changing ports.
- Use only one camera in the config (the manual's examples use two):
  `--robot.cameras="{ handeye: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}"`
- Arm camera (`handeye`) is index `0` (640x480). Index `1` is the Mac's built-in webcam. Confirmed with `python -m lerobot.find_cameras opencv` (images land in `SO101/lerobot/outputs/captured_images`).
- Camera was black on the hub at first and fuzzy until the lens was focused. Check the lens film, the focus barrel, and plug it straight into the Mac if it goes black.
- macOS asks for camera permission for the terminal app.

## Progress

- [x] Miniconda and `lerobot` env
- [x] Follower port found
- [x] Leader port found
- [x] Calibrate follower (manual 4.1), id `my_awesome_follower_arm`
- [x] Calibrate leader (manual 4.1), id `my_awesome_leader_arm`
- [x] Teleoperate without camera (manual 4.2), works
- [ ] Teleoperate with camera (manual 4.3)
- [ ] Record a dataset (manual 5)

## Ports through the hub

Replugging through the multi-port USB hub kept the same names (checked with `ls /dev/tty.usbmodem*`). Recheck after any replug.

## Gotchas

- Calibration: on the leader, sweep the gripper (trigger) fully open and closed. Check the gripper row spans 1,000+ steps before pressing Enter. Otherwise the follower gripper barely moves.
- Calibration pose: upper arm vertical, forearm horizontal, wrist straight, gripper level.

## Commands

Teleoperate without camera. Ports are filled in. Ids are the manual's defaults and should match what you use in calibration:

```
python -m lerobot.teleoperate \
  --robot.type=so101_follower --robot.port=/dev/tty.usbmodem5C821086721 --robot.id=my_awesome_follower_arm \
  --teleop.type=so101_leader --teleop.port=/dev/tty.usbmodem5C821102991 --teleop.id=my_awesome_leader_arm
```

## Devtools (`SO101/devtools/`)

Untested on hardware unless noted. None of these modify stock lerobot or `EXP001/`.

- `teleoperate_override.py`: stock Rerun teleop with an `o` hotkey toggling leader override (eases in, no jump). Standalone; do not run while a daemon owns the follower port.
- `armd_override.py`: EXP001's daemon plus a leader-arm override (`{"op":"override","on":true|false}`). Run instead of `armd.py`. Same socket, so `armctl.py` still works; `goto` is refused while override is on. Dry-run tested with stubbed hardware only.
- `monitor.py`: Rerun viewer for the daemon (camera, present/target/leader joints, torque, temps, faults) with an `o` hotkey to toggle override. Needs `armd_override.py` running.
