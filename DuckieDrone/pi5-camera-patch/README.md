# Pi 5 camera patch for the Duckiedrone (ente)

**Problem:** on a Raspberry Pi 5, `driver-camera` crash-loops ("Could not read image from camera").
`duckietown/dt-duckiebot-interface:ente-arm64v8` builds libcamera from the upstream tag `v0.4.0`
with only the `rpi/vc4` (Pi 4) pipeline, so libcamera inside the container sees no cameras.

**Patch:** `../dt-duckiebot-interface` (submodule, fork branch `pi5-camera-pisp`, commit `86d5d85`):
build libcamera `v0.5.2+rpt20250903` with `rpi/vc4,rpi/pisp`, and add `nlohmann-json3-dev` for libpisp.

**Build** (on the drone, tagged `local/dt-duckiebot-interface:ente-pi5`): `./build.sh`, then `./build.sh status`

**Verify:** `./verify.sh baseline | image | capture | system | rollback`. Evidence is saved to `results/`.

Results (2026-11-14, Pi 5 + OV5647 160°):

| Stage | Result |
|---|---|
| baseline | official image crash-looping; host `rpicam-still` reference saved |
| image | `pisp` + `vc4` pipelines and IPAs present; picamera2 0.3.36 imports |
| capture | ov5647 enumerated; 640x480 frame matches host reference; 32 fps; valid timestamps |
| system | 10 min soak: healthy, 0 restarts, no throttling; topic at 15.9 Hz; dashboard shows video |

**Note:** `dts duckiebot update` may restore the official image. If it does, run `./verify.sh system` to swap the patched one back in.
