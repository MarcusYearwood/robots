#!/usr/bin/env bash
# Staged verification for the Pi 5 camera patch (dt-duckiebot-interface, branch pi5-camera-pisp).
# Runs from the Mac; everything on the drone goes over SSH.
#
#   ./verify.sh baseline   # stage 0: record current (broken) state
#   ./verify.sh image      # stage 1: patched image has rpi/pisp + matching libcamera
#   ./verify.sh capture    # stage 2: throwaway container captures real frames (stops driver-camera)
#   ./verify.sh system     # stage 3: driver-camera on patched image, healthy for $SOAK_MIN minutes
#   ./verify.sh rollback   # stage 5: put the official image back
#
# Env: DRONE (default duckie@amelia02.local), PATCHED (patched image tag), SOAK_MIN (default 10)
set -uo pipefail

DRONE="${DRONE:-duckie@amelia02.local}"
OFFICIAL="duckietown/dt-duckiebot-interface:ente-arm64v8"
PATCHED="${PATCHED:-local/dt-duckiebot-interface:ente-pi5}"
SOAK_MIN="${SOAK_MIN:-10}"
OUT="$(cd "$(dirname "$0")" && pwd)/results"
mkdir -p "$OUT"

on_drone() { ssh -o BatchMode=yes -o ConnectTimeout=10 "$DRONE" "$@"; }
pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*"; FAILED=1; }
FAILED=0

stage_baseline() {
    on_drone "docker inspect -f '{{.Config.Image}} restarts={{.RestartCount}} state={{.State.Status}}' driver-camera" \
        | tee "$OUT/baseline.txt"
    on_drone "docker logs --tail 5 driver-camera 2>&1" >> "$OUT/baseline.txt"
    on_drone "sudo rpicam-still -n -t 800 -o /tmp/ref.jpg >/dev/null 2>&1 && ls -l /tmp/ref.jpg" \
        && scp -q -o BatchMode=yes "$DRONE:/tmp/ref.jpg" "$OUT/host_reference.jpg" \
        && pass "host reference frame saved to results/host_reference.jpg" \
        || fail "host could not capture a reference frame"
}

stage_image() {
    local r
    r=$(on_drone "docker run --rm --entrypoint sh $PATCHED -c '
        ls /usr/share/libcamera/pipeline/rpi/;
        ls /usr/lib/aarch64-linux-gnu/libcamera/ipa/ 2>/dev/null || ls /usr/lib/libcamera/ipa/ 2>/dev/null;
        python3 -c \"import libcamera, importlib.metadata as m; print(\\\"picamera2\\\", m.version(\\\"picamera2\\\"))\"'" 2>&1)
    echo "$r" | tee "$OUT/image.txt"
    grep -qx pisp <<<"$r" && pass "rpi/pisp pipeline present" || fail "rpi/pisp pipeline missing"
    grep -qx vc4 <<<"$r" && pass "rpi/vc4 pipeline still present (Pi 4 support kept)" || fail "rpi/vc4 pipeline missing"
    grep -q ipa_rpi_pisp <<<"$r" && pass "pisp IPA module present" || fail "pisp IPA module missing"
    grep -q '^picamera2 ' <<<"$r" && pass "python libcamera + picamera2 import" || fail "python bindings broken"
}

stage_capture() {
    echo "Stopping driver-camera (it is crash-looping; the camera is single-owner)."
    on_drone "docker update --restart=no driver-camera >/dev/null; docker stop driver-camera >/dev/null"
    on_drone "docker run --rm --privileged -v /dev:/dev -v /run/udev:/run/udev -v /tmp:/out \
        --entrypoint python3 $PATCHED -c '
import time, numpy as np
from picamera2 import Picamera2
from PIL import Image
info = Picamera2.global_camera_info(); print(\"cameras:\", info)
assert info and \"ov5647\" in str(info).lower(), \"ov5647 not enumerated\"
cam = Picamera2()
cam.configure(cam.create_video_configuration(main={\"size\": (640, 480), \"format\": \"RGB888\"}))
cam.start(); time.sleep(1.0)
ts, t0 = [], time.time()
for _ in range(150):
    req = cam.capture_request()
    ts.append(req.get_metadata().get(\"SensorTimestamp\", 0))
    arr = req.make_array(\"main\"); req.release()
dt = time.time() - t0; cam.stop()
Image.fromarray(arr[:, :, ::-1]).save(\"/out/patched_frame.jpg\")
d = np.diff(ts)
g = arr.mean(axis=2)
edges = np.abs(np.diff(g, axis=0)).mean() + np.abs(np.diff(g, axis=1)).mean()
print(f\"shape={arr.shape} mean={arr.mean():.1f} std={arr.std():.1f} edges={edges:.2f}\")
print(f\"fps={150/dt:.1f} ts_nonzero={all(t > 0 for t in ts)} ts_monotonic={bool((d > 0).all())}\")
'" 2>&1 | grep -v -E "^\[[0-9:.]+\] .*(INFO|WARN)" | tee "$OUT/capture.txt"
    scp -q -o BatchMode=yes "$DRONE:/tmp/patched_frame.jpg" "$OUT/patched_frame.jpg" 2>/dev/null
    grep -q "ov5647" "$OUT/capture.txt" && pass "ov5647 enumerated inside container" || fail "camera not enumerated"
    grep -q "shape=(480, 640, 3)" "$OUT/capture.txt" && pass "frame is 640x480 RGB" || fail "wrong/no frame"
    # A smooth vignette (lens covered / dark scene) has std > 10 but almost no edges; require both,
    # and require a lit scene (mean > 40) so the test can't pass on darkness.
    awk -F'mean=' '/mean=/{split($2,a," "); exit !(a[1]+0 > 40)}' "$OUT/capture.txt" && pass "scene is lit (mean > 40)" || fail "frame too dark: uncover lens / light the scene, then rerun baseline + capture"
    awk -F'edges=' '/edges=/{exit !($2+0 > 3)}' "$OUT/capture.txt" && pass "frame has real detail (edge energy > 3)" || fail "frame has no detail (flat or vignette only)"
    awk -F'fps=' '/fps=/{split($2,a," "); exit !(a[1]+0 >= 12)}' "$OUT/capture.txt" && pass "frame rate >= 12 fps" || fail "frame rate too low"
    grep -q "ts_nonzero=True ts_monotonic=True" "$OUT/capture.txt" && pass "sensor timestamps valid" || fail "bad timestamps"
    echo "Compare visually: results/host_reference.jpg vs results/patched_frame.jpg"
}

swap_driver_camera_image() {  # $1 = image to run driver-camera with (same config, new image)
    on_drone "set -e
        docker inspect driver-camera > /tmp/driver-camera.inspect.json
        cfg=\$(python3 - <<'PY'
import json, shlex
c = json.load(open('/tmp/driver-camera.inspect.json'))[0]
hc, cc = c['HostConfig'], c['Config']
a = ['--name driver-camera', '--restart unless-stopped']
a += ['--privileged'] if hc.get('Privileged') else []
a += ['--network ' + hc['NetworkMode']] if hc.get('NetworkMode') else []
a += ['-v ' + shlex.quote(b) for b in (hc.get('Binds') or [])]
a += ['-e ' + shlex.quote(e) for e in (cc.get('Env') or [])]
a += ['--label ' + shlex.quote(k + '=' + v) for k, v in (cc.get('Labels') or {}).items()]
a += ['--entrypoint ' + shlex.quote(' '.join(cc['Entrypoint']))] if cc.get('Entrypoint') else []
print(' '.join(a), '|||', ' '.join(shlex.quote(x) for x in (cc.get('Cmd') or [])))
PY
)
        docker rm -f driver-camera >/dev/null
        eval docker run -d \${cfg%|||*} $1 \${cfg#*|||} >/dev/null
        docker inspect -f '{{.Config.Image}} {{.State.Status}}' driver-camera"
}

stage_system() {
    swap_driver_camera_image "$PATCHED" | tee "$OUT/system.txt"
    echo "Soaking for $SOAK_MIN minutes..."
    sleep $((SOAK_MIN * 60))
    on_drone "docker inspect -f 'restarts={{.RestartCount}} health={{if .State.Health}}{{.State.Health.Status}}{{end}}' driver-camera;
        docker ps --format '{{.Names}} {{.Status}}' | grep -v healthy;
        sudo vcgencmd get_throttled; sudo vcgencmd measure_temp" | tee -a "$OUT/system.txt"
    grep -q "restarts=0 health=healthy" "$OUT/system.txt" && pass "driver-camera healthy, 0 restarts" || fail "driver-camera not stable"
    grep -q "throttled=0x0" "$OUT/system.txt" && pass "no throttling/undervoltage" || fail "throttling seen"
    echo "Also check by eye: dashboard > Mission Control > Camera shows live video."
}

stage_rollback() {
    swap_driver_camera_image "$OFFICIAL" | tee "$OUT/rollback.txt"
    grep -q "$OFFICIAL" "$OUT/rollback.txt" && pass "official image restored" || fail "rollback failed"
}

case "${1:-}" in
    baseline|image|capture|system|rollback) "stage_$1" ;;
    *) sed -n '2,12p' "$0"; exit 2 ;;
esac
echo; [ "$FAILED" = 0 ] && echo "STAGE '$1': ALL CHECKS PASSED" || { echo "STAGE '$1': SOME CHECKS FAILED"; exit 1; }
