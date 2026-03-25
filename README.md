# luckfox-vision-bridge

Vision-guided bridge between **Luckfox Pico Mini B** (vision node) and **ESP32** (flight controller).

## Why Native C/C++ on Luckfox

Your Buildroot image has core C/C++ runtime libraries (`libstdc++`, `libgcc_s`, `libatomic`) but very limited memory. For low-latency continuous camera + UART loops, native code is more stable than Python on this class of board.

This repo now uses:
- `luckfox_native/` as the primary runtime (C/C++)
- `luckfox/` Python folder as utility/testing scripts

## Critical Build Requirement (Luckfox Pico SDK)

Use **Luckfox Pico SDK toolchain** for deployment builds.

If you build with default Codespaces host compilers, the output becomes `x86-64` and will not run on Luckfox (`armv7l`).

Always verify build artifact architecture before copying:

```bash
file luckfox_native/build-armv7/luckfox_bridge
```

Expected: ARM ELF, not `x86-64`.

## Current Scope

- Phase 1: AprilTag targeting (native C++)
- Phase 2: native RKNN plant mode in the same runtime binary
- Continuous guidance packets Luckfox -> ESP32
- Continuous status packets ESP32 -> Luckfox
- Mission state machine: `SEARCH -> TRACK -> APPROACH -> HOLD`
- Timeout safety behavior on communication loss

## Detection Mechanism (AprilTag)

Each loop does the following:

1. Capture frame from camera (`640x480` default).
2. Convert to grayscale.
3. Detect tags from `tag36h11` dictionary.
4. Pick best tag by largest image area.
5. Estimate pose using camera intrinsics + configured tag size.
6. Compute control features:
   - `dx_norm`, `dy_norm` in range `[-1, 1]`
   - `z_est_m` from pose translation
   - `yaw_err_deg` from rotation matrix
   - `confidence` from normalized tag area
7. Build guidance packet and send over UART.
8. Read ESP32 status packet and log telemetry.

If target is stale/lost, the mission manager switches to `SEARCH` and sets `emergency_hover` in flags.

## Runtime Modes

Configured in `luckfox/config/default.json`:

```json
"runtime": {
  "engine": "native",
  "mode": "apriltag"
}
```

Supported now:
- `apriltag`: vision + guidance (requires OpenCV build with ArUco/AprilTag modules)
- `plant`: RKNN inference + guidance (requires RKNN runtime library and headers at build time)
- `disabled`: no detection, still runs control loop skeleton

Plant mode config keys:

```json
"plant": {
  "enabled": true,
  "model_path": "/root/models/model_rv1103.rknn",
  "input_size": 640,
  "min_confidence": 0.45,
  "target_class_id": -1,
  "rgb_input": true
}
```

`target_class_id=-1` means accept any class and choose highest-confidence candidate.

CLI override:

```bash
./luckfox_bridge --config luckfox/config/default.json --mode apriltag
```

## Debug and Error Reporting

Enable debug in config:

```json
"debug": {
  "enabled": true,
  "status_every_n": 10
}
```

Behavior:
- Prints periodic debug line every `status_every_n` frames.
- All hard errors print explicit reason (`errno` + message where applicable).
- CSV includes an `error` column for per-frame error state.
- In debug mode, writes an annotated video with overlays (state, confidence, `dx/dy/z/yaw`, and target marker).
- If tag is visible but `target=0` persists, lower `apriltag.min_confidence` (start with `0.25~0.35`) and keep `apriltag.confidence_full_area_ratio=0.02`.

CLI overrides:
- `--debug` force debug on
- `--quiet` force debug off
- `--no-uart` run camera/detection path without serial link

You can also disable UART from config:

```json
"uart": {
  "enabled": false
}
```

Set debug video output path in config:

```json
"logging": {
  "csv_path": "/tmp/vision_bridge_log.csv",
  "annotated_video_path": "/root/vision_bridge_annotated.mp4"
}
```

Recommended for size/quality balance:
- Use `.mp4` extension for annotated output (`mp4v/H264` path is tried first).
- Write debug video under `/root` instead of `/tmp` to avoid tmpfs/OOM kills.

Run and stop with `Ctrl+C`:

```bash
/root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug
```

On clean stop, the app prints:
- `Debug video saved: /tmp/vision_bridge_annotated.avi`

If OpenCV `VideoWriter` is not available on your Buildroot image, the app now falls back automatically to a single raw video file:
- `/tmp/vision_bridge_annotated.avi.y4m`

Then convert on host:

```bash
ffmpeg -i /tmp/vision_bridge_annotated.avi.y4m -c:v libx264 -pix_fmt yuv420p /tmp/vision_bridge_annotated.avi
```

Copy video to your laptop for inspection:

```bash
scp root@<LUCKFOX_IP>:/tmp/vision_bridge_annotated.avi .
```

## Universal Build Script

Use one script for native or cross build:

```bash
./tools/build_luckfox_bridge.sh --help
```

Auto mode (recommended):

```bash
./tools/build_luckfox_bridge.sh
```

By default, the script also copies the fresh output to:
- `luckfox_native/build/luckfox_bridge`

So old commands keep working and you do not accidentally run a stale binary.

Build and run immediately:

```bash
./tools/build_luckfox_bridge.sh --run --mode apriltag -- --no-uart
```

Force cross build for Luckfox ARMv7:

```bash
CROSS_PREFIX=arm-buildroot-linux-uclibcgnueabihf- ./tools/build_luckfox_bridge.sh --target armv7
```

If no ARM toolchain is found, the script now fails immediately instead of silently using host `gcc/g++`.

For SDK-based builds, point the script to SDK toolchain explicitly:

```bash
CC=/path/to/sdk/bin/arm-buildroot-linux-uclibcgnueabihf-gcc \
CXX=/path/to/sdk/bin/arm-buildroot-linux-uclibcgnueabihf-g++ \
./tools/build_luckfox_bridge.sh --target armv7 --clean
```

If OpenCV is in Buildroot staging (common with Luckfox SDK), pass it explicitly and require detection:

```bash
STAGING_DIR=/path/to/luckfox-pico/output/staging \
CROSS_PREFIX=arm-rockchip830-linux-uclibcgnueabihf- \
./tools/build_luckfox_bridge.sh --target armv7 --clean --require-opencv
```

Disable legacy copy if needed:

```bash
./tools/build_luckfox_bridge.sh --legacy-sync off
```

Output binary:
- Native: `luckfox_native/build-native/luckfox_bridge`
- ARMv7 cross: `luckfox_native/build-armv7/luckfox_bridge`

## Important Library Note

If OpenCV is not available in your Buildroot image, CMake still builds the binary, but `mode=apriltag` will fail fast with a clear startup error:
- `Binary built without OpenCV. AprilTag mode is unavailable.`

In that case, either:
1. Enable OpenCV (with `aruco`, `calib3d`, `videoio`) in Buildroot, or
2. Keep `mode=disabled` until the image is rebuilt.

## Troubleshooting

### `mode=apriltag` prints: `Binary built without OpenCV`

Cause:
- Runtime image / toolchain was built without OpenCV modules required by detector.

Fix:
1. Rebuild rootfs/SDK with OpenCV modules:
   - `core`, `imgproc`, `videoio`, `calib3d`, `aruco`
2. Rebuild `luckfox_bridge` with that SDK toolchain.

### Runtime loader error: `can't load library 'libopencv_*.so.406'` (or `libav*.so`)

Cause:
- Binary was linked against shared libs that are not present on the target rootfs.
- Typical missing groups: OpenCV (`libopencv_*`) and FFmpeg (`libavcodec`, `libavformat`, `libavutil`, `libswscale`, `libswresample`).

Recommended fix for this Luckfox setup:
- Keep libraries in `addons/lib` inside repo as a transfer bundle.
- Push them directly from build machine to target `/lib` on Luckfox.

On build machine:

```bash
# 1) Collect direct + transitive dependencies
mkdir -p addons/lib
arm-rockchip830-linux-uclibcgnueabihf-readelf -d luckfox_native/build-armv7/luckfox_bridge | grep NEEDED

# 2) Copy required libs from Buildroot staging/sysroot into addons/lib
# Adjust STAGING_DIR/SYSROOT paths to your SDK.
cp -av "$STAGING_DIR/usr/lib/libopencv_"*.so* addons/lib/ 2>/dev/null || true
cp -av "$STAGING_DIR/usr/lib/libavcodec.so"* addons/lib/ 2>/dev/null || true
cp -av "$STAGING_DIR/usr/lib/libavformat.so"* addons/lib/ 2>/dev/null || true
cp -av "$STAGING_DIR/usr/lib/libavutil.so"* addons/lib/ 2>/dev/null || true
cp -av "$STAGING_DIR/usr/lib/libswscale.so"* addons/lib/ 2>/dev/null || true
cp -av "$STAGING_DIR/usr/lib/libswresample.so"* addons/lib/ 2>/dev/null || true
```

Deploy to board directly from build machine:

```bash
# Copy shared libs directly to target runtime path
scp addons/lib/* root@<luckfox-ip>:/lib/

# Copy runtime binary
scp luckfox_native/build-armv7/luckfox_bridge root@<luckfox-ip>:/root/luckfox_bridge
```

Run on Luckfox:

```sh
chmod +x /root/luckfox_bridge
/root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug
```

Alternative (if you cannot write `/lib`, temporary only):

```bash
scp -r addons/lib root@<luckfox-ip>:/root/addons/
```

```sh
LD_LIBRARY_PATH=/root/addons/lib:$LD_LIBRARY_PATH /root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug
```

Notes:
- Your confirmed working path is `/lib`; use that as primary on this board.
- Keep a backup before overwriting existing system libs if versions differ.
- If the board still reports a missing `.so`, copy that exact library from the same SDK/staging used for build.
- Keep ABI consistent: target libs must come from the same toolchain/rootfs generation used to compile the binary.

### `mode=disabled` prints: `open(/dev/ttyS1) failed: No such file or directory`

Cause:
- Configured UART node does not exist on your image.

Fix:
1. Test without UART:
   - add `--no-uart`, or set `"uart.enabled": false`
2. Discover actual UART device:
   - `ls /dev/ttyS* /dev/ttyAMA* /dev/ttyUSB* 2>/dev/null`
3. Set correct `uart.port` in `luckfox/config/default.json`
4. If UART nodes are missing entirely, enable them in board DTS/Buildroot and rebuild image.

### `mode=apriltag` prints: `Failed to open camera device`

Cause:
- OpenCV video backend cannot open/probe the configured camera node.
- Common reasons: wrong `/dev/videoX`, camera busy by another process, unsupported pixel format/backend mismatch.

Fix:
1. Set explicit camera path in config:

```json
"camera": {
  "device": 0,
  "device_path": "/dev/video0"
}
```

2. If your stream works on another node, use it (for example `/dev/video11`).
3. Make sure no process is locking camera:
   - `fuser /dev/video0 /dev/video11 2>/dev/null`
4. Keep `debug.enabled=true` and check startup diagnostics; the app now logs all attempted open methods (`CAP_V4L2` and fallback).
5. With `--debug`, the app also prints OpenCV Video I/O build summary (V4L/FFmpeg/GStreamer) and a direct POSIX probe for the selected `/dev/videoX`.
6. If OpenCV backends fail to open the camera node, the app now tries a direct V4L2 mmap fallback (NV12 mplane) for AprilTag mode.

### After reboot: `VIDIOC_S_FMT ... Device or resource busy`

Cause:
- Camera service (`rkipc`) auto-starts again after reboot and locks `/dev/video11`.

How to confirm:

```sh
fuser /dev/video0 /dev/video11 2>/dev/null
ps | grep -E "rtsp|rkipc|isp|camera" | grep -v grep
```

If you see `rkipc`, camera is busy.

Temporary fix (every boot):

```sh
killall rkipc 2>/dev/null
killall rtsp 2>/dev/null
v4l2-ctl --device=/dev/video11 --stream-mmap=3 --stream-count=30
```

If `v4l2-ctl` shows stable FPS (e.g. `25.00 fps`), run the app:

```sh
/root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug
```

Permanent fix (recommended):
1. Disable `rkipc` auto-start script/service in your image startup.
2. Keep only one process owning camera during AprilTag runtime.

Check likely init scripts:

```sh
ls /etc/init.d | grep -Ei "rkipc|rtsp|camera"
```

Then disable/remove startup entry according to your init system.

### Camera Busy Recovery (Practical Checklist)

When AprilTag fails after reboot with camera busy errors, use this exact sequence:

```sh
# 1) Find who owns the camera node
fuser /dev/video0 /dev/video11 2>/dev/null

# 2) Identify process details (replace 4080 with actual PID from fuser)
cat /proc/4080/comm
cat /proc/4080/cmdline | tr '\0' ' '; echo
readlink /proc/4080/exe

# 3) Stop known camera services
killall rkipc 2>/dev/null
killall rtsp 2>/dev/null
killall ispserver 2>/dev/null
killall rkaiq_3A_server 2>/dev/null

# 4) Kill remaining owner PID directly if still busy
kill -9 4080 2>/dev/null

# 5) Confirm camera is free
fuser /dev/video0 /dev/video11 2>/dev/null

# 6) Validate streaming directly
v4l2-ctl --device=/dev/video11 --stream-mmap=3 --stream-count=30
```

If step 6 shows stable FPS, run the app:

```sh
/root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug
```

Recommended camera config:

```json
"camera": {
  "device_path": "/dev/video11"
}
```

### Prevent Auto-Restart of Camera Service

If camera lock returns every reboot, disable auto-start of camera daemon:

```sh
ls -l /etc/init.d/S91rkipc
/etc/init.d/S91rkipc stop
chmod -x /etc/init.d/S91rkipc
```

Then reboot and re-test ownership with `fuser`.

### Why Keep OpenCV Capture Path If V4L2 Fallback Works?

Short answer: keep both for now.

- On your current image, direct V4L2 fallback is the reliable path.
- OpenCV VideoCapture path is still useful for portability across other images/boards where backends are correctly enabled.
- Keeping both has no major runtime penalty (only small startup attempts before fallback).
- Removing OpenCV path now may reduce flexibility for future deployment variants.

So current recommendation:
1. Keep OpenCV path in code.
2. Use V4L2 fallback operationally on this board.
3. Revisit removal only after finalizing one fixed production image.

### End-to-end on-board checklist

1. Ensure camera free:
   - `killall rkipc 2>/dev/null`
2. Ensure stream node works:
   - `v4l2-ctl --device=/dev/video11 --stream-mmap=3 --stream-count=30`
3. Use config:
   - `camera.device_path = "/dev/video11"`
   - `debug.enabled = true`
4. Run AprilTag:
   - `/root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug`
5. Detection success signs:
   - `target=1`
   - `conf > 0`
   - `dx/dy/z` change while moving tag.
   - Annotated video shows center crosshair + moving target marker and changing telemetry text.

If camera is exposed through RTSP in your setup, set:

```json
"camera": {
  "rtsp_url": "rtsp://<ip>:<port>/<path>"
}
```

and run with `--debug`. The app prints available OpenCV VideoIO backends at startup.

### You still see old error text after rebuilding

If runtime output still shows old messages (for example exact old text like `Failed to open camera device 0`), you are likely running a stale binary on the board.

Use this flow:

```bash
# on build machine
file luckfox_native/build-armv7/luckfox_bridge
scp luckfox_native/build-armv7/luckfox_bridge root@<luckfox-ip>:/root/luckfox_bridge

# on luckfox
chmod +x /root/luckfox_bridge
/root/luckfox_bridge --config /root/luckfox/config/default.json --mode apriltag --no-uart --debug
```

The updated binary prints a `Build: <date time>` line at startup; use it to verify you are running the latest build.

## RKNN Phase 2 (Native, Same Binary)

Current runtime on board is **native C++** (`luckfox_native/build/luckfox_bridge`) using the same build flow as AprilTag.

RKNN build note (only additional difference vs AprilTag build):
- Keep using the same build command and toolchain flow.
- For `--mode plant`, ensure your SDK/staging exposes:
  - `rknn_api.h`
  - `librknnmrt.so`
- If RKNN headers/libs are missing, binary still builds, but plant mode is disabled at runtime with a clear error.

Phase 2 runs in the same native binary (`luckfox_bridge`) using `--mode plant` (or `runtime.mode=plant` in config).

Runtime behavior in plant mode:
- camera capture path is identical to AprilTag mode
- RKNN model loads from `plant.model_path`
- output decode expects YOLO-like tensor layout from your export (`[1, 10, 8400]`)
- guidance packet uses `target_type=2` for plant
- debug annotated video includes bbox + object center + mission state telemetry

## RKNN Model Folder Review

`RKNN_Model/` is valid as a conversion pipeline and contains all required phases:
1. `pt_to_onnx.py`
2. `check_onnx_io.py`
3. `generate_dataset_txt.py`
4. `onnx_or_tflite_to_rknn.py`
5. `run_pipeline.sh`

Your quantization with 80 images is usable for first deployment.  
For better stability across lighting conditions, increase calibration set to 150-300 real camera-domain images when possible.

Model review notes from your current output:
- `RKNN_Model/output/model_rv1103.rknn` exists and size looks valid.
- ONNX output shape was reported as `[1, 10, 8400]`, i.e. `4 box + 6 class` channels for YOLO-like decode.
- Native postprocess in `luckfox_bridge` is aligned to this layout.

## Run (manual)

Normal run:

```bash
./luckfox_native/build-armv7/luckfox_bridge --config luckfox/config/default.json --mode apriltag
```

Dry run without ESP32:

```bash
./luckfox_native/build-armv7/luckfox_bridge --config luckfox/config/default.json --mode apriltag --no-uart
```

RKNN plant mode:

```bash
./luckfox_native/build-armv7/luckfox_bridge --config luckfox/config/default.json --mode plant
```

RKNN plant dry run without ESP32:

```bash
./luckfox_native/build-armv7/luckfox_bridge --config luckfox/config/default.json --mode plant --no-uart --debug
```

## Validate Quickly

1. Print a `tag36h11` AprilTag.
2. Run with `--no-uart` first.
3. Move tag in front of camera.
4. Check `logging.csv_path` (default `/tmp/vision_bridge_log.csv`):
   - `target` becomes `1`
   - `dx/dy/z` change with movement
   - `error` stays `none`
5. Connect ESP32 UART and run without `--no-uart`.

## ESP32 Side

```bash
cd esp32
idf.py build
idf.py flash monitor
```

## Optional Python Utilities

- Camera calibration: `luckfox/scripts/calibrate_camera.py`
- Protocol tests: `luckfox/tests/test_protocol.py`

Run tests:

```bash
python -m pytest -q luckfox/tests
```
