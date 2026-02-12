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

- AprilTag-only targeting (no plant model yet)
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
- `disabled`: no detection, still runs control loop skeleton

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

## Why RKNN Is Not Used Yet

Current implementation is **AprilTag-only MVP**. AprilTag is classical marker detection and does not require NPU/RKNN.

RKNN is intended for **plant/object neural detection** (Phase 2), which requires:
1. Dataset collection + labeling
2. Model training (YOLO-n class)
3. ONNX -> RKNN conversion + quantization
4. Runtime integration and arbitration with AprilTag

## Run (manual)

Normal run:

```bash
./luckfox_native/build-armv7/luckfox_bridge --config luckfox/config/default.json --mode apriltag
```

Dry run without ESP32:

```bash
./luckfox_native/build-armv7/luckfox_bridge --config luckfox/config/default.json --mode apriltag --no-uart
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
