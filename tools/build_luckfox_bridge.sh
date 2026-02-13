#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/luckfox_native"

TARGET="auto"                  # auto|native|armv7
BUILD_DIR=""
BUILD_TYPE="Release"
USE_OPENCV="ON"                # ON|OFF
RUN_AFTER_BUILD=0
RUN_MODE="apriltag"
RUN_CONFIG="$ROOT_DIR/luckfox/config/default.json"
RUN_EXTRA_ARGS=()
CLEAN=0
JOBS=""
SYNC_LEGACY_PATH=1

usage() {
  cat <<'EOF'
Usage: tools/build_luckfox_bridge.sh [options] [-- <runtime args>]

Options:
  --target <auto|native|armv7>   Build target mode (default: auto)
  --build-dir <dir>              Custom build directory
  --build-type <type>            CMake build type (default: Release)
  --opencv <on|off>              Enable OpenCV lookup (default: on)
  --clean                         Remove build directory before configure
  --jobs <N>                      Parallel build jobs (default: auto)
  --legacy-sync <on|off>          Copy built binary to luckfox_native/build/luckfox_bridge (default: on)
  --run                           Run binary after successful build
  --mode <apriltag|disabled>      Runtime mode used with --run
  --config <path>                 Runtime config path used with --run
  --help                          Show this help

Cross-compile environment (for --target armv7):
  CROSS_PREFIX=<prefix>           Example: arm-buildroot-linux-uclibcgnueabihf-
  CC=<path-to-gcc>                Optional explicit C compiler
  CXX=<path-to-g++>               Optional explicit C++ compiler
  SYSROOT=<path>                  Optional CMake sysroot

Examples:
  tools/build_luckfox_bridge.sh --target native --run --mode disabled
  CROSS_PREFIX=arm-buildroot-linux-uclibcgnueabihf- tools/build_luckfox_bridge.sh --target armv7
  tools/build_luckfox_bridge.sh --target armv7 --opencv off --run -- --no-uart --debug
EOF
}

die() {
  echo "[ERROR] $*" >&2
  exit 1
}

info() {
  echo "[INFO] $*"
}

detect_host_arch() {
  uname -m 2>/dev/null || echo unknown
}

find_cross_compiler() {
  local -a candidates=(
    "arm-buildroot-linux-uclibcgnueabihf-"
    "arm-linux-gnueabihf-"
    "arm-linux-musleabihf-"
  )
  for p in "${candidates[@]}"; do
    if command -v "${p}gcc" >/dev/null 2>&1 && command -v "${p}g++" >/dev/null 2>&1; then
      echo "$p"
      return 0
    fi
  done
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target) TARGET="${2:-}"; shift 2 ;;
    --build-dir) BUILD_DIR="${2:-}"; shift 2 ;;
    --build-type) BUILD_TYPE="${2:-}"; shift 2 ;;
    --opencv)
      case "${2:-}" in
        on|ON) USE_OPENCV="ON" ;;
        off|OFF) USE_OPENCV="OFF" ;;
        *) die "--opencv must be on|off" ;;
      esac
      shift 2
      ;;
    --clean) CLEAN=1; shift ;;
    --jobs) JOBS="${2:-}"; shift 2 ;;
    --legacy-sync)
      case "${2:-}" in
        on|ON) SYNC_LEGACY_PATH=1 ;;
        off|OFF) SYNC_LEGACY_PATH=0 ;;
        *) die "--legacy-sync must be on|off" ;;
      esac
      shift 2
      ;;
    --run) RUN_AFTER_BUILD=1; shift ;;
    --mode) RUN_MODE="${2:-}"; shift 2 ;;
    --config) RUN_CONFIG="${2:-}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    --) shift; RUN_EXTRA_ARGS=("$@"); break ;;
    *) die "Unknown argument: $1 (use --help)" ;;
  esac
done

command -v cmake >/dev/null 2>&1 || die "cmake not found in PATH"

HOST_ARCH="$(detect_host_arch)"
if [[ "$TARGET" == "auto" ]]; then
  if [[ "$HOST_ARCH" == armv7l || "$HOST_ARCH" == armv6l || "$HOST_ARCH" == aarch64 ]]; then
    TARGET="native"
  else
    TARGET="armv7"
  fi
fi

if [[ "$TARGET" != "native" && "$TARGET" != "armv7" ]]; then
  die "--target must be auto|native|armv7"
fi

if [[ -z "$BUILD_DIR" ]]; then
  if [[ "$TARGET" == "armv7" ]]; then
    BUILD_DIR="$SRC_DIR/build-armv7"
  else
    BUILD_DIR="$SRC_DIR/build-native"
  fi
fi

if [[ "$CLEAN" -eq 1 ]]; then
  info "Cleaning build dir: $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"

CMAKE_ARGS=(
  -S "$SRC_DIR"
  -B "$BUILD_DIR"
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  "-DLVB_USE_OPENCV=$USE_OPENCV"
)

if [[ "$TARGET" == "armv7" ]]; then
  CROSS_PREFIX="${CROSS_PREFIX:-}"
  if [[ -z "${CC:-}" || -z "${CXX:-}" ]]; then
    if [[ -z "$CROSS_PREFIX" ]]; then
      CROSS_PREFIX="$(find_cross_compiler || true)"
    fi
  fi

  if [[ -z "${CC:-}" && -z "${CXX:-}" && -z "$CROSS_PREFIX" ]]; then
    die "No ARM cross toolchain detected. Set CROSS_PREFIX or CC/CXX explicitly."
  fi

  CROSS_CC="${CC:-${CROSS_PREFIX}gcc}"
  CROSS_CXX="${CXX:-${CROSS_PREFIX}g++}"

  command -v "$CROSS_CC" >/dev/null 2>&1 || die "C compiler not found: $CROSS_CC"
  command -v "$CROSS_CXX" >/dev/null 2>&1 || die "C++ compiler not found: $CROSS_CXX"

  DUMP_MACHINE="$("$CROSS_CXX" -dumpmachine 2>/dev/null || true)"
  if [[ "$DUMP_MACHINE" != *arm* && "$DUMP_MACHINE" != *aarch64* ]]; then
    die "Selected CXX compiler is not ARM: $CROSS_CXX (dumpmachine=$DUMP_MACHINE)"
  fi

  CMAKE_ARGS+=(
    -DCMAKE_SYSTEM_NAME=Linux
    -DCMAKE_SYSTEM_PROCESSOR=arm
    "-DCMAKE_C_COMPILER=$CROSS_CC"
    "-DCMAKE_CXX_COMPILER=$CROSS_CXX"
  )

  CROSS_PKG_CONFIG="${PKG_CONFIG_TOOL:-${CROSS_PREFIX}pkg-config}"
  if command -v "$CROSS_PKG_CONFIG" >/dev/null 2>&1; then
    CMAKE_ARGS+=("-DPKG_CONFIG_EXECUTABLE=$(command -v "$CROSS_PKG_CONFIG")")
    info "PKG_CONFIG=$CROSS_PKG_CONFIG"
  else
    info "Cross pkg-config not found (${CROSS_PKG_CONFIG}). CMake pkg-config fallback may fail."
  fi

  if [[ -z "${SYSROOT:-}" ]]; then
    SYSROOT="$("$CROSS_CXX" --print-sysroot 2>/dev/null || true)"
  fi
  if [[ -n "${SYSROOT:-}" ]]; then
    CMAKE_ARGS+=("-DCMAKE_SYSROOT=$SYSROOT")
    export PKG_CONFIG_SYSROOT_DIR="${PKG_CONFIG_SYSROOT_DIR:-$SYSROOT}"
    export PKG_CONFIG_LIBDIR="${PKG_CONFIG_LIBDIR:-$SYSROOT/usr/lib/pkgconfig:$SYSROOT/usr/share/pkgconfig:$SYSROOT/usr/lib/$DUMP_MACHINE/pkgconfig}"
    info "SYSROOT=$SYSROOT"
  fi

  info "Cross build selected (armv7)"
  info "CC=$CROSS_CC"
  info "CXX=$CROSS_CXX"
else
  info "Native build selected for host arch: $HOST_ARCH"
fi

info "Configuring CMake..."
cmake "${CMAKE_ARGS[@]}"

if [[ -z "$JOBS" ]]; then
  if command -v nproc >/dev/null 2>&1; then
    JOBS="$(nproc)"
  else
    JOBS="4"
  fi
fi

info "Building with -j $JOBS ..."
cmake --build "$BUILD_DIR" -j "$JOBS"

BIN="$BUILD_DIR/luckfox_bridge"
[[ -f "$BIN" ]] || die "Build finished but binary not found: $BIN"

if command -v file >/dev/null 2>&1; then
  info "Binary info: $(file "$BIN")"
else
  info "Binary built at: $BIN"
fi

info "Build output path: $BIN"

if [[ "$SYNC_LEGACY_PATH" -eq 1 ]]; then
  LEGACY_DIR="$SRC_DIR/build"
  LEGACY_BIN="$LEGACY_DIR/luckfox_bridge"
  mkdir -p "$LEGACY_DIR"
  cp -f "$BIN" "$LEGACY_BIN"
  chmod +x "$LEGACY_BIN" || true
  if command -v file >/dev/null 2>&1; then
    info "Legacy sync path: $LEGACY_BIN"
    info "Legacy binary info: $(file "$LEGACY_BIN")"
  else
    info "Legacy sync path: $LEGACY_BIN"
  fi
fi

if [[ "$RUN_AFTER_BUILD" -eq 1 ]]; then
  info "Running binary..."
  "$BIN" --config "$RUN_CONFIG" --mode "$RUN_MODE" "${RUN_EXTRA_ARGS[@]}"
fi
