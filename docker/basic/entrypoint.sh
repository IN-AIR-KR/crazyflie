#!/usr/bin/env bash
set -e

WS_DIR=/workspace/cf_ws
SRC_DIR="$WS_DIR/src"
FIRMWARE_DIR=/workspace/crazyflie-firmware

fix-usb-perms

if [ -d "$SRC_DIR" ]; then
    echo "[entrypoint] Running rosdep install against $SRC_DIR..."
    sudo rosdep install --from-paths "$SRC_DIR" --ignore-src -r -y || true
fi

# crazyswarm2's crazyflie_sim (backend:=sim) imports `cffirmware`, the firmware's
# own control/estimator code exposed to Python via SWIG. It isn't on PyPI — it has
# to be built from the crazyflie-firmware source (bind-mounted here, see compose.yml).
# `crazyflie-firmware/build/` is shared with the `firmware` container, which may have
# already built it there for a *different* Python version (SWIG's .so is version-specific,
# e.g. cpython-310 vs this image's cpython-312) — check that it actually imports rather
# than just checking the file exists, and rebuild if it doesn't.
export PYTHONPATH="$FIRMWARE_DIR/build:$PYTHONPATH"
if [ -f "$FIRMWARE_DIR/Makefile" ] && ! python3 -c "import cffirmware" 2>/dev/null; then
    echo "[entrypoint] Building cffirmware Python bindings for $(python3 --version) (needed for backend:=sim)..."
    rm -f "$FIRMWARE_DIR"/build/cffirmware.py "$FIRMWARE_DIR"/build/_cffirmware*.so "$FIRMWARE_DIR"/build/cffirmware_wrap.c
    make -C "$FIRMWARE_DIR" bindings_python \
        || echo "[entrypoint] cffirmware bindings build failed — backend:=sim won't work until this is fixed (backend:=cflib/cpp are unaffected)."
fi

source /opt/ros/jazzy/setup.bash

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[entrypoint] No build found yet. Run: colcon build --symlink-install"
fi

exec "$@"
