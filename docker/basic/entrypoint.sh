#!/usr/bin/env bash
set -e

WS_DIR=/workspace/cf_ws
SRC_DIR="$WS_DIR/src"

fix-usb-perms

if [ -d "$SRC_DIR" ]; then
    echo "[entrypoint] Running rosdep install against $SRC_DIR..."
    sudo rosdep install --from-paths "$SRC_DIR" --ignore-src -r -y || true
fi

source /opt/ros/jazzy/setup.bash

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[entrypoint] No build found yet. Run: colcon build --symlink-install"
fi

exec "$@"
