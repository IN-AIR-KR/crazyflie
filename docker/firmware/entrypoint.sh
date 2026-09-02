#!/usr/bin/env bash
set -e

REPO_DIR=/workspace/crazyflie-firmware

if [ -f "$REPO_DIR/pixi.toml" ] && [ ! -d "$REPO_DIR/.pixi/envs" ]; then
    echo "[entrypoint] First run: 'pixi install' against $REPO_DIR (this may take a while)..."
    (cd "$REPO_DIR" && pixi install)
fi

exec "$@"
