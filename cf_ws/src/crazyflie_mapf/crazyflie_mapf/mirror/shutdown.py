"""Power off mirrored real Crazyflies once the real side has landed them and released the radio.

    python3 -m crazyflie_mapf.mirror.shutdown power-off   # run by docker/mapf.sh

mirror.launch.py records the real robots it serves in STATE. Without that file this does nothing,
so a simulation-only run is never affected. Landing is done by the mirror node itself: when the
simulation goes away it lands every flying robot, stops their motors and exits.
"""

import json
import os
import sys
import threading
from pathlib import Path

STATE = Path("/tmp/mapf_real.json")
# Everything the separate real-side launch (mirror_real.launch.py) needs, written by mirror.launch.py.
PLAN = Path("/tmp/mapf_real_plan.json")
AIRBORNE = ("FLYING", "STOPPING")


def write_state(robots):
    """robots: {name: uri}; an empty mapping removes the record."""
    if robots:
        STATE.write_text(json.dumps(robots))
    elif STATE.exists():
        STATE.unlink()


def read_state():
    try:
        return json.loads(STATE.read_text())
    except (OSError, ValueError):
        return {}


def power_off(timeout=5.0, tries=3):
    """Turn the recorded real robots off over the radio (same as their power button).

    The power-down packet can be acknowledged without the robot switching off, so each robot is
    pinged afterwards and the command is repeated until it stops answering.
    """
    robots = read_state()
    if not robots:
        return 0
    import time
    from cflib.utils.power_switch import PowerSwitch
    from .scan import ping

    def send(uri):
        result = {}

        def run():
            try:
                switch = PowerSwitch(uri)
                switch.platform_power_down()
                switch.close()
                result["ok"] = True
            except Exception as exc:
                result["error"] = exc
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        thread.join(timeout)
        return result.get("error")

    for name, uri in robots.items():
        error, answers = None, None
        for _ in range(tries):
            error = send(uri)
            time.sleep(1.5)
            try:
                answers = ping([uri], 20)[uri]
            except Exception as exc:  # radio busy: cannot confirm
                error, answers = exc, None
                break
            if answers <= 2:
                break
        if answers is not None and answers <= 2:
            print(f"{name}: 전원 꺼짐 확인 ({uri})", flush=True)
        else:
            reason = error or f"끄기 명령 후에도 응답 {answers}/20"
            print(f"{name}: 전원이 꺼지지 않았습니다 ({reason}). 기체 전원 버튼으로 끄세요.", flush=True)
    write_state({})
    return 0


def main(args=None):
    args = sys.argv[1:] if args is None else args
    if args[:1] == ["power-off"]:
        return power_off()
    print(__doc__)
    return 2


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)  # cflib/rclpy helper threads must not keep the shell waiting
