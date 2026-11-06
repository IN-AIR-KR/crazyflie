"""Emergency-stop real Crazyflies by number: motors off at once (a flying robot falls).

    python3 -m crazyflie_mapf.mirror.estop 2        # cf2
    python3 -m crazyflie_mapf.mirror.estop 2 5      # cf2 and cf5 (also: cf2,cf5)
    python3 -m crazyflie_mapf.mirror.estop all      # every robot in hardware/robots.yaml

1. If a real server serves the robot (mirror: /real/cfN, hardware.launch: /cfN), its emergency
   service is called.
2. Otherwise the emergency-stop packet is sent straight from the Crazyradio (no connection needed).
After an emergency stop the robot ignores commands until it is power-cycled.
"""

import argparse
import os
import subprocess
import sys

from ..common.trajectory import config_directory, read_mapping
from .detect import parse

# CRTP localization port (6), generic channel (1), EMERGENCY_STOP (3) - as cflib's Localization.
EMERGENCY_PACKET = (((6 & 0x0F) << 4) | (3 << 2) | (1 & 0x03), 3)


def resolve(args, robots):
    names = []
    for arg in args:
        for item in arg.replace(",", " ").split():
            if item == "all":
                names += list(robots)
            elif item.isdigit():
                names.append(f"cf{item}")
            else:
                names.append(item)
    unknown = [name for name in names if name not in robots]
    if unknown:
        raise SystemExit(f"hardware/robots.yaml에 없는 기체: {', '.join(unknown)}")
    return list(dict.fromkeys(names))


def running(pattern):
    return subprocess.run(["pgrep", "-f", pattern], capture_output=True).returncode == 0


def via_server(names):
    """Call <ns>/<name>/emergency where a real server serves the robot. Returns the names stopped."""
    import rclpy
    from std_srvs.srv import Empty
    rclpy.init()
    node = rclpy.create_node("mapf_estop")
    # The simulation's server also offers /cfN/emergency; use it only when hardware.launch runs.
    prefixes = ["/real"]
    if running("^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch crazyflie_mapf hardware\\.launch\\.py"):
        prefixes.append("")
    done = []
    for name in names:
        for prefix in prefixes:
            client = node.create_client(Empty, f"{prefix}/{name}/emergency")
            # A fresh node needs a moment of DDS discovery before it sees the server's services.
            if not client.wait_for_service(timeout_sec=3.0):
                continue
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.done():
                print(f"{name}: 비상 정지 (서버 {prefix}/{name}/emergency)", flush=True)
                done.append(name)
                break
    node.destroy_node()
    rclpy.shutdown()
    return done


def via_radio(name, uri, attempts=30):
    from cflib.drivers.crazyradio import Crazyradio
    dongle, channel, rate, address = parse(uri)
    rates = {"250K": Crazyradio.DR_250KPS, "1M": Crazyradio.DR_1MPS, "2M": Crazyradio.DR_2MPS}
    try:
        radio = Crazyradio(devid=dongle)
    except Exception as exc:
        print(f"{name}: 동글을 열 수 없음 ({exc}). 서버가 동글을 쓰는 중이면 그 서버가 이 기체를 다루지 않는 것입니다.", flush=True)
        return False
    try:
        radio.set_arc(3)
        radio.set_channel(channel)
        radio.set_data_rate(rates[rate])
        radio.set_address(address)
        acks = sum(1 for _ in range(attempts) if (lambda a: a is not None and a.ack)(radio.send_packet(EMERGENCY_PACKET)))
    finally:
        radio.close()
    if acks:
        print(f"{name}: 비상 정지 (무선 직접 전송, 응답 {acks}/{attempts})", flush=True)
        return True
    print(f"{name}: 응답 없음 - 이미 꺼졌거나 범위 밖입니다. 날고 있으면 전원 버튼으로 끄세요.", flush=True)
    return False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("robots", nargs="+", help="기체 번호(2), 이름(cf2), 쉼표 목록(2,5) 또는 all")
    parser.add_argument("--config", default=str(config_directory() / "hardware" / "robots.yaml"))
    opt = parser.parse_args(args)
    robots = read_mapping(opt.config)["robots"]
    names = resolve(opt.robots, robots)
    stopped = via_server(names)
    failed = [name for name in names if name not in stopped and not via_radio(name, robots[name]["uri"])]
    if stopped or len(failed) < len(names):
        print("비상 정지 후에는 기체 전원을 껐다 켜야 다시 쓸 수 있습니다.", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
