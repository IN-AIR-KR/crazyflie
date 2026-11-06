"""Simulation of every robot + real Crazyflies that copy their simulated twins.

All robots fly in SIL exactly as in the simulation launches. Each robot that also exists in the
hardware robots file and answers on the radio (real:=auto) or is named (real:=cf3,cf5) is served
by a second, real Crazyswarm2 server under /<namespace>, and the `mirror` node forwards its twin's
takeoff, full-state setpoints, land and emergency to it. The DMPC never sees real states.
"""

import copy
import json
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from crazyflie_mapf.common import simulation_launch
from crazyflie_mapf.common.hardware_launch import SERVERS, _mocap_params
from crazyflie_mapf.common.trajectory import load_experiment, read_mapping


def select_real(argument, simulated, hardware, scan):
    """Robots to mirror: names in both files, filtered by the radio scan. Returns (names, notes)."""
    candidates = {name: robot["uri"] for name, robot in hardware["robots"].items() if name in simulated}
    if argument in ("", "none"):
        return [], ["real:=none - 시뮬레이션만 실행합니다."]
    if argument == "auto":
        requested = sorted(candidates)
    else:
        requested = [name.strip() for name in argument.split(",") if name.strip()]
        unknown = [name for name in requested if name not in candidates]
        if unknown:
            raise ValueError(f"real에 지정한 {unknown}은(는) 시뮬레이션 robots와 hardware robots 양쪽에 있어야 합니다.")
    notes = []
    found = scan({name: candidates[name] for name in requested}) if requested else {}
    for name in requested:
        if name not in found:
            notes.append(f"{name}: 무선 응답 없음 - 시뮬레이션만 합니다.")
    return sorted(found), notes


POSITIONING_DECKS = ("Flow2", "Lighthouse", "Loco")


def real_robots(hardware, names, homes, offset, onboard=False):
    """Hardware robots file with only the mirrored robots enabled, starting at their twins' homes.

    onboard (no mocap): the robots use a copy of their type with motion capture disabled, the
    PID controller (steadier with flow-deck estimates, as in crazyflie_test), and an onboard
    estimate initialised at their simulated start.
    """
    data = copy.deepcopy(hardware)
    for name, robot in data["robots"].items():
        robot["enabled"] = name in names
        if name in names:
            if robot["type"].endswith("_sim"):
                raise ValueError(f"{name}: hardware robots의 기체 유형이 *_sim입니다. 실기체 유형을 쓰세요.")
            robot["initial_position"] = [h + o for h, o in zip(homes[name], offset)]
            if onboard:
                kind = robot["type"] + "_onboard"
                if kind not in data["robot_types"]:
                    data["robot_types"][kind] = copy.deepcopy(data["robot_types"][robot["type"]])
                    data["robot_types"][kind]["motion_capture"] = {"enabled": False, "tracking": "vendor"}
                robot["type"] = kind
                params = robot.setdefault("firmware_params", {})
                params.setdefault("stabilizer", {})["controller"] = 1
                # Start the onboard estimate at the simulated start. The server applies these on
                # connect in key order (initialX, initialY, initialYaw, initialZ, resetEstimation),
                # so the reset uses the new initial values. A namespaced server ignores later ROS
                # parameter changes (crazyflie_server_cpp only handles events of /crazyflie_server).
                home = robot["initial_position"]
                params["kalman"] = {"initialX": float(home[0]), "initialY": float(home[1]),
                                    "initialZ": float(home[2]), "initialYaw": 0.0, "resetEstimation": 1}
    return data


def read_decks(uris, timeout=10.0):
    """Deck names per URI, read in a separate process so a hung cflib link can never keep the
    dongle away from the real server. Returns {uri: [decks] | error string}."""
    import json
    import subprocess
    import sys
    cmd = [sys.executable, "-m", "crazyflie_mapf.mirror.scan", "--decks", *uris, "--info-timeout", str(timeout)]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout * len(uris) + 15).stdout
    except subprocess.TimeoutExpired:
        return {uri: "덱 확인 시간 초과" for uri in uris}
    for line in out.splitlines():
        if line.startswith("DECKS "):
            return json.loads(line[6:])
    return {uri: "덱 확인 실패" for uri in uris}


def check_decks(names, uris, read=read_decks):
    """Onboard mode needs a positioning deck. Returns (usable names, notes)."""
    decks = read([uris[name] for name in names])
    usable, notes = [], []
    for name in names:
        result = decks.get(uris[name], "응답 없음")
        if isinstance(result, str):
            notes.append(f"{name}: 덱 확인 실패({result}) - 시뮬레이션만 합니다.")
            continue
        found = [deck for deck in result if deck in POSITIONING_DECKS]
        if found:
            usable.append(name)
            notes.append(f"{name}: 모캡 없이 {', '.join(found)} 덱으로 위치 추정")
        else:
            notes.append(f"{name}: 위치 추정 덱(Flow/Lighthouse/Loco)이 없어 모캡 없이는 띄울 수 없음 - 시뮬레이션만 합니다.")
    return usable, notes


def start(context):
    from crazyflie_mapf.mirror.detect import reachable

    directory = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    mode = LaunchConfiguration('obstacles').perform(context)
    if mode not in ('none', 'static', 'dynamic'):
        raise ValueError('obstacles는 none / static / dynamic입니다.')
    paths = {key: directory / filename for key, filename in (
        ('robots', 'robots.yaml'), ('planner', 'planner.yaml'), ('goals', 'goals.yaml'),
        ('obstacles', f'obstacles_{mode}.yaml'), ('mirror', 'mirror.yaml'))}
    _, settings, homes, _ = load_experiment(paths['robots'], paths['planner'], paths['goals'], paths['obstacles'])
    if LaunchConfiguration('operation').perform(context) != 'run':
        raise ValueError('mirror 모드는 operation:=run만 지원합니다.')
    config = read_mapping(paths['mirror'])
    hardware = read_mapping((directory / config['hardware_robots']).resolve())
    if not isinstance(hardware.get('robots'), dict) or not isinstance(hardware.get('robot_types'), dict):
        raise ValueError('hardware robots 파일에 robots와 robot_types가 필요합니다.')
    argument = LaunchConfiguration('real').perform(context).strip()
    logs = []
    names, notes = select_real(argument, homes, hardware, lambda uris: reachable(uris, log=logs.append))
    mocap_arg = LaunchConfiguration('mocap').perform(context) or str(config['mocap'])
    if mocap_arg.lower() not in ('true', 'false'):
        raise ValueError('mocap은 true/false입니다.')
    use_mocap = mocap_arg.lower() == 'true'
    if names and not use_mocap:
        names, deck_notes = check_decks(names, {n: hardware['robots'][n]['uri'] for n in names})
        notes += deck_notes
    offset = [float(v) for v in config['offset']]
    low, high = config['bounds']
    for name in names:
        home = [h + o for h, o in zip(homes[name], offset)]
        if not all(lo <= p <= hi for p, lo, hi in zip(home, low, high)):
            raise ValueError(f'{name}: 시작 위치 {home}가 mirror.yaml bounds 밖입니다.')
    for low_s, high_s, o, low_r, high_r in zip(*settings['bounds'], offset, low, high):
        if low_s + o < low_r - 1e-9 or high_s + o > high_r + 1e-9:
            raise ValueError('planner.yaml bounds(+offset)가 mirror.yaml의 실기체 bounds보다 넓습니다.')

    actions = [LogInfo(msg=f'[mirror] {line}') for line in logs + notes]
    position_source = 'Qualisys 모캡' if use_mocap else '모캡 없음(기체 자체 추정)'
    actions.append(LogInfo(msg=f"[mirror] 실기체: {', '.join(names) if names else '없음'} / 위치: {position_source} / 시뮬레이션: {', '.join(homes)}"))
    # cascade: any simulation process exiting (a closed Gazebo/RViz window, Ctrl+C, a crash) ends
    # the whole simulation. The real side below is a separate process that outlives it and lands.
    actions += simulation_launch.start(context, mode, cascade=True, require_start=bool(names))
    from crazyflie_mapf.mirror.shutdown import PLAN, write_state
    # docker/mapf.sh powers off exactly these robots after the real side has landed them.
    write_state({name: hardware['robots'][name]['uri'] for name in names})
    if not names:
        if PLAN.exists():
            PLAN.unlink()
        return actions

    namespace = str(config.get('namespace', 'real')).strip('/')
    backend = LaunchConfiguration('backend').perform(context) or config['backend']
    if backend not in SERVERS:
        raise ValueError('backend는 cpp 또는 cflib입니다.')
    robots = real_robots(hardware, names, homes, offset, onboard=not use_mocap)
    server_yaml = Path(get_package_share_directory('crazyflie')) / 'config/server.yaml'
    server = yaml.safe_load(server_yaml.read_text())['/crazyflie_server']['ros__parameters']
    server['robot_description'] = (Path(get_package_share_directory('crazyflie_description'))
                                   / 'urdf/crazyflie_description.urdf').read_text()
    mocap = None
    if use_mocap:
        mocap = _mocap_params(robots, (directory / config['motion_capture_file']).resolve())
        server['poses_qos_deadline'] = mocap['topics']['poses']['qos']['deadline']
    plan = dict(namespace=namespace, server_package=SERVERS[backend], robots=robots, server=server, mocap=mocap,
                mirror_arguments=['--mirror', str(paths['mirror']), '--config', str(paths['planner']),
                                  '--robots', str(paths['robots']), '--real', ','.join(names),
                                  '--namespace', namespace] + ([] if use_mocap else ['--onboard']))
    PLAN.write_text(json.dumps(plan))
    # setsid --fork: the real launch gets its own session, so stopping the simulation launch
    # (and its process group) never signals the real server while a robot is in the air.
    actions.append(ExecuteProcess(cmd=['setsid', '--fork', 'ros2', 'launch', 'crazyflie_mapf', 'mirror_real.launch.py'],
                                  output='screen'))
    return actions


def generate_launch_description():
    config = str(Path(get_package_share_directory('crazyflie_mapf')) / 'config/mirror')
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=config, description='Folder with mirror.yaml and the simulation configuration'),
        DeclareLaunchArgument('obstacles', default_value='none', description='none / static / dynamic (virtual obstacles)'),
        DeclareLaunchArgument('real', default_value='auto', description='auto (radio scan) / none / cf3,cf5'),
        DeclareLaunchArgument('backend', default_value='', description='cpp / cflib; empty uses mirror.yaml'),
        DeclareLaunchArgument('mocap', default_value='', description='true (Qualisys) / false (onboard estimate, needs Flow/Lighthouse/Loco deck); empty uses mirror.yaml'),
        # Read by simulation_launch.start
        DeclareLaunchArgument('operation', default_value='run', description='mirror mode always runs SIL'),
        DeclareLaunchArgument('gazebo', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('gazebo_gui', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('rviz', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('software_rendering', default_value='', description='true / false (GPU); empty uses simulation.yaml'),
        OpaqueFunction(function=start),
    ])
