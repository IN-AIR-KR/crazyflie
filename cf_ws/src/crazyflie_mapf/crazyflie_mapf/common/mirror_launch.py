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
from crazyflie_mapf.common.overrides import declare as declare_overrides, experiment_paths
from crazyflie_mapf.common.positioning import (LABELS as POSITIONING_LABELS, MODES, REQUIRED_DECKS, lighthouse_robots,
                                               load_lighthouse, resolve as resolve_positioning, upload_system)
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


def real_robots(hardware, names, homes, offset, mode="mocap", lighthouse=None):
    """Hardware robots file with only the mirrored robots enabled, starting at their twins' homes.

    onboard (no mocap): the robots use a copy of their type with motion capture disabled, the
    PID controller (steadier with flow-deck estimates, as in crazyflie_test), and an onboard
    estimate initialised at their simulated start.
    lighthouse: mocap disabled and lighthouse.yaml's firmware parameters (absolute positions,
    so the estimate is not initialised: the start check sees where the robot really is).
    """
    if mode not in MODES:
        raise ValueError(f"positioning은 {' / '.join(MODES)}입니다.")
    onboard = mode == "onboard"
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
    if mode == "lighthouse":
        data = lighthouse_robots(data, names, lighthouse)
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


def check_decks(names, uris, read=read_decks, wanted=REQUIRED_DECKS["onboard"]):
    """Onboard and Lighthouse modes need a positioning deck. Returns (usable names, notes)."""
    decks = read([uris[name] for name in names])
    usable, notes = [], []
    for name in names:
        result = decks.get(uris[name], "응답 없음")
        if isinstance(result, str):
            notes.append(f"{name}: 덱 확인 실패({result}) - 시뮬레이션만 합니다.")
            continue
        found = [deck for deck in result if deck in wanted]
        if found:
            usable.append(name)
            notes.append(f"{name}: {', '.join(found)} 덱으로 위치 측정")
        else:
            notes.append(f"{name}: {'/'.join(wanted)} 덱이 없어 이 위치 측정 방식으로 띄울 수 없음 - 시뮬레이션만 합니다.")
    return usable, notes


def upload_lighthouse(names, uris, system_file, upload=upload_system):
    """Write the base-station setup to each robot. Robots that fail are not flown. Returns (names, notes)."""
    if not names or system_file is None:
        return names, [] if system_file else ["Lighthouse system_file 없음: 기체에 저장된 기지국 설정을 씁니다."]
    results = upload(system_file, [uris[name] for name in names])
    usable, notes = [], []
    for name in names:
        result = results.get(uris[name], "응답 없음")
        if result == "ok":
            usable.append(name)
            notes.append(f"{name}: Lighthouse 기지국 설정 업로드 완료 ({system_file.name})")
        else:
            notes.append(f"{name}: Lighthouse 기지국 설정 업로드 실패({result}) - 시뮬레이션만 합니다.")
    return usable, notes


def start(context):
    from crazyflie_mapf.mirror.detect import reachable

    directory = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    mode = LaunchConfiguration('obstacles').perform(context)
    if mode not in ('none', 'static', 'dynamic'):
        raise ValueError('obstacles는 none / static / dynamic입니다.')
    paths = experiment_paths(context, directory, mode)
    paths['mirror'] = directory / 'mirror.yaml'
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
    positioning = resolve_positioning(context, config)
    use_mocap = positioning == 'mocap'
    lighthouse = load_lighthouse(directory / config.get('lighthouse_file', '../hardware/lighthouse.yaml')) if positioning == 'lighthouse' else None
    uris = {n: hardware['robots'][n]['uri'] for n in names}
    if names and not use_mocap:
        names, deck_notes = check_decks(names, uris, wanted=REQUIRED_DECKS[positioning])
        notes += deck_notes
    if names and lighthouse is not None:
        names, upload_notes = upload_lighthouse(names, uris, lighthouse['system_file'])
        notes += upload_notes
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
    position_source = POSITIONING_LABELS[positioning]
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
    robots = real_robots(hardware, names, homes, offset, mode=positioning, lighthouse=lighthouse)
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
                                  '--namespace', namespace, '--positioning', positioning])
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
        DeclareLaunchArgument('positioning', default_value='', description='mocap (Qualisys) / lighthouse (Lighthouse deck + base stations) / onboard (estimate reset to the start, Flow/Lighthouse/Loco deck); empty uses mirror.yaml'),
        DeclareLaunchArgument('mocap', default_value='', description='legacy: true = positioning:=mocap, false = positioning:=onboard'),
        *declare_overrides(),
        # Read by simulation_launch.start
        DeclareLaunchArgument('operation', default_value='run', description='mirror mode always runs SIL'),
        DeclareLaunchArgument('gazebo', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('gazebo_gui', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('rviz', default_value='', description='true / false; empty uses simulation.yaml'),
        DeclareLaunchArgument('software_rendering', default_value='', description='true / false (GPU); empty uses simulation.yaml'),
        OpaqueFunction(function=start),
    ])
