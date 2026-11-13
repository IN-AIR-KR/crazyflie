"""Where real Crazyflies get their position from: mocap, lighthouse or onboard.

mocap       Qualisys via motion_capture_tracking; the server forwards poses to the Crazyflie.
lighthouse  Lighthouse deck + base stations. Absolute positions in the base-station frame, measured
            on board, so no mocap node. Set up the base stations in cfclient so that frame is the
            planner's world frame. The system file (geometry/calibration) can be uploaded on launch.
onboard     Flow/Loco/Lighthouse decks with the estimate reset to the start (no absolute frame
            assumed): the path is shifted to where the robot sits (mirror mode only).
"""

import copy
from pathlib import Path

from launch.substitutions import LaunchConfiguration

from .trajectory import read_mapping

MODES = ('mocap', 'lighthouse', 'onboard')
# Deck each mode needs on every real robot (None: no deck check).
REQUIRED_DECKS = {'mocap': None, 'lighthouse': ('Lighthouse',), 'onboard': ('Flow2', 'Lighthouse', 'Loco')}
LABELS = {'mocap': 'Qualisys 모캡', 'lighthouse': 'Lighthouse', 'onboard': '모캡 없음(기체 자체 추정)'}


def _bool(text, name):
    if text.lower() not in ('true', 'false'):
        raise ValueError(f'{name}은 true/false입니다.')
    return text.lower() == 'true'


def resolve(context, config):
    """positioning:= launch argument, else the legacy mocap:= argument, else the YAML file."""
    argument = LaunchConfiguration('positioning').perform(context).strip()
    legacy = LaunchConfiguration('mocap').perform(context).strip()
    if argument and legacy:
        raise ValueError('positioning:=과 mocap:=은 함께 쓸 수 없습니다. positioning:=mocap|lighthouse|onboard를 쓰세요.')
    if not argument and legacy:
        argument = 'mocap' if _bool(legacy, 'mocap') else 'onboard'
    if not argument:
        if 'positioning' in config:
            argument = str(config['positioning'])
        elif type(config.get('mocap')) is bool:
            argument = 'mocap' if config['mocap'] else 'onboard'
        else:
            raise ValueError('설정 파일에 positioning: mocap | lighthouse | onboard가 필요합니다.')
    if argument not in MODES:
        raise ValueError(f'positioning은 {" / ".join(MODES)}입니다: {argument}')
    return argument


def load_lighthouse(path):
    """lighthouse.yaml -> dict(system_file=Path | None, firmware_params=dict)."""
    path = Path(path).resolve()
    data = read_mapping(path)
    params = data.get('firmware_params', {})
    if not isinstance(params, dict) or not all(isinstance(v, dict) for v in params.values()):
        raise ValueError(f'{path}: firmware_params는 group: {{name: value}} 형식이어야 합니다.')
    estimator = params.get('stabilizer', {}).get('estimator')
    if estimator not in (None, 2):
        raise ValueError(f'{path}: Lighthouse는 Kalman 추정기(stabilizer.estimator: 2)가 필요합니다.')
    system = str(data.get('system_file') or '').strip()
    system_file = None
    if system:
        system_file = (path.parent / system).resolve()
        if not system_file.is_file():
            raise ValueError(f'Lighthouse system_file이 없습니다: {system_file}')
    upload = data.get('upload_system', True)
    if type(upload) is not bool:
        raise ValueError(f'{path}: upload_system은 true/false입니다.')
    return dict(system_file=system_file if upload else None, firmware_params=params)


def _merge(target, params):
    for group, values in params.items():
        target.setdefault(group, {}).update(values)


def lighthouse_robots(robots, names, lighthouse):
    """Copy of a Crazyswarm2 robots mapping where `names` use Lighthouse: mocap disabled on their
    type and the Lighthouse firmware parameters set per robot (robot level wins over `all`)."""
    data = copy.deepcopy(robots)
    for name in names:
        robot = data['robots'][name]
        kind = robot['type'] if robot['type'].endswith('_lighthouse') else robot['type'] + '_lighthouse'
        if kind not in data['robot_types']:
            data['robot_types'][kind] = copy.deepcopy(data['robot_types'][robot['type']])
            data['robot_types'][kind]['motion_capture'] = {'enabled': False, 'tracking': 'vendor'}
        robot['type'] = kind
        _merge(robot.setdefault('firmware_params', {}), lighthouse['firmware_params'])
    return data


def upload_system(system_file, uris, timeout=20.0):
    """Write the base-station geometry/calibration to every robot, in a separate process so a
    hung cflib link never keeps the dongle from the server. Returns {uri: 'ok' | error}."""
    import json
    import subprocess
    import sys
    cmd = [sys.executable, '-m', 'crazyflie_mapf.mirror.lighthouse', '--upload', str(system_file),
           '--timeout', str(timeout), *uris]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout * len(uris) + 15).stdout
    except subprocess.TimeoutExpired:
        return {uri: 'Lighthouse 설정 업로드 시간 초과' for uri in uris}
    for line in out.splitlines():
        if line.startswith('UPLOAD '):
            return json.loads(line[7:])
    return {uri: 'Lighthouse 설정 업로드 실패' for uri in uris}
