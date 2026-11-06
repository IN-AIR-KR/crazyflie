"""Real Crazyflie flight: Crazyswarm2 hardware server + optional mocap + this package's DMPC nodes.

Differences from simulation_launch.py:
- crazyflie_server_cpp / crazyflie_server_py instead of crazyflie_sim, no Gazebo, use_sim_time=false.
- A process exit does NOT shut the whole launch down. Killing the server or executor while
  airborne drops the drones, so a crashed planner or a closed RViz must leave them hovering
  (the executor brakes to HOVER when trajectories expire) until the operator lands them.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from crazyflie_mapf.common.trajectory import load_experiment, read_mapping

SERVERS = {'cpp': 'crazyflie_server_cpp', 'cflib': 'crazyflie_server_py'}


def _flag(context, name, config):
    override = LaunchConfiguration(name).perform(context)
    if override:
        if override.lower() not in ('true', 'false'):
            raise ValueError(f'{name}은 true/false입니다.')
        return override.lower() == 'true'
    if type(config[name]) is not bool:
        raise ValueError(f'hardware.yaml의 {name}은 bool이어야 합니다.')
    return config[name]


def _mocap_params(robots, path):
    """Same rigid-body construction as crazyswarm2's crazyflie/launch/launch.py."""
    content = read_mapping(path)
    params = content['/motion_capture_tracking']['ros__parameters']
    params['rigid_bodies'] = {}
    for name, robot in robots['robots'].items():
        kind = robots['robot_types'][robot['type']]['motion_capture']
        if robot['enabled'] and kind.get('tracking') == 'librigidbodytracker':
            params['rigid_bodies'][name] = {'initial_position': robot['initial_position'],
                                            'marker': kind['marker'], 'dynamics': kind['dynamics']}
    return params


def start(context):
    directory = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    mode = LaunchConfiguration('obstacles').perform(context)
    if mode not in ('none', 'static'):
        raise ValueError('실기체 obstacles는 none 또는 static입니다. 동적 장애물은 실물 추적이 없어 지원하지 않습니다.')
    paths = {key: directory / filename for key, filename in (
        ('robots', 'robots.yaml'), ('planner', 'planner.yaml'), ('goals', 'goals.yaml'), ('obstacles', f'obstacles_{mode}.yaml'))}
    robots, settings, homes, _ = load_experiment(paths['robots'], paths['planner'], paths['goals'], paths['obstacles'])
    if mode == 'none' and settings['obstacles']:
        raise ValueError('장애물 없는 모드에서는 obstacles: []를 사용하세요.')
    if any('motion' in o for o in settings['obstacles']):
        raise ValueError('실기체 모드에서는 motion(동적 장애물)을 사용할 수 없습니다.')
    for name in homes:
        if robots['robots'][name]['type'].endswith('_sim'):
            raise ValueError(f'{name}: 실기체 모드에서는 *_sim 기체 유형을 쓸 수 없습니다.')

    config = read_mapping(directory / 'hardware.yaml')
    backend = LaunchConfiguration('backend').perform(context) or config['backend']
    if backend not in SERVERS:
        raise ValueError('backend는 cpp 또는 cflib입니다.')
    mocap, rviz, require_start = (_flag(context, key, config) for key in ('mocap', 'rviz', 'require_start'))

    server_yaml = Path(get_package_share_directory('crazyflie')) / 'config/server.yaml'
    server = yaml.safe_load(server_yaml.read_text())['/crazyflie_server']['ros__parameters']
    server['robot_description'] = (Path(get_package_share_directory('crazyflie_description'))
                                   / 'urdf/crazyflie_description.urdf').read_text()
    actions = []
    if mocap:
        params = _mocap_params(robots, (directory / config['motion_capture_file']).resolve())
        server['poses_qos_deadline'] = params['topics']['poses']['qos']['deadline']
        actions.append(Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
                            name='motion_capture_tracking', output='screen', parameters=[params]))
    actions.append(Node(package=SERVERS[backend], executable='crazyflie_server', name='crazyflie_server',
                        output='screen', parameters=[robots, server]))

    common = ['--robots', str(paths['robots']), '--config', str(paths['planner']),
              '--goals', str(paths['goals']), '--obstacles', str(paths['obstacles'])]
    for executable in ('fleet_manager', 'trajectory_planner', 'drone_executor'):
        arguments = common + (['--require-start'] if executable == 'drone_executor' and require_start else [])
        actions.append(Node(package='crazyflie_mapf', executable=executable, name=executable,
                            arguments=arguments, output='screen', parameters=[{'use_sim_time': False}]))
    if rviz:
        rviz_file = (directory / config['rviz_config']).resolve()
        if not rviz_file.is_file():
            raise ValueError(f'RViz 설정 파일이 없습니다: {rviz_file}')
        actions.append(Node(package='rviz2', executable='rviz2', name='mapf_rviz', output='screen',
                            arguments=['-d', str(rviz_file)], parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    config = str(Path(get_package_share_directory('crazyflie_mapf')) / 'config/hardware')
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=config, description='Folder with hardware robots/planner/goals/obstacles/hardware.yaml'),
        DeclareLaunchArgument('obstacles', default_value='none', description='none / static'),
        DeclareLaunchArgument('backend', default_value='', description='cpp / cflib; empty uses hardware.yaml'),
        DeclareLaunchArgument('mocap', default_value='', description='true / false; empty uses hardware.yaml'),
        DeclareLaunchArgument('rviz', default_value='', description='true / false; empty uses hardware.yaml'),
        DeclareLaunchArgument('require_start', default_value='', description='true / false; empty uses hardware.yaml'),
        OpaqueFunction(function=start),
    ])
