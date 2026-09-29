"""Stage 1 SIL 서버 — Crazyswarm2 sim(4기) + RViz2 + (선택) Gazebo Sim 시각화.

    ros2 launch /workspace/stage1/sil/swarm_sim.launch.py            # Gazebo 포함
    ros2 launch /workspace/stage1/sil/swarm_sim.launch.py gazebo:=False

crazyflie_test/launch.py 와 같은 구성이지만, 기체 설정을 stage1/sil/crazyflies_swarm4.yaml
에서 읽고(서브모듈을 건드리지 않으려고), 가상 장애물 모델도 Gazebo 에 띄운다.
장애물의 움직임은 swarm_mpc_node.py 가 /world/empty/set_pose 로 매 주기 옮긴다.
Gazebo 는 시각화 전용이다 — 충돌·공기역학은 Crazyswarm2 sim 이 계산한다.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from scenario_sweep import SweepConfig, SweepScenario  # noqa: E402

WORLD_NAME = 'empty'
CF_YAML = os.path.join(HERE, 'crazyflies_swarm4.yaml')
RVIZ = os.path.join(HERE, 'swarm.rviz')
FLIGHT_Z = 0.5


def _cf_model(name, mesh):
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><static>true</static><link name="body">
<visual name="v"><geometry><mesh><uri>file://{mesh}</uri></mesh></geometry></visual>
</link></model></sdf>'''


def _obstacle_model(name, radius, height):
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><model name="{name}"><static>true</static><link name="body">
<visual name="v"><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
<material><ambient>0.9 0.3 0.2 1</ambient><diffuse>0.9 0.3 0.2 1</diffuse></material></visual>
</link></model></sdf>'''


def generate_launch_description():
    gazebo_on = LaunchConfiguration('gazebo')
    rviz_on = LaunchConfiguration('rviz')
    gz_cond = IfCondition(gazebo_on)

    with open(CF_YAML, encoding='utf-8') as f:
        robots = {n: r for n, r in yaml.safe_load(f)['robots'].items() if r.get('enabled')}
    mesh = os.path.join(get_package_share_directory('crazyflie_description'),
                        'urdf', 'cf2_assembly_with_props.dae')

    actions = [
        DeclareLaunchArgument('gazebo', default_value='True'),
        DeclareLaunchArgument('rviz', default_value='True'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('crazyflie'), 'launch', 'launch.py')),
            launch_arguments={'crazyflies_yaml_file': CF_YAML, 'backend': 'sim',
                              'mocap': 'False', 'rviz': 'False', 'teleop': 'False'}.items()),
        Node(package='rviz2', executable='rviz2', arguments=['-d', RVIZ],
             parameters=[{'use_sim_time': True}], condition=IfCondition(rviz_on)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r -v 2 empty.sdf', 'on_exit_shutdown': 'True'}.items(),
            condition=gz_cond),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='gazebo_set_pose_service_bridge',
             arguments=[f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
             condition=gz_cond),
        Node(package='crazyflie_test', executable='gazebo_pose_bridge', name='gazebo_pose_bridge',
             parameters=[{'robot_names': list(robots), 'reference_frame': 'world',
                          'world_name': WORLD_NAME}],
             condition=gz_cond),
    ]
    for name, r in robots.items():
        x, y, z = r.get('initial_position', [0.0, 0.0, 0.0])
        actions.append(Node(package='ros_gz_sim', executable='create', name=f'spawn_{name}',
                            arguments=['-world', WORLD_NAME, '-name', name,
                                       '-string', _cf_model(name, mesh),
                                       '-x', str(x), '-y', str(y), '-z', str(z)],
                            condition=gz_cond))
    # 장애물 모델은 기본 구성(mixed) 기준으로 띄운다. 처음엔 멀리 두고 노드가 옮긴다.
    sc = SweepScenario(SweepConfig())
    for k, o in enumerate(sc.obstacle_list):
        actions.append(Node(package='ros_gz_sim', executable='create', name=f'spawn_obs{k}',
                            arguments=['-world', WORLD_NAME, '-name', f'obs{k}',
                                       '-string', _obstacle_model(f'obs{k}', o.radius, 2 * FLIGHT_Z),
                                       '-x', '-50', '-y', str(-50 - 2 * k), '-z', str(FLIGHT_Z)],
                            condition=gz_cond))
    return LaunchDescription(actions)
