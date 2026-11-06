"""Real side of mirror mode: real Crazyswarm2 server (+ mocap) and the mirror node.

Started by mirror.launch.py in its own session, from the plan it writes to /tmp/mapf_real_plan.json.
It outlives the simulation: when the simulation stops, the mirror node lands the robots, stops their
motors and exits; this launch then shuts the real server down.
"""

import json

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from crazyflie_mapf.mirror.shutdown import PLAN


def generate_launch_description():
    plan = json.loads(PLAN.read_text())
    ns = plan['namespace']
    # tf2 broadcasts on the absolute /tf; keep real frames (also named cf3...) out of the simulation's TF.
    remap = [('/tf', f'/{ns}/tf'), ('/tf_static', f'/{ns}/tf_static')]
    actions = [LogInfo(msg=f"[mirror_real] 실기체 서버 시작: {', '.join(n for n, r in plan['robots']['robots'].items() if r['enabled'])}")]
    if plan['mocap'] is not None:
        actions.append(Node(package='motion_capture_tracking', executable='motion_capture_tracking_node',
                            name='motion_capture_tracking', namespace=ns, output='screen',
                            parameters=[plan['mocap']], remappings=remap))
    actions.append(Node(package=plan['server_package'], executable='crazyflie_server', name='crazyflie_server',
                        namespace=ns, output='screen', parameters=[plan['robots'], plan['server']], remappings=remap))
    mirror = Node(package='crazyflie_mapf', executable='mirror_node', name='mirror', output='screen',
                  arguments=plan['mirror_arguments'], parameters=[{'use_sim_time': False}])
    actions.append(mirror)
    actions.append(RegisterEventHandler(OnProcessExit(target_action=mirror, on_exit=[
        LogInfo(msg='[mirror_real] mirror 종료: 실기체 서버를 닫습니다.'),
        EmitEvent(event=Shutdown(reason='mirror finished'))])))
    return LaunchDescription(actions)
