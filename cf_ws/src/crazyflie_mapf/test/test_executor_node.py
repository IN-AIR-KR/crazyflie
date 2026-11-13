"""Hardware safety gates of DroneExecutorNode with a fake Crazyswarm stack (no server, no radio)."""

import json

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('geometry_msgs.msg')

from crazyflie_mapf.common.trajectory import config_directory, load_dmpc_settings  # noqa: E402
from crazyflie_mapf.executor.main import DroneExecutorNode  # noqa: E402


class Message:
    def __init__(self, data):
        self.data = data


class Clock:
    """Integer nanoseconds, like rclpy, so measurement stamps equal the clock exactly."""

    def __init__(self):
        self.ns = 100 * 10**9

    @property
    def t(self):
        return self.ns / 1e9

    def now(self):
        clock = self

        class Time:
            nanoseconds = clock.ns
        return Time()


class Logger:
    def info(self, *_):
        pass

    warning = error = info


class Node:
    """Minimal stand-in for crazyflie_py.CrazyflieServer."""

    def __init__(self):
        self.clock = Clock()
        self.calls = []

    def create_publisher(self, *_args, **_kwargs):
        class Publisher:
            def publish(self, _message):
                pass
        return Publisher()

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def create_timer(self, *_args, **_kwargs):
        return object()

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return Logger()

    def emergency(self):
        self.calls.append('emergency')


class Handle:
    def __init__(self):
        self.full_states = 0
        self.last_full_state = None
        self.arms = []
        self.notified = 0

    def cmdFullState(self, *args):
        self.full_states += 1
        self.last_full_state = args

    def notifySetpointsStop(self):
        self.notified += 1

    def arm(self, arm=True):
        self.arms.append(arm)


class Stack:
    def __init__(self, names):
        self.allcfs = Node()
        self.handles = {name: Handle() for name in names}

    def names(self):
        return list(self.handles)

    def get(self, name):
        return self.handles[name]

    def takeoff(self, **_kwargs):
        self.allcfs.calls.append('takeoff')

    def land(self, **_kwargs):
        self.allcfs.calls.append('land')


def make(require_start):
    settings = load_dmpc_settings(config_directory() / 'hardware' / 'planner.yaml')
    homes = {'cf1': (0.0, 0.0, 0.0)}
    stack = Stack(homes)
    node = DroneExecutorNode(stack, homes, settings, require_start=require_start)
    return stack, node


def settle(stack, node, height=0.0, seconds=0.5):
    """Feed a stationary pose stream and tick the executor."""
    clock = stack.allcfs.clock
    for _ in range(int(seconds * 50)):
        clock.ns += 20 * 10**6
        node._record('cf1', (0.0, 0.0, height), clock.t)
        node.tick()


def control(node, op):
    node._control(Message(json.dumps({'op': op})))


def test_hardware_waits_for_start_before_takeoff():
    stack, node = make(require_start=True)
    settle(stack, node)
    assert node.phase == 'WAITING_POSE' and 'takeoff' not in stack.allcfs.calls
    control(node, 'start')
    settle(stack, node)
    assert node.phase == 'TAKEOFF' and stack.allcfs.calls == ['takeoff']


def test_simulation_default_takes_off_without_start():
    stack, node = make(require_start=False)
    settle(stack, node)
    assert node.phase == 'TAKEOFF'


def test_land_before_start_never_takes_off():
    stack, node = make(require_start=True)
    control(node, 'land')
    control(node, 'start')
    settle(stack, node)
    assert node.phase == 'LANDED' and 'takeoff' not in stack.allcfs.calls


def test_emergency_stops_streaming_and_ignores_later_commands():
    stack, node = make(require_start=False)
    settle(stack, node)
    settle(stack, node, height=node.settings['takeoff_height'], seconds=4.0)
    assert node.phase == 'AIRBORNE'
    control(node, 'emergency')
    sent = stack.handles['cf1'].full_states
    control(node, 'land')
    settle(stack, node, height=node.settings['takeoff_height'])
    assert node.phase == 'EMERGENCY'
    assert stack.allcfs.calls == ['takeoff', 'emergency']
    assert stack.handles['cf1'].full_states == sent


def test_failed_takeoff_lands_automatically():
    stack, node = make(require_start=False)
    settle(stack, node)
    # The drone never reaches takeoff height before the timeout.
    short = node.settings['takeoff_height'] - 2 * node.settings['arrival_radius']
    settle(stack, node, height=short, seconds=node.settings['takeoff_timeout'] + 1.0)
    assert node.phase == 'LANDING' and stack.allcfs.calls == ['takeoff', 'land']


def _airborne_then_land(require_start=False):
    stack, node = make(require_start=require_start)
    settle(stack, node)
    settle(stack, node, height=node.settings['takeoff_height'], seconds=4.0)
    assert node.phase == 'AIRBORNE'
    control(node, 'land')
    settle(stack, node, height=node.settings['takeoff_height'], seconds=0.1)
    return stack, node


def test_hardware_lands_with_streamed_setpoints_and_disarms():
    """No streamed -> high-level handoff (it crashed on hardware): descend with cmdFullState, then disarm."""
    stack, node = _airborne_then_land()
    handle = stack.handles['cf1']
    assert node.settings['land_low_level'] is True
    assert node.phase == 'DESCENDING' and handle.notified == 0
    s = node.settings
    descent = (s['takeoff_height'] - s['land_height']) / s['land_speed']
    settle(stack, node, height=s['takeoff_height'], seconds=descent / 2)
    position, velocity = handle.last_full_state[0], handle.last_full_state[1]
    assert s['land_height'] < position[2] < s['takeoff_height'] and velocity[2] == -s['land_speed']
    settle(stack, node, height=s['land_height'], seconds=descent / 2 + s['land_settle'] + 0.2)
    assert node.phase == 'LANDED'
    assert handle.last_full_state[0][2] == s['land_height']
    assert handle.arms == [True, False]          # armed before takeoff, disarmed on the floor
    assert stack.allcfs.calls == ['takeoff'] and handle.notified == 0


def test_low_level_landing_falls_back_when_the_floor_is_not_reached():
    stack, node = _airborne_then_land()
    s = node.settings
    descent = (s['takeoff_height'] - s['land_height']) / s['land_speed']
    settle(stack, node, height=s['takeoff_height'], seconds=descent + s['land_settle'] + s['land_duration'] + 0.5)
    assert node.phase == 'LANDING' and stack.allcfs.calls == ['takeoff', 'land']
    assert stack.handles['cf1'].arms == [True]    # never disarmed in the air
