"""Mirror mode: real robots copy their simulated twins; guards land only the affected robot."""

import pytest

from crazyflie_mapf.common.trajectory import config_directory, load_dmpc_settings, read_mapping
from crazyflie_mapf.mirror.detect import parse
from crazyflie_mapf.mirror.functions import MirrorSupervisor

HOMES = {'cf1': (-1.2, 0.0, 0.0), 'cf3': (0.267, -1.1699, 0.0), 'cf5': (1.0812, 0.5207, 0.0)}


def settings(**overrides):
    planner = load_dmpc_settings(config_directory() / 'mirror' / 'planner.yaml')
    mirror = read_mapping(config_directory() / 'mirror' / 'mirror.yaml')
    data = {key: mirror[key] for key in ('offset', 'bounds', 'home_tolerance', 'settle_time', 'max_tracking_error',
                                         'tracking_grace', 'pose_timeout', 'command_timeout', 'sim_timeout')}
    data['settle_time'] = 0.0  # tests below that need it set it explicitly
    data.update({key: planner[key] for key in ('takeoff_height', 'takeoff_duration', 'land_height', 'land_duration')})
    data.update(overrides)
    return data


def airborne(names=('cf3', 'cf5'), **overrides):
    """Both real robots placed at their homes, simulation taken off and streaming."""
    sup = MirrorSupervisor(HOMES, list(names), settings(**overrides))
    for name in names:
        sup.pose(name, sup.homes[name], 0.0)
    assert sup.sim_phase('WAITING_POSE', 0.0) == []
    actions = sup.sim_phase('TAKEOFF', 0.1)
    assert sorted(actions) == [('takeoff', name) for name in sorted(names)]
    sup.sim_phase('AIRBORNE', 3.0)
    return sup


def stream(sup, now, positions):
    """One tick: fresh real poses equal to the given positions, simulated setpoints, guards."""
    sup.sim_phase('AIRBORNE', now)
    actions = []
    for name, position in positions.items():
        sup.pose(name, position, now)
        target, extra = sup.command(name, position, now)
        actions += extra
    return actions + sup.check(now)


def test_takeoff_only_robots_placed_at_their_simulated_start():
    sup = MirrorSupervisor(HOMES, ['cf3', 'cf5'], settings())
    sup.pose('cf3', sup.homes['cf3'], 0.0)
    sup.pose('cf5', (0.0, 0.0, 0.0), 0.0)  # placed at the wrong spot
    assert sup.sim_phase('TAKEOFF', 0.1) == [('takeoff', 'cf3')]
    assert sup.robots['cf5']['state'] == 'DROPPED' and 'from its simulated start' in sup.robots['cf5']['reason']
    # A dropped robot never joins later.
    assert sup.command('cf5', (0.0, 0.0, 1.0), 3.0) == (None, [])


def test_robot_without_pose_is_not_flown():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings())
    assert sup.sim_phase('TAKEOFF', 0.1) == []
    assert sup.robots['cf3']['reason'] == 'no fresh real pose'


def test_setpoints_are_forwarded_with_offset_only_while_streaming():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings(offset=[0.1, -0.2, 0.0]))
    sup.pose('cf3', sup.homes['cf3'], 0.0)
    sup.sim_phase('TAKEOFF', 0.1)
    assert sup.command('cf3', (0.267, -1.1699, 1.0), 1.0) == (None, [])  # high-level takeoff still running
    sup.sim_phase('AIRBORNE', 3.0)
    target, actions = sup.command('cf3', (0.267, -1.1699, 1.0), 3.0)
    assert actions == [] and target == pytest.approx((0.367, -1.3699, 1.0))


def test_simulated_landing_and_emergency_are_mirrored():
    sup = airborne()
    stream(sup, 3.1, {'cf3': (0.267, -1.1699, 1.0), 'cf5': (1.0812, 0.5207, 1.0)})
    assert sorted(sup.sim_phase('LAND_HANDOFF', 4.0)) == [('stop', 'cf3'), ('stop', 'cf5')]
    assert sorted(sup.check(4.31)) == [('land', 'cf3'), ('land', 'cf5')]
    sup = airborne()
    assert sup.sim_phase('EMERGENCY', 4.0) == [('emergency',)]
    assert sup.sim_phase('EMERGENCY', 4.1) == []


def test_tracking_error_lands_only_that_robot_after_grace():
    sup = airborne()
    good = {'cf3': (0.267, -1.1699, 1.0), 'cf5': (1.0812, 0.5207, 1.0)}
    assert stream(sup, 3.1, good) == []
    # Within the grace period a large error is tolerated (takeoff settling).
    sup.sim_phase('AIRBORNE', 3.5)
    sup.command('cf5', good['cf5'], 3.5)
    sup.command('cf3', good['cf3'], 3.5)
    sup.pose('cf3', (0.267, -1.1699, 1.5), 3.5)
    sup.pose('cf5', good['cf5'], 3.5)
    assert sup.check(3.5) == []
    sup.sim_phase('AIRBORNE', 4.2)
    sup.command('cf3', good['cf3'], 4.2)
    sup.command('cf5', good['cf5'], 4.2)
    sup.pose('cf3', (0.267, -1.1699, 1.5), 4.2)
    sup.pose('cf5', good['cf5'], 4.2)
    assert sup.check(4.2) == [('stop', 'cf3')]
    assert 'tracking error' in sup.robots['cf3']['reason'] and sup.robots['cf5']['state'] == 'FLYING'
    # cf3 is no longer fed setpoints; cf5 still is.
    assert sup.command('cf3', good['cf3'], 4.3)[0] is None
    assert sup.command('cf5', good['cf5'], 4.3)[0] is not None


@pytest.mark.parametrize('lost, reason', [('pose', 'real pose lost'), ('command', 'simulated setpoint stream stopped'),
                                          ('sim', 'simulation states lost')])
def test_lost_inputs_land_the_robot(lost, reason):
    sup = airborne(['cf3'])
    stream(sup, 3.1, {'cf3': (0.267, -1.1699, 1.0)})
    now = 3.1 + 1.5
    if lost != 'sim':
        sup.sim_phase('AIRBORNE', now)
    if lost != 'pose':
        sup.pose('cf3', (0.267, -1.1699, 1.0), now)
    if lost != 'command':
        sup.command('cf3', (0.267, -1.1699, 1.0), now)
    assert sup.check(now) == [('stop', 'cf3')]
    assert sup.robots['cf3']['reason'] == reason


def test_setpoint_outside_real_bounds_lands_the_robot():
    sup = airborne(['cf3'])
    target, actions = sup.command('cf3', (0.0, -2.5, 1.0), 3.1)
    assert target is None and actions == [('land', 'cf3')]


def test_lost_simulation_during_takeoff_lands_without_stop():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings())
    sup.pose('cf3', sup.homes['cf3'], 0.0)
    sup.sim_phase('TAKEOFF', 0.1)
    sup.pose('cf3', sup.homes['cf3'], 1.5)
    assert sup.check(1.5) == [('land', 'cf3')]


def test_mirrored_robot_must_be_simulated():
    with pytest.raises(ValueError):
        MirrorSupervisor(HOMES, ['cf9'], settings())


def test_radio_uri_parsing():
    assert parse('radio://1/80/2M/E7E7E7E7E3') == (1, 80, '2M', bytes.fromhex('E7E7E7E7E3'))
    with pytest.raises(ValueError):
        parse('udp://127.0.0.1')


def test_select_real_scans_only_robots_in_both_files():
    pytest.importorskip('launch')
    from crazyflie_mapf.common.mirror_launch import real_robots, select_real
    hardware = read_mapping(config_directory() / 'hardware' / 'robots.yaml')
    seen = {}

    def scan(uris):
        seen.update(uris)
        return {name: uri for name, uri in uris.items() if name in ('cf3', 'cf5')}
    names, notes = select_real('auto', HOMES, hardware, scan)
    assert names == ['cf3', 'cf5'] and set(seen) == set(HOMES)
    assert notes == ['cf1: 무선 응답 없음 - 시뮬레이션만 합니다.']
    assert select_real('none', HOMES, hardware, scan)[0] == []
    assert select_real('cf5', HOMES, hardware, scan)[0] == ['cf5']
    with pytest.raises(ValueError):
        select_real('cf2', HOMES, hardware, scan)  # cf2 is not simulated here
    robots = real_robots(hardware, ['cf3'], HOMES, [0.5, 0.0, 0.0])
    assert [n for n, r in robots['robots'].items() if r['enabled']] == ['cf3']
    assert robots['robots']['cf3']['initial_position'] == pytest.approx([0.767, -1.1699, 0.0])
    assert hardware['robots']['cf1']['enabled'] is True  # original left untouched


def test_simulated_executor_ignores_namespaced_real_server():
    pytest.importorskip('crazyflie_py')
    from crazyflie_mapf.executor.stack import root_services_only
    services = [('/cf1/start_trajectory', ['t']), ('/real/cf3/start_trajectory', ['t']), ('/all/takeoff', ['t'])]
    assert root_services_only(services) == [('/cf1/start_trajectory', ['t']), ('/all/takeoff', ['t'])]


def test_pose_must_stay_near_the_start_for_the_settle_time():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings(settle_time=2.0))
    sup.pose('cf3', sup.homes['cf3'], 0.0)
    sup.pose('cf3', sup.homes['cf3'], 1.0)
    assert 'steady' in sup.ready('cf3', 1.0)
    sup.pose('cf3', sup.homes['cf3'], 2.0)
    assert sup.ready('cf3', 2.0) is None


def test_diverging_estimate_that_passes_home_for_an_instant_is_refused():
    """Without a positioning sensor the estimate runs away; one sample near home must not fly."""
    sup = MirrorSupervisor(HOMES, ['cf3'], settings(settle_time=2.0))
    home = sup.homes['cf3']
    for k in range(20):
        now = k*.1
        sup.pose('cf3', (home[0], home[1]+3.0-0.3*k, 0.0), now)  # sweeps through home at k=10
    assert sup.sim_phase('TAKEOFF', 2.0) == []
    assert sup.robots['cf3']['state'] == 'DROPPED'


def test_onboard_mode_waits_for_estimator_alignment():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings(onboard=True))
    sup.pose('cf3', sup.homes['cf3'], 0.0)
    assert 'aligning' in sup.ready('cf3', 0.0)
    sup.aligned('cf3')
    sup.pose('cf3', sup.homes['cf3'], 0.1)
    assert sup.ready('cf3', 0.1) is None
    assert sup.snapshot(0.1)['mode'] == 'onboard'


def test_onboard_real_robots_disable_mocap_and_use_pid():
    pytest.importorskip('launch')
    from crazyflie_mapf.common.mirror_launch import real_robots
    hardware = read_mapping(config_directory() / 'hardware' / 'robots.yaml')
    robots = real_robots(hardware, ['cf3'], HOMES, [0, 0, 0], onboard=True)
    kind = robots['robots']['cf3']['type']
    assert kind == 'cf21_active_onboard'
    assert robots['robot_types'][kind]['motion_capture']['enabled'] is False
    assert robots['robots']['cf3']['firmware_params']['stabilizer']['controller'] == 1
    kalman = robots['robots']['cf3']['firmware_params']['kalman']
    assert (kalman['initialX'], kalman['initialY'], kalman['initialZ']) == pytest.approx(HOMES['cf3'])
    # The server applies firmware params in key order: the reset must come after the initial values.
    assert sorted(kalman)[-1] == 'resetEstimation'
    mocap = real_robots(hardware, ['cf3'], HOMES, [0, 0, 0])
    assert 'firmware_params' not in mocap['robots']['cf3']
    assert 'firmware_params' not in hardware['robots']['cf3']  # original untouched


def test_onboard_mode_keeps_only_robots_with_a_positioning_deck():
    pytest.importorskip('launch')
    from crazyflie_mapf.common.mirror_launch import check_decks
    uris = {'cf2': 'u2', 'cf5': 'u5', 'cf3': 'u3'}
    decks = {'u2': [], 'u5': ['Flow2', 'Multiranger'], 'u3': 'timeout'}
    usable, notes = check_decks(['cf2', 'cf3', 'cf5'], uris, read=lambda wanted: decks)
    assert usable == ['cf5']
    assert any('cf2' in n and 'Flow/Lighthouse/Loco' in n for n in notes)
    assert any('cf3' in n and '실패' in n for n in notes)


def test_shutdown_state_records_only_mirrored_robots(tmp_path, monkeypatch):
    from crazyflie_mapf.mirror import shutdown
    monkeypatch.setattr(shutdown, 'STATE', tmp_path / 'real.json')
    shutdown.write_state({'cf2': 'radio://0/80/2M/E7E7E7E7E2'})
    assert shutdown.read_state() == {'cf2': 'radio://0/80/2M/E7E7E7E7E2'}
    shutdown.write_state({})  # simulation-only launch clears the record
    assert not (tmp_path / 'real.json').exists() and shutdown.read_state() == {}
    # Nothing recorded: power-off is a no-op (no radio).
    assert shutdown.power_off() == 0


def test_session_ends_only_after_flying_robots_have_landed():
    """Whatever stops the simulation, the real side lands first and only then stops motors and exits."""
    sup = airborne(['cf3'])
    stream(sup, 3.1, {'cf3': (0.267, -1.1699, 1.0)})
    assert not sup.over(3.2)                      # simulation alive
    # Simulation gone (no /fleet/states): the guard lands the robot...
    sup.pose('cf3', (0.267, -1.1699, 1.0), 4.5)
    assert sup.check(4.5) == [('stop', 'cf3')]
    assert not sup.over(4.5)                      # ...still in the air
    sup.pose('cf3', (0.267, -1.1699, 1.0), 4.9)
    assert sup.check(4.9) == [('land', 'cf3')]
    land = sup.settings['land_duration']
    assert not sup.over(4.9 + land)               # descending
    assert sup.over(4.9 + land + 1.1)             # down: stop motors and exit
    assert sup.flown() == ['cf3']


def test_session_without_flight_ends_when_the_simulation_disappears():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings())
    assert not sup.over(0.0)                      # waiting for the simulation to come up
    sup.sim_phase('WAITING_POSE', 1.0)
    assert not sup.over(1.5)
    assert sup.over(1.0 + sup.settings['sim_timeout'] + 0.1)
    assert sup.flown() == []
    never = MirrorSupervisor(HOMES, ['cf3'], settings(sim_start_timeout=5.0))
    never.over(0.0)
    assert not never.over(4.0) and never.over(5.1)


def test_estop_accepts_numbers_names_lists_and_all():
    from crazyflie_mapf.mirror.estop import EMERGENCY_PACKET, resolve
    robots = {f'cf{i}': {} for i in range(1, 6)}
    assert resolve(['2'], robots) == ['cf2']
    assert resolve(['2', 'cf5', '2'], robots) == ['cf2', 'cf5']
    assert resolve(['2,5'], robots) == ['cf2', 'cf5']
    assert resolve(['all'], robots) == list(robots)
    with pytest.raises(SystemExit):
        resolve(['9'], robots)
    # Same bytes as cflib Localization.send_emergency_stop(): header 0x6D, EMERGENCY_STOP 3.
    assert EMERGENCY_PACKET == (0x6D, 3)


def test_onboard_takeoff_accepts_ground_creep_and_shifts_the_path():
    """Flow-deck estimate crept 0.23 m on the ground: fly anyway, relative to where it sits."""
    sup = MirrorSupervisor(HOMES, ['cf3'], settings(onboard=True, settle_time=2.0))
    sup.aligned('cf3')
    home = sup.homes['cf3']
    for k in range(601):  # 60 s of slow creep on the ground, 0.23 m in total (as logged)
        sup.pose('cf3', (home[0] + 0.23*k/600, home[1], 0.0), k*.1)
    assert sup.sim_phase('TAKEOFF', 60.0) == [('takeoff', 'cf3')]
    sup.sim_phase('AIRBORNE', 60.0)
    target, _ = sup.command('cf3', (home[0] + 0.5, home[1], 1.0), 60.0)
    assert target == pytest.approx((home[0] + 0.73, home[1], 1.0))


def test_onboard_diverging_estimate_never_becomes_ready():
    sup = MirrorSupervisor(HOMES, ['cf3'], settings(onboard=True, settle_time=2.0))
    sup.aligned('cf3')
    for k in range(40):   # 1 m/s runaway (no positioning deck)
        sup.pose('cf3', (k*.1, 0.0, 0.0), k*.1)
        assert sup.ready('cf3', k*.1) is not None
