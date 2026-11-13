"""Lighthouse positioning and the start:= / goal:= launch overrides."""

import pytest

from crazyflie_mapf.common.trajectory import config_directory, load_experiment, read_mapping

pytest.importorskip('launch')

from crazyflie_mapf.common.mirror_launch import check_decks, real_robots, upload_lighthouse  # noqa: E402
from crazyflie_mapf.common.overrides import apply, experiment_paths, parse_points  # noqa: E402
from crazyflie_mapf.common.positioning import lighthouse_robots, load_lighthouse  # noqa: E402

HOMES = {'cf1': (-1.2, 0.0, 0.0), 'cf3': (0.9708, -0.7053, 0.0)}
LIGHTHOUSE = config_directory() / 'hardware' / 'lighthouse.yaml'


class Context:
    def __init__(self, **values):
        self.launch_configurations = values


def test_lighthouse_robots_disable_mocap_and_keep_the_real_start():
    hardware = read_mapping(config_directory() / 'hardware' / 'robots.yaml')
    lighthouse = load_lighthouse(LIGHTHOUSE)
    robots = real_robots(hardware, ['cf3'], HOMES, [0, 0, 0], mode='lighthouse', lighthouse=lighthouse)
    robot = robots['robots']['cf3']
    assert robot['enabled'] and robot['type'] == 'cf21_active_lighthouse'
    assert robots['robot_types'][robot['type']]['motion_capture']['enabled'] is False
    assert robot['firmware_params']['stabilizer']['estimator'] == 2
    assert robot['firmware_params']['lighthouse']['method'] == 1
    # Absolute positions: the estimate is not reset to the start, so a misplaced robot is refused.
    assert 'kalman' not in robot['firmware_params']
    assert robot['initial_position'] == list(HOMES['cf3'])
    assert robots['robot_types']['cf21_active']['motion_capture']['enabled'] is True  # mocap type untouched
    assert 'cf21_active_lighthouse' not in hardware['robot_types']  # original untouched


def test_lighthouse_conversion_is_idempotent():
    hardware = read_mapping(config_directory() / 'hardware' / 'robots.yaml')
    lighthouse = load_lighthouse(LIGHTHOUSE)
    once = lighthouse_robots(hardware, ['cf1'], lighthouse)
    assert lighthouse_robots(once, ['cf1'], lighthouse)['robots']['cf1']['type'] == 'cf21_active_lighthouse'


def test_lighthouse_requires_kalman_and_an_existing_system_file(tmp_path):
    (tmp_path / 'lh.yaml').write_text('firmware_params:\n  stabilizer: {estimator: 1}\n')
    with pytest.raises(ValueError, match='Kalman'):
        load_lighthouse(tmp_path / 'lh.yaml')
    (tmp_path / 'lh.yaml').write_text('system_file: missing.yaml\n')
    with pytest.raises(ValueError, match='system_file'):
        load_lighthouse(tmp_path / 'lh.yaml')
    (tmp_path / 'system.yaml').write_text('type: lighthouse_system_configuration\n')
    (tmp_path / 'lh.yaml').write_text('system_file: system.yaml\n')
    assert load_lighthouse(tmp_path / 'lh.yaml')['system_file'] == tmp_path / 'system.yaml'
    (tmp_path / 'lh.yaml').write_text('system_file: system.yaml\nupload_system: false\n')
    assert load_lighthouse(tmp_path / 'lh.yaml')['system_file'] is None


def test_lighthouse_mode_needs_the_lighthouse_deck():
    uris = {'cf1': 'u1', 'cf3': 'u3'}
    decks = {'u1': ['Lighthouse'], 'u3': ['Flow2']}
    usable, notes = check_decks(['cf1', 'cf3'], uris, read=lambda _: decks, wanted=('Lighthouse',))
    assert usable == ['cf1']
    assert any('cf3' in n and 'Lighthouse 덱이 없어' in n for n in notes)


def test_failed_system_upload_drops_only_that_robot(tmp_path):
    uris = {'cf1': 'u1', 'cf3': 'u3'}
    system = tmp_path / 'system.yaml'
    usable, notes = upload_lighthouse(['cf1', 'cf3'], uris, system, upload=lambda f, u: {'u1': 'ok', 'u3': 'timeout'})
    assert usable == ['cf1'] and any('cf3' in n and 'timeout' in n for n in notes)
    assert upload_lighthouse(['cf1'], uris, None, upload=None)[0] == ['cf1']


def test_parse_points():
    assert parse_points('cf1:-1.2,0/cf3:1,2,3', 'start') == {'cf1': [-1.2, 0.0], 'cf3': [1.0, 2.0, 3.0]}
    assert parse_points('cf1:0,0 cf2:1,1;cf3:2,2', 'start').keys() == {'cf1', 'cf2', 'cf3'}
    assert parse_points('', 'goal') == {}
    for bad in ('cf1', 'cf1:1', 'cf1:a,b', 'cf1:1,2/cf1:3,4', '1cf:0,0'):
        with pytest.raises(ValueError):
            parse_points(bad, 'start')


def test_apply_overrides_only_named_robots():
    robots = read_mapping(config_directory() / 'mirror' / 'robots.yaml')
    goals = read_mapping(config_directory() / 'mirror' / 'goals.yaml')
    new_robots, new_goals = apply(robots, goals, {'cf1': [-1.0, 0.2]}, {'cf1': [1.0, -0.2], 'cf2': [0.3, 1.0, 1.5]})
    assert new_robots['robots']['cf1']['initial_position'] == [-1.0, 0.2, 0.0]
    assert new_robots['robots']['cf2']['initial_position'] == robots['robots']['cf2']['initial_position']
    assert new_goals['goals']['cf1'] == [1.0, -0.2, goals['goals']['cf1'][2]]  # x,y keeps the YAML height
    assert new_goals['goals']['cf2'] == [0.3, 1.0, 1.5]
    assert robots['robots']['cf1']['initial_position'] == [-1.2, 0.0, 0.0]  # original untouched
    with pytest.raises(ValueError, match='없는 기체'):
        apply(robots, goals, {'cf9': [0, 0]}, {})


def test_apply_refuses_disabled_robots():
    robots = read_mapping(config_directory() / 'hardware' / 'robots.yaml')
    goals = read_mapping(config_directory() / 'hardware' / 'goals.yaml')
    disabled = next(name for name, r in robots['robots'].items() if not r['enabled'])
    with pytest.raises(ValueError, match='enabled: false'):
        apply(robots, goals, {disabled: [0, 0]}, {})


def test_overridden_experiment_is_validated(tmp_path, monkeypatch):
    from crazyflie_mapf.common import overrides
    monkeypatch.setattr(overrides, 'OUTPUT', tmp_path)
    directory = config_directory() / 'mirror'
    paths = experiment_paths(Context(start='cf1:-1.0,0.1', goal='cf1:1.0,-0.1,1.2'), directory, 'none')
    assert paths['robots'].parent == tmp_path / 'mirror' and paths['planner'] == directory / 'planner.yaml'
    _, _, homes, goals = load_experiment(paths['robots'], paths['planner'], paths['goals'], paths['obstacles'])
    assert homes['cf1'] == pytest.approx((-1.0, 0.1, 0.0)) and goals['cf1'] == pytest.approx((1.0, -0.1, 1.2))
    # Without arguments the YAML files are used directly.
    assert experiment_paths(Context(), directory, 'none')['robots'] == directory / 'robots.yaml'
    # A goal outside the flight space is refused before anything starts.
    paths = experiment_paths(Context(goal='cf1:5,0,1'), directory, 'none')
    with pytest.raises(ValueError):
        load_experiment(paths['robots'], paths['planner'], paths['goals'], paths['obstacles'])
