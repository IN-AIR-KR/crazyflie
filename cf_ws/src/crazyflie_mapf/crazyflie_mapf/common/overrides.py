"""start:= / goal:= launch arguments: per-robot start and goal positions without editing YAML.

    start:=cf1:-1.2,0/cf3:0.97,-0.71        x,y (z = 0) or x,y,z
    goal:=cf1:1.2,0,1.0/cf3:-0.97,0.71      x,y,z, or x,y to keep the YAML goal's height

Robots are separated by '/' (or ';' or spaces when quoted). Robots that are not named keep their
YAML values. The overridden files are written next to each other in a temporary folder and every
node of the launch reads them, so the usual validation (bounds, separation, takeoff columns) applies.
"""

import re
import tempfile
from pathlib import Path

import yaml

from .trajectory import read_mapping

OUTPUT = Path(tempfile.gettempdir()) / 'mapf_override'


def parse_points(text, label):
    """'cf1:-1.2,0/cf3:1,2,3' -> {'cf1': [-1.2, 0.0], 'cf3': [1.0, 2.0, 3.0]}."""
    result = {}
    for item in (part for part in re.split(r'[/;\s]+', text.strip()) if part):
        name, sep, numbers = item.partition(':')
        if not sep or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError(f'{label}: "{item}"는 이름:x,y[,z] 형식이어야 합니다. 예) {label}:=cf1:-1.2,0/cf3:0.97,-0.71')
        try:
            values = [float(v) for v in numbers.split(',')]
        except ValueError:
            raise ValueError(f'{label}: "{item}"의 좌표가 숫자가 아닙니다.') from None
        if len(values) not in (2, 3):
            raise ValueError(f'{label}: "{item}"는 x,y 또는 x,y,z 좌표가 필요합니다.')
        if name in result:
            raise ValueError(f'{label}: {name}을(를) 두 번 지정했습니다.')
        result[name] = values
    return result


def apply(robots, goals, starts, targets):
    """New (robots, goals) mappings with the start/goal overrides applied."""
    robots = yaml.safe_load(yaml.safe_dump(robots))
    goals = yaml.safe_load(yaml.safe_dump(goals))
    for label, names in (('start', starts), ('goal', targets)):
        for name in names:
            robot = robots.get('robots', {}).get(name)
            if robot is None:
                raise ValueError(f'{label}: {name}은(는) robots.yaml에 없는 기체입니다.')
            if not robot.get('enabled'):
                raise ValueError(f'{label}: {name}은(는) robots.yaml에서 enabled: false입니다.')
    for name, values in starts.items():
        robots['robots'][name]['initial_position'] = values if len(values) == 3 else [*values, 0.0]
    table = goals.setdefault('goals', {})
    for name, values in targets.items():
        if len(values) == 2:
            if name not in table:
                raise ValueError(f'goal: {name}은(는) YAML 목적지가 없어 높이를 정할 수 없습니다. x,y,z로 지정하세요.')
            values = [*values, float(table[name][2])]
        table[name] = values
    return robots, goals


def experiment_paths(context, directory, mode):
    """robots/planner/goals/obstacles paths of the experiment in `directory`, with start:= and
    goal:= applied (overridden robots.yaml and goals.yaml are written to a temporary folder)."""
    directory = Path(directory)
    paths = {key: directory / filename for key, filename in (
        ('robots', 'robots.yaml'), ('planner', 'planner.yaml'), ('goals', 'goals.yaml'), ('obstacles', f'obstacles_{mode}.yaml'))}
    starts = parse_points(context.launch_configurations.get('start', ''), 'start')
    targets = parse_points(context.launch_configurations.get('goal', ''), 'goal')
    if not starts and not targets:
        return paths
    robots, goals = apply(read_mapping(paths['robots']), read_mapping(paths['goals']), starts, targets)
    folder = OUTPUT / directory.name
    folder.mkdir(parents=True, exist_ok=True)
    paths['robots'], paths['goals'] = folder / 'robots.yaml', folder / 'goals.yaml'
    paths['robots'].write_text(yaml.safe_dump(robots, sort_keys=False, allow_unicode=True))
    paths['goals'].write_text(yaml.safe_dump(goals, sort_keys=False, allow_unicode=True))
    return paths


def describe(homes, goals):
    """One log line per robot: start -> goal."""
    fmt = lambda p: '(' + ', '.join(f'{v:.2f}' for v in p) + ')'  # noqa: E731
    return [f'{name}: 시작 {fmt(homes[name])} -> 목적지 {fmt(goals[name])}' for name in homes]


def declare():
    from launch.actions import DeclareLaunchArgument
    return [
        DeclareLaunchArgument('start', default_value='', description='per-robot start, e.g. cf1:-1.2,0/cf3:0.97,-0.71 (x,y[,z]); empty uses robots.yaml'),
        DeclareLaunchArgument('goal', default_value='', description='per-robot goal, e.g. cf1:1.2,0,1.0/cf3:-0.97,0.71,1.0 (x,y,z or x,y keeping the YAML height); empty uses goals.yaml'),
    ]

