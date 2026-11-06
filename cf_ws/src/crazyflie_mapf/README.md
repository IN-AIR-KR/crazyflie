# crazyflie_mapf

이 폴더에서 기체, 목적지, 계획기, 시뮬레이터를 설정하고 실행하는 독립 ROS 2 패키지다.
7대 DMPC 교차 이동을 **장애물 없음 / 정적 장애물 / 동적 장애물** 모드로 실행한다.
PBS 핵심 `priority.py`와 `priority_demo`도 유지하지만, DMPC 비행에는 사용하지 않는다.

## 폴더 구성

```text
crazyflie_mapf/
├── config/
│   ├── robots.yaml           # 사용 기체, 연결 주소, 초기 위치
│   ├── goals.yaml            # 기체별 목적지
│   ├── obstacles_none.yaml   # 장애물 없음
│   ├── obstacles_static.yaml # 고정 박스의 위치·크기
│   ├── obstacles_dynamic.yaml # 움직이는 박스의 왕복 경로·주기
│   ├── planner.yaml          # 속도, 가속도, 안전거리, 예측 시간
│   ├── simulation.yaml       # SIL 제어기, 화면, 파일 경로
│   ├── visualization.rviz    # RViz 표시와 카메라
│   ├── world.sdf             # 바닥·조명 (장애물은 YAML에서 생성)
│   ├── hardware/             # 실기체 전용 설정 (robots, planner, goals, 장애물, mocap, hardware.yaml)
│   └── mirror/               # 시뮬레이션 + 실기체 따라 하기 설정 (mirror.yaml)
├── launch/
│   ├── no_obstacles.launch.py
│   ├── static_obstacles.launch.py
│   ├── dynamic_obstacles.launch.py
│   ├── simulation.launch.py  # 기존 명령 호환: 정적 모드
│   ├── hardware.launch.py    # 실기체 비행 (Crazyradio + Qualisys)
│   └── mirror.launch.py      # 전체 시뮬레이션 + 연결된 실기체가 같은 동작 수행
├── crazyflie_mapf/
│   ├── priority.py           # PBS 우선순위 탐색
│   ├── demo.py               # PBS 합성 조건 예제
│   ├── trajectory/           # DMPC QP, 검증, 지표, 시각화
│   ├── executor/             # 비행 실행기와 Gazebo pose 연동
│   ├── fleet/                # 목적지 배정과 완료 상태
│   └── common/               # simulation_launch.py: 공통 실행, 설정·ROS 도구
└── test/
```

실행 시 `crazyflie_test`와 `crazyflie_delivery`의 코드·설정·launch를 사용하지 않는다.
공통 기반인 ROS 2 Jazzy, Crazyswarm2의 `crazyflie_sim`, `crazyflie_py`,
`crazyflie_description`, firmware Python 바인딩(`cffirmware`), Gazebo Sim은 필요하다.
Python 의존성은 NumPy, SciPy, PyYAML, OSQP 1.x이며 기존 basic 이미지에 설치되어 있다.
원본 코드와 이 패키지의 후속 변경은 자동 동기화되지 않는다.

## 설정

- **robots.yaml**: `enabled`, `uri`, `initial_position: [x, y, z]`, `type`을 기체별로 지정한다.
  좌표는 `all.reference_frame` 기준이며 단위는 m다. Crazyradio는 IP 대신
  `radio://동글번호/채널/전송속도/기체주소`를 사용한다. 기본 주소는 예시다.
  이전 `crazyflies.yaml`은 `robots.yaml`로 이름을 변경했다.
- **goals.yaml**: `frame: world`와 `goals: {cf1: [x, y, z], ...}`로 목적지를 지정한다.
  초기 위치에서 자동 계산하지 않는다. 비활성 기체의 목적지는 무시한다.
- **obstacles_none/static/dynamic.yaml**: 각 launch가 해당 모드의 파일만 읽는다.
  기존 `obstacles.yaml`은 `obstacles_static.yaml`로 이름을 변경했다.
- **planner.yaml**: `speed`, `acceleration`, `deceleration`, `horizontal_separation`,
  `vertical_separation`, `horizon`, `plan_rate`, `takeoff_height`, `bounds` 등을 지정한다.
  현재 속도 상한은 2.5m/s, 가속도는 0.65m/s², 감속도는 0.9m/s²다.
  실제 속도는 경로 길이와 회피 제약에 따라 상한보다 낮을 수 있다.
- **simulation.yaml**: `operation`, `gazebo`, `gazebo_gui`, `rviz`, `render_engine`,
  `software_rendering`, `sim.controller` 등을 지정한다. 상대 파일 경로는 config 폴더 기준이다.
  `gazebo_ip: 127.0.0.1`은 같은 PC의 Gazebo 프로세스 간 통신 주소다.

기체 추가 시 robots와 goals에 같은 이름을 추가한다. 2대/4대로 제한한 코드는 없지만
현재 기본 설정은 7대이며 계산 비용은 기체 수에 따라 증가한다.
목적지 누락, 좌표계 불일치, 경계 밖/겹치는 목적지, 겹치는 이륙 위치는 시작 전에 거부한다.
`sim.visualizations.rviz.enabled`는 TF 위치 피드백 때문에 true로 유지한다.
`firmware_bindings`는 기존 Docker의 `/workspace/crazyflie-firmware/build`를 가리킨다.
네이티브 실행 시 바꾸거나, 이미 Python import가 가능하면 빈 문자열로 둔다.
위 세 launch는 항상 SIL 서버를 사용하며 무선 기체에 연결하지 않는다. 실기체는 아래 [실기체 비행](#실기체-비행)을 참고한다.

## 장애물 설정

`config/obstacles_static.yaml` 또는 `config/obstacles_dynamic.yaml`을 수정하고 launch를 재시작한다. 위치·크기의 단위는 m이며,
`position`은 **바닥 위치가 아니라 박스 중심**이다. 회전 없는 박스만 지원한다.

```yaml
frame: world
obstacles:
  - name: center_box
    enabled: true
    type: box
    position: [0.0, 0.0, 0.8]
    size: [0.5, 0.5, 1.6]
```

이 예제는 지면 위 높이 1.6m 박스다. 장애물을 더 만들려면 서로 다른 `name`으로
항목을 추가한다. `enabled: false`는 해당 박스를 끄며, `obstacles: []`는 모두 없앤다.
Gazebo의 박스와 RViz의 주황색 박스, DMPC 회피 계산은 같은 YAML을 읽는다.
RViz의 옅은 타원체는 기본 안전 영역이다.

계획기는 박스에 `robot_radius + collision_margin`을 더한 뒤 그 박스 전체를 감싸는
타원체를 사용한다. 각 반축은 `sqrt(3) * (size/2 + robot_radius + collision_margin)`이다.
추적·시간 불확실성도 계산 중 추가하므로 실제 박스보다 넓게 우회하며 좁은 통로는
사용하지 못할 수 있다. 연속 가속도 궤적 전체와 감속 fallback을 검사한다.
초기 이륙 경로와 목적지가 안전 영역에 들어가면 시작 전에 거부하고,
실행 중 요청하는 목적지도 검사한다. 잘못된 frame, 크기, 중복 이름은 오류로 처리한다.

이것은 정적/예측 가능한 동적 장애물에 대한 국소 DMPC 확장이다. 복잡한 미로에서 경로를 반드시
찾는 전역 계획기는 아니며, 해를 찾지 못하면 감속하거나 UNSAFE를 보고한다.
Gazebo 박스에는 collision 요소도 있지만 드론 동역학은 별도 SIL에서 계산되므로
Gazebo 접촉 물리가 비행을 막아주는 구조는 아니다.

## 세 가지 실행 모드

공통 기체·목적지·계획 설정은 그대로 공유하고 장애물 파일만 분리한다.
한 번에 하나의 launch만 실행한다. 이전 시뮬레이션은 먼저 종료한다.

| 실행 파일 | 사용하는 장애물 설정 |
| --- | --- |
| `no_obstacles.launch.py` | `config/obstacles_none.yaml` (`obstacles: []`) |
| `static_obstacles.launch.py` | `config/obstacles_static.yaml` |
| `dynamic_obstacles.launch.py` | `config/obstacles_dynamic.yaml` |

각 launch는 `crazyflie_mapf/common/simulation_launch.py`의 공통 실행 함수를 호출한다.
`simulation.launch.py`는 기존 사용자를 위해 정적 모드로 유지한다.
`config_dir`, `operation`, `gazebo`, `gazebo_gui`, `rviz` 인자는 세 모드 모두 동일하다.
장애물 없는 모드에 활성 박스를 넣거나 정적 모드에 `motion`을 넣으면 오류로 처리한다.

```bash
ros2 launch crazyflie_mapf no_obstacles.launch.py
ros2 launch crazyflie_mapf static_obstacles.launch.py
ros2 launch crazyflie_mapf dynamic_obstacles.launch.py
```

동적 모드의 기본 예제는 다음과 같다.

```yaml
frame: world
obstacles:
  - name: moving_box
    enabled: true
    type: box
    position: [0.0, -0.7, 0.8]
    size: [0.5, 0.5, 1.6]
    motion:
      type: ping_pong
      end_position: [0.0, 0.7, 0.8]
      period: 7.0
      start_time: 4.0
```

`position`에서 `end_position`까지 1.4m를 3.5초에 이동하고, 돌아오는 데 3.5초로 총 7초 주기다.
이동 속도는 0.4m/s다.
시뮬레이션 시작 후 4초까지 정지한 뒤 움직인다. 양 끝에서 즉시 방향을 바꾸는
등속 왕복 모델이며 회전은 없다. 동적 YAML 안에 `motion` 없는 정적 박스를 섞을 수 있다.

Gazebo pose, RViz, 계획기는 동일한 ROS `/clock` 시간과 이동 함수를 사용한다.
계획기는 예측 구간의 미래 장애물 위치를 사용하고, 방향 전환 시점에서 구간을 나눠
상대 운동의 연속 충돌을 검사한다. 감속 경로에도 같은 검사를 적용한다.
목적지에서 계속 호버하고 수직 이륙할 수 있도록, 목적지와 이륙 경로는 장애물의
**전체 왕복 범위를 감싼 보수적인 안전 영역** 밖에 있어야 한다.

이 모드는 미리 알려진 이동 경로를 예측하는 시뮬레이션이다. 센서로 감지한 임의의
이동 장애물을 추적하는 기능은 아니다. `operation:=preview`는 `/clock` 없는 ROS
시스템 시간을 사용하므로 왕복 위상이 SIL 실행과 다르며 대기 시간은 이미 지난 것으로 본다.

## 빌드·실행

### 스크립트로 실행 (`docker/mapf.sh`)

호스트 `~/crazyflie`에서 실행한다. NVIDIA GPU가 있으면 자동으로 GPU 구성(`compose.gpu.yml`)과 GPU 렌더링을 쓴다.

```bash
docker/mapf.sh build                  # 처음 한 번: 이미지(OSQP 포함) + 패키지 빌드
docker/mapf.sh sim                    # 시뮬레이션 5대 (3.6 m). 도착하면 자동 착륙
docker/mapf.sh mirror                 # 시뮬레이션 + 무선 응답하는 실기체가 따라 함
docker/mapf.sh mirror real:=cf3,cf5   # 지정한 실기체만
docker/mapf.sh mirror mocap:=false    # 모캡 없이 기체 자체 위치 추정으로 따라 함
docker/mapf.sh scan                   # 지금 무선으로 잡히는 기체·배터리·덱 (mirror가 꺼져 있을 때)
docker/mapf.sh estop 2                # cf2 강제 정지(모터 즉시 끔). estop 2 5 / estop all
docker/mapf.sh hardware               # 실기체만
docker/mapf.sh legacy static          # 기존 7대 시뮬레이션 (none / static / dynamic)

docker/mapf.sh start | land | emergency | status | stop   # 다른 터미널에서 조작
docker/mapf.sh help
```

뒤에 붙인 인자는 ros2 launch로 전달된다(예: `docker/mapf.sh sim gazebo:=false`). Ctrl+C로 끄면 Gazebo·RViz도 정리된다.
GPU 없이 실행하려면 `MAPF_GPU=0 docker/mapf.sh sim`.

### 직접 실행

호스트 `~/crazyflie`에서:

```bash
docker compose -f docker/compose.yml up -d basic
xhost +local:docker
docker compose -f docker/compose.yml exec basic bash
```

컨테이너 안에서:

```bash
cd /workspace/cf_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select crazyflie_mapf
source install/setup.bash
ros2 launch crazyflie_mapf simulation.launch.py
```

YAML은 symlink 설치되므로 값을 수정한 뒤 launch를 재시작하면 적용된다.
새 파일이나 실행 파일 구성을 추가하면 다시 빌드한다. 다른 ROS 시뮬레이션은 먼저
종료한다. `/fleet/*`, `/tf`, `/clock` 토픽을 사용한다. Ctrl+C, RViz 닫기 또는 Gazebo
종료 시 함께 시작한 프로세스를 종료한다. 모든 목적지에 도착하면 `auto_land_delay`(기본 2초) 호버한 뒤 전체가 착륙한다(`planner.yaml`의 `auto_land: false`면 호버 유지).

```bash
# GUI 없는 SIL 검사
ros2 launch crazyflie_mapf simulation.launch.py gazebo:=false rviz:=false

# 비행 서버 없이 이상적 궤적만 표시 (Gazebo는 실행하지 않음)
ros2 launch crazyflie_mapf simulation.launch.py operation:=preview

# 패키지 소스의 설정 폴더를 명시
ros2 launch crazyflie_mapf simulation.launch.py config_dir:=/workspace/cf_ws/src/crazyflie_mapf/config

# PBS 예제 및 테스트
ros2 run crazyflie_mapf priority_demo
python3 -m pytest src/crazyflie_mapf/test -q
```


### GPU로 화면 띄우기

Gazebo·RViz를 Mesa 소프트웨어 렌더링으로 띄우면 CPU를 많이 써서(Gazebo GUI만 코어 8개 이상) 계획 계산이
1~2초로 늦어지고, 계획이 유효 시간(0.4초)을 넘겨 BRAKING이 반복될 수 있다. NVIDIA GPU가 있으면 GPU로 띄운다.

```bash
# 호스트 ~/crazyflie (nvidia-container-toolkit 필요)
docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d basic
xhost +local:docker
docker compose -f docker/compose.yml -f docker/compose.gpu.yml exec basic bash
# 컨테이너 안
ros2 launch crazyflie_mapf mirror.launch.py software_rendering:=false
```

`software_rendering:=false`는 모든 시뮬레이션 launch에서 쓸 수 있다. 비우면 `simulation.yaml` 값(기본 true)을 따른다.

## 실기체 비행

`hardware.launch.py`는 Crazyswarm2 실기체 서버(`crazyflie_server_cpp` 또는 `cflib`)와
Qualisys `motion_capture_tracking`, 이 패키지의 계획기·실행기를 실행한다. Gazebo와 sim time은 쓰지 않는다.
설정은 `config/hardware/`만 읽으며 시뮬레이션 설정과 분리되어 있다.

| 항목 | 시뮬레이션 | 실기체 (`config/hardware/planner.yaml`) |
| --- | --- | --- |
| 속도 상한 | 2.5 m/s | **0.5 m/s** |
| 가속도 / 감속도 | 0.65 / 0.9 m/s² | **0.3 / 0.4 m/s²** |
| 명령 전송 주기 | 100 Hz | **50 Hz** (무선 대역폭) |
| 비행 공간 | 9 × 9 m | **3.6 × 3.6 m**, 높이 0.2~2.0 m |
| 배치 | 반경 2.5 m 원 7대, 높이 1.2 m | **반경 1.2 m 원 5대**, 높이 1.0 m |
| 상태 끊김 허용 | 3 s | **1 s** |

SIL에서 5대 교차가 모두 도착했다(BRAKING 없음, 임무 시간 약 9초).
같은 3.6 m 공간에 7대를 두면(이웃 간격 1.04 m) SIL 추종 오차 때문에 서로 양보하지 못하고 감속·호버로 멈췄다.
`collision_margin`을 0.09 이상으로 키우거나 공간을 바꾸면 일부 대수에서
서로 양보하지 못하고 감속 상태로 멈출 수 있다(국소 DMPC의 교착). 설정을 바꾸면 먼저 SIL로 확인한다.

### 비행 전 준비

1. `config/hardware/robots.yaml`: 실제 `uri`, 실제 놓은 `initial_position`(Qualisys world 좌표).
   기체 이름(cf1...)은 QTM rigid body 이름과 같아야 한다. **기본값은 cf1 한 대만 켜져 있다.**
2. `config/hardware/motion_capture.yaml`: `hostname`을 QTM PC의 IP로 바꾼다.
3. `config/hardware/planner.yaml`의 `bounds`를 실제 비행 공간(그물/벽 안쪽)에 맞춘다.
4. `config/hardware/goals.yaml`: 켠 기체의 목적지. 경계 안, 서로 충분히 떨어져 있어야 한다.
5. 배터리 충전, 프로펠러·마커 상태, Crazyradio 연결을 확인한다. 한 동글당 3~4대를 권장한다.

설정이 잘못되면(목적지 누락, 경계 밖, 겹치는 위치, `*_sim` 기체 유형, 동적 장애물) launch 시작 전에 거부한다.

### 실행

```bash
# 컨테이너 안 (빌드는 위와 같음)
ros2 launch crazyflie_mapf hardware.launch.py                 # 장애물 없음, cpp 서버, Qualisys
ros2 launch crazyflie_mapf hardware.launch.py obstacles:=static
ros2 launch crazyflie_mapf hardware.launch.py backend:=cflib rviz:=false
ros2 launch crazyflie_mapf hardware.launch.py mocap:=false    # Lighthouse 등 온보드 위치 추정
```

`require_start: true`(기본)이면 launch 후 바로 이륙하지 않는다. RViz와 로그로 위치가 맞는지 확인한 뒤
다른 터미널에서 명령을 보낸다.

```bash
# 이륙 및 비행 시작
ros2 topic pub --once /fleet/control std_msgs/msg/String '{data: "{\"op\": \"start\"}"}'
# 착륙 (감속·호버 후 착륙)
ros2 topic pub --once /fleet/control std_msgs/msg/String '{data: "{\"op\": \"land\"}"}'
# 비상 정지: 모든 모터 즉시 정지. 이후 기체 전원을 껐다 켜야 한다.
ros2 topic pub --once /fleet/control std_msgs/msg/String '{data: "{\"op\": \"emergency\"}"}'
```

### 안전 동작과 주의

- 모든 목적지에 도착하면 2초 호버 후 자동 착륙한다(`auto_land`). 도중에 내리려면 `land`를 보낸다.
- 이륙 시간 초과(`takeoff_timeout`) 시 자동으로 착륙한다.
- 위치가 `state_timeout`/`wall_timeout` 이상 끊기거나 계획이 만료되면 전체 기체가 감속 후 호버한다.
- 계획기나 RViz가 종료돼도 나머지 프로세스는 계속 동작한다(실행기가 호버 유지). 이때 `land`로 내린다.
- **비행 중 Ctrl+C는 서버와 실행기를 종료시켜 기체가 떨어진다.** 반드시 착륙 후 종료한다.
- 동적 장애물 모드는 실물 추적이 없어 지원하지 않는다. 정적 장애물은 실물이 없으면 비행 금지 구역처럼 동작한다.
- 1대 → 2대 → 3대 → 5대 순서로 늘리고, 각 단계에서 `/trajectory/status`의 `BRAKING`·`over_budget`,
  서버의 통신 경고(지연, 패킷 손실)를 확인한다.

## 시뮬레이션 따라 하기 (mirror)

`mirror.launch.py`는 모든 기체를 시뮬레이션(SIL)으로 비행시키고, **무선이 연결된 실기체만**
같은 이름의 시뮬레이션 기체와 똑같이 움직이게 한다. 예를 들어 5대 시뮬레이션 중 cf3, cf5만
응답하면 실제 cf3, cf5가 시뮬레이션 cf3, cf5의 이륙·비행·착륙을 그대로 따라 한다.

```text
fleet_manager → trajectory_planner → drone_executor ──/cfN/cmd_full_state──→ SIL 서버 (5대)
                                           │                    │
                                     /fleet/states        (cf3, cf5만)
                                           ▼                    ▼
                                         mirror 노드 ──/real/cfN/cmd_full_state──→ 실기체 서버 (/real)
                                           ▲                                          │
                                           └────────── /real/cfN/pose (Qualisys) ─────┘
```

- **시뮬레이션이 주인이다.** 계획기는 시뮬레이션 상태만 보고, 실기체 상태는 계획에 쓰이지 않는다.
- 실기체 서버는 `/real` 이름공간에서, 시뮬레이션과 별도 프로세스(`mirror_real.launch.py`)로 실행한다.
  실기체 TF는 `/real/tf`로 분리해 시뮬레이션과 섞이지 않는다.
- 이륙·착륙·비상정지는 시뮬레이션 실행기의 단계(`/fleet/states`)를 보고 똑같이 보낸다.
- 시뮬레이션 계획은 실기체가 들어갈 공간을 써야 하므로 `config/mirror/`는 실기체용 속도(0.5 m/s)와
  배치(`hardware/`와 같음)를 쓴다. 실기체 uri·유형·mocap은 `config/hardware/`를 그대로 읽는다.

### 실행

```bash
ros2 launch crazyflie_mapf mirror.launch.py                # real:=auto: 무선 응답하는 기체를 자동으로 찾음
ros2 launch crazyflie_mapf mirror.launch.py real:=cf3,cf5  # 지정한 기체만 (응답 없으면 시뮬레이션만)
ros2 launch crazyflie_mapf mirror.launch.py real:=none     # 시뮬레이션만
```

`real:=auto`는 launch 시작 시 `hardware/robots.yaml`의 각 uri에 Crazyradio로 신호를 보내 응답하는 기체를
고른다(`enabled` 값은 무시). 실행 중에 새로 켜진 기체는 합류하지 않는다. 실기체가 하나라도 있으면
시뮬레이션도 `start` 명령을 기다린다.
`/mirror/status`와 로그에서 각 실기체의 `ready` 여부를 확인한 뒤 위의 `start` 명령을 보낸다.

### 종료하면 실기체는 착륙 → 모터 정지 → 전원 끔

실기체 쪽(실기체 서버, 모캡, mirror 노드)은 `mirror_real.launch.py`로 **시뮬레이션과 별도 프로세스**에서 돈다.
그래서 시뮬레이션이 어떻게 꺼져도(Ctrl+C, `docker/mapf.sh stop`, Gazebo·RViz 창 닫기, 충돌) 실기체 서버는 살아 있고:

1. mirror 노드가 시뮬레이션이 사라진 것을 `sim_timeout`(1초) 안에 감지하고 날던 기체를 바닥까지 착륙시킨다.
2. 착륙이 끝나면 비행했던 기체에 모터 정지(emergency)를 보내고 종료한다. 실기체 서버도 함께 닫힌다.
3. `docker/mapf.sh`가 이를 최대 25초 기다린 뒤 무선으로 전원을 끄고, 응답이 없어진 것까지 확인한다
   (`MAPF_POWER_OFF=0`이면 전원은 켜 둔다). USB로 충전 중인 기체는 전원이 다시 켜질 수 있다.

즉시 멈춰야 하면 `docker/mapf.sh estop <번호>`: 서버가 있으면 서버로, 없으면 Crazyradio로 직접 비상 정지를 보낸다.
날던 기체는 그 자리에서 떨어지고, 이후 전원을 껐다 켜야 다시 쓸 수 있다.
터미널·컨테이너가 강제로 죽으면 착륙 명령을 보낼 수 없다. 그때는 `estop`이나 기체 전원 버튼을 쓴다.

### 위치 측정: 모캡 사용 / 미사용 (`mocap:=true|false`)

| | `mocap:=true` (기본) | `mocap:=false` |
| --- | --- | --- |
| 위치 출처 | Qualisys rigid body (이름 = 기체 이름) | 기체 자체 추정 (Flow/Lighthouse/Loco 덱) |
| 시작 위치 | 기체를 시뮬레이션 시작점(+offset)의 실제 좌표에 놓아야 함 | 놓은 자리가 시뮬레이션 시작점이 됨 (기수는 +x 방향) |
| launch 전 검사 | - | 기체마다 덱 확인. 위치 덱이 없거나 연결이 안 되면 그 기체는 시뮬레이션만 |
| 실행 중 | - | mirror가 `kalman.initialX/Y/Z/Yaw`를 시작점으로 설정하고 `resetEstimation` |
| 서버 설정 | Mellinger | 모캡 끈 기체 유형(`*_onboard`), PID 컨트롤러 |

두 모드 모두 이륙 순간 위치가 시작점에서 `home_tolerance` 안에 `settle_time`(2초) 이상 머물러야 띄운다.
위치 덱 없이 모캡도 없으면 추정값이 초당 수 m씩 발산하므로(시험에서 1초에 약 10 m) 이 검사에서 걸러진다.
모캡 없는 모드에서 추종 오차·지오펜스 검사는 기체 자체 추정값 기준이라, 덱의 누적 오차(드리프트)는 잡지 못한다.

### 실기체별 안전 동작 (`config/mirror/mirror.yaml`)

한 실기체에 문제가 생기면 **그 기체만 착륙**하고 시뮬레이션과 다른 기체는 계속한다.

| 조건 | 동작 |
| --- | --- |
| 이륙 순간 시뮬레이션 시작 위치(+`offset`)에서 `home_tolerance` 이상 떨어짐 / `settle_time` 동안 안정적이지 않음 / 위치 없음 | 그 기체는 이륙하지 않음 |
| Qualisys 위치가 `pose_timeout` 이상 끊김 | 착륙 |
| 시뮬레이션 명령이 `command_timeout` 이상 끊김 / `/fleet/states`가 `sim_timeout` 이상 끊김 | 착륙 |
| 시뮬레이션 명령과 실제 위치 차이가 `max_tracking_error` 초과 (`tracking_grace` 이후) | 착륙 |
| 명령 위치가 실기체 `bounds` 밖 | 착륙 |
| 시뮬레이션 착륙 / 이륙 실패 / 비상정지 | 똑같이 착륙 / 착륙 / 비상정지 |

실험실 원점이 시뮬레이션과 다르면 `offset`으로 옮긴다. launch는 계획 공간(+offset)이 실기체 `bounds`
안에 있는지 시작 전에 확인한다.

### 현재 한계

- 기본 시나리오는 3.6 m 공간의 5대 교차이며 SIL에서 모두 도착했다. 7대 교차는 이 느린 설정에서
  3.6 m·5 m 공간 모두 서로 양보하지 못하고 감속·호버로 멈췄다(안전하지만 임무 미완료).
  대수·배치를 바꾸면 시뮬레이션에서 먼저 완료되는지 확인한다. 실기체는 시뮬레이션의 호버도 그대로 따라 한다.
- SIL은 이 PC에서 실시간의 약 0.1~0.2배 속도로 돈다. 실기체는 명령이 오는 대로 따라 하므로 비행 경로는 같지만,
  이륙(고수준 명령, 실제 시간 기준)은 실기체가 먼저 끝나고 시뮬레이션이 따라 올라온다.
- 실기체 연결은 SIL 서버를 `/real`에 하나 더 띄워 대신하는 방식으로 끝까지 시험했다
  (이륙·추종 오차 최대 약 0.11 m·착륙 확인). 실제 무선·Qualisys 비행은 아직 하지 않았다.

## 구현과 검증 범위

중앙 PC에서 기체별 DMPC 가속도 QP를 풀고 예측을 교환한다. 연속 구간 이격, 상태
신선도, 명령 만료, 감속 fallback을 검사한다. Gazebo는 SIL pose를 표시하는 시각화다.
정적 박스와 설정된 왕복 경로를 따르는 동적 박스의 회피를 지원한다. 박스 배송은 포함하지 않는다. 실기체 모드는 아직 실제 비행으로 검증하지 않았다.
계획 갱신은 목표 5Hz이며 마감 시간을 보장하지 않는다. `/trajectory/status`의
`over_budget`과 계획 시간, `/fleet/status`의 도달 상태로 실제 진행을 확인한다.

계획기·실행기·편대·지표·시각화와 관련 회귀 테스트는 기존 교차 실험에서 가져왔다.
감속(BRAKING) 궤적은 정상 추종 중(오차가 `tracking_reset_distance` 이내)이면 측정값이 아니라 직전 기준 궤적에서
시작한다. 측정값에서 시작하면 SIL의 고도 편차 때문에 감속이 반복될 때 기준 고도가 바닥까지 내려갔다.
기존 드론 간 회피에 정적·동적 장애물 제약과 연속 충돌 검사를 추가했다.

참고 문헌은 PBS의 [Searching with Consistent Prioritization for Multi-Agent Path Finding](https://arxiv.org/abs/1812.06356)
및 DMPC 코드에 명시된 [arXiv:1909.05150](https://arxiv.org/abs/1909.05150)이다.
PBS의 `PrioritySearch.solve()`는 외부 `evaluate(candidate)` 충돌 검사기를 받는다.
데모의 합성 조건은 실제 충돌 검사기가 아니다. 논문 전체 재현이나 일반적인 최적성·완전성을 보장하지 않는다.

## 연구 문제 기록

[DMPC 문제 기록과 연구 기여 후보](docs/research_problem_log.md)에 속도 증가 실험에서 확인한
QP 수렴 실패, 회피 제약의 실행 불가능성, SIL 결과 차이와 감속 후 대응의 한계를 정리했다.
확인된 사실과 미검증 가설을 구분하며, 관련 로그와 분석 시점의 코드·설정 사본을 함께 보관한다.
해결책은 아직 운영 코드에 적용하지 않았다.

패키지 공개 전에는 package.xml의 maintainer 이메일을 실제 주소로 바꾼다.
