# crazyflie

Crazyflie 드론 개발 통합 저장소. Docker로 개발 환경을 통일한다.

## 구성

- [`crazyflie-firmware/`](crazyflie-firmware/) — 드론 펌웨어. [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) git submodule.
- [`cf_ws/src/crazyflie-basic/`](cf_ws/src/crazyflie-basic/) — 드론 제어용 ROS 2 패키지 (crazyswarm2 기반). [IN-AIR-KR/crazyflie-basic](https://github.com/IN-AIR-KR/crazyflie-basic) git submodule (`master` 추적).
- [`cf_ws/src/crazyflie_mapf/`](cf_ws/src/crazyflie_mapf/) — 다중 드론 DMPC 경로 계획 + 시뮬레이션을 실제 드론이 따라 하는 mirror 모드. 이 저장소에서 직접 관리하는 ROS 2 패키지.
- [`docker/`](docker/) — 실행 환경 + 편의 스크립트. `mapf.sh`가 `crazyflie_mapf` 실행을 한 명령으로 묶는다.

## 사전 준비

- Docker Engine + Docker Compose v2
- Linux, X11/XWayland (GUI용)
- (선택) GPU 쓸 경우 `nvidia-container-toolkit`
- Crazyradio PA 또는 Crazyflie USB 동글

## 실행

```bash
git clone --recursive https://github.com/IN-AIR-KR/crazyflie.git crazyflie   # 최초 1회
cd crazyflie

./docker/build.sh          # 이미지 빌드
./docker/container_start.sh  # 컨테이너 켜기
./docker/cf_basic.sh       # basic 컨테이너 접속
./docker/cfclient.sh       # cfclient(드론 GUI) 실행
```

GPU 쓰는 경우 `build.sh`/`container_start.sh` 대신:
```bash
docker compose -f docker/compose.yml -f docker/compose.gpu.yml build
docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d
```

**시뮬레이션 확인** (`cf_basic.sh`로 접속한 뒤, 최초 1회 빌드):
```bash
colcon build --symlink-install
```

빌드 후에는 컨테이너 밖에서 `launch.sh`로 바로 실행할 수 있다. `backend:=sim`이면 RViz2와 Gazebo Sim이 함께 열리고, RViz2 창을 닫으면 launch와 Gazebo도 같이 종료된다:
```bash
./docker/launch.sh                    # launch.py 기본값
./docker/launch.sh backend:=sim       # SIL + RViz2 + Gazebo Sim
./docker/launch.sh backend:=sim gazebo:=False  # Gazebo 없이 SIL + RViz2
./docker/launch.sh mode:=mocap backend:=cflib   # 실제 드론
```

Gazebo에는 `crazyflies_<mode>.yaml`에서 활성화된 드론이 생성되며, Crazyswarm2 SIL이 계산한 자세를 30 Hz로 반영한다. 따라서 현재 Gazebo 연동은 **비행 시각화 용도**이고, 충돌·모터·공기역학을 Gazebo가 계산하는 물리 시뮬레이션은 아니다.

### 온라인 다중 드론 궤적 계획

현재 주 구현 목표는 **공통 `world` 좌표계에서 2대/4대가 목적지로 동시에 이동하는
온라인 DMPC 궤적 계획**이다. 한 중앙 PC에서 기체별 최적화 문제를 풀고 미래 궤적을
교환한다. 목적지 할당, 궤적 계획, 공통 full-state 실행기를 분리했다.

```bash
# 이전 launch를 종료한 뒤 실행
./docker/trajectory.sh preview                              # 2대 궤적 확인
./docker/trajectory.sh run                                  # 2대 교차 이동
./docker/trajectory.sh run trajectory_scenario:=crossing4    # 4대 교차 이동
```

계획 속도/예측 시간/수평·수직 안전 이격은
[`crazyflie_delivery/config/dmpc.yaml`](cf_ws/src/crazyflie-basic/crazyflie_test/crazyflie_delivery/config/dmpc.yaml),
기체 시작 위치는 `config/crazyflies_trajectory2.yaml`과 `crazyflies_trajectory4.yaml`에서
설정한다. 새 실행 파일 설치를 위해 패키지를 다시 빌드해야 한다.
[구조와 논문 적용 범위](cf_ws/src/crazyflie-basic/crazyflie_test/DELIVERY_DESIGN.md)를 참고한다.

이 모드는 장애물 없는 공간의 목적지 이동부터 검증한다. 배송 박스와 고정 장애물은
이후 연결할 대상으로 두며, 기존 배송 비교 모드는 아래 명령으로 계속 실행할 수 있다.

> 이 환경은 ROS 2 Jazzy와 공식 조합인 **Gazebo Sim**을 사용한다. 인터넷 자료에서 흔히 보이는 회색 UI의 **Gazebo Classic(Gazebo 11)**과는 다른 후속 제품이다. Gazebo Classic용 플러그인이나 world 파일은 그대로 사용할 수 없다.

### 단일 기체 키보드 조종

직접 관리하는 단일 기체 제어 모듈은
[`crazyflie_delivery/`](cf_ws/src/crazyflie-basic/crazyflie_test/crazyflie_delivery/)에 있다.
`drone_node/functions.py`는 제어 함수, `drone_node/callbacks.py`는 입력 수신,
`drone_node/main.py`는 실행 모드 선택을 담당한다. 할당·경로 계획·충돌 관제도
각 노드 폴더로 나누었다. 빌드와 `drone_main` 실행법은
[패키지 README](cf_ws/src/crazyflie-basic/crazyflie_test/README.md)를 참고한다.

층당 4개 × 4층으로 쌓은 박스 16개와 A/B/C 배송 구역은
[`crazyflie_delivery/gazebo/scene.yaml`](cf_ws/src/crazyflie-basic/crazyflie_test/crazyflie_delivery/gazebo/scene.yaml)에서
설정한다. `./docker/launch.sh backend:=sim mode:=opticalflow_multi delivery_scene:=True`로
드론 3대와 배송 장면을 함께 실행한다.

4대의 배송 경로를 RViz에서 확인하거나 실제 시뮬레이션 배송을 실행하려면 다음을
사용한다. 이전 launch를 Ctrl+C로 종료한 뒤 둘 중 하나를 실행한다.

```bash
./docker/delivery.sh preview   # Gazebo + RViz, 드론별 할당 경로 확인
./docker/delivery.sh run       # 4대에 할당하고 한 대씩 운반·적재·복귀·착륙

# 기존 동시 배송 비교 모드: 전역경로 + 구역 예약 + TTC
./docker/delivery.sh preview delivery_strategy:=coordinated
./docker/delivery.sh run delivery_strategy:=coordinated
```

기존 동작은 `delivery_strategy:=baseline`으로 보존했다. 새 구조/설정/토픽/검증 방법은
[다중 드론 관제 안내](cf_ws/src/crazyflie-basic/crazyflie_test/COORDINATED_DELIVERY.md)를 참고한다.

경로 색은 cf1 청록 / cf2 주황 / cf3 보라 / cf4 노랑이다. 이 명령은 Gazebo 시뮬레이션용이며
자세한 구성과 하드웨어 연결 조건은 [배송 코드 안내](cf_ws/src/crazyflie-basic/crazyflie_test/README.md)를 참고한다.

`single_cf.py`는 소스에서 직접 실행되므로 텔레옵 스크립트 사용에는 재빌드가 필요 없다. `ros2 run crazyflie_test single_cf` 명령도 사용하려면 최초 한 번 패키지를 다시 빌드한다:

```bash
docker compose -f docker/compose.yml exec basic bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   colcon build --symlink-install --packages-select crazyflie_test"
```

터미널 1에서 서버와 시각화를 실행하고, 터미널 2에서 키보드 조종기를 실행한다:

```bash
# 터미널 1: 시뮬레이션 + RViz2 + Gazebo Sim
./docker/launch.sh backend:=sim

# 터미널 2: 키보드 입력을 받는 단일 기체 조종기
./docker/single_cf.sh
```

시뮬레이션에서는 자동으로 `setpoint` 모드가 선택된다. 주요 키는 `t` 이륙, `l` 착륙, `w/a/s/d` 수평 이동, `q/e` 회전, `r/f` 상승·하강, `h` 정지, `Esc` 안전 종료이며 `x`는 즉시 모터를 끄는 비상 정지다. 옵션은 그대로 전달할 수 있다:

```bash
./docker/single_cf.sh --height 0.5 --speed 0.3
./docker/single_cf.sh --mode setpoint
```

**실제 드론 사용**:
1. Crazyradio를 꽂는다.
2. `./docker/cfclient.sh`에서 "Scan"으로 드론 연결 (필요하면 Bootloader 탭에서 펌웨어 플래시).
3. `cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml`의 `uri`를 내 드론 주소로 수정 후 `backend:=cflib`로 launch.

**끝내기**:
```bash
docker compose -f docker/compose.yml down
```

## 다중 드론 경로 계획 + 실제 드론 따라 하기 (`crazyflie_mapf`)

시뮬레이션 속 드론 5대가 **서로 부딪히지 않으면서 동시에 반대편 목적지로 이동**하고,
무선으로 연결된 실제 드론은 **같은 번호의 시뮬레이션 드론과 똑같이 움직인다.**
예를 들어 실제 cf5만 켜 두면, 실제 cf5가 시뮬레이션 cf5의 이륙·이동·착륙을 그대로 따라 한다.

### 방법론

**1. 경로를 어떻게 만드나 (DMPC: 분산 모델 예측 제어)**

- 드론마다 **앞으로 4초 동안**의 움직임을 0.2초 단위로 계획한다. 계획하는 값은 매 순간의 가속도다.
- 이 계획을 **0.2초마다(5Hz) 새로 계산**한다. 실제 위치가 조금 어긋나도 다음 계산에서 바로잡힌다.
- 드론들은 서로의 계획(앞으로 4초 경로)을 주고받는다. 두 경로가 가까워지는 순간이 보이면
  "이 시점에는 상대에게서 이만큼 떨어져라"는 조건을 추가하고 다시 계산한다(최대 8번).
- 드론마다 **보이지 않는 안전 풍선**(가로 반지름 0.52m, 세로 1.1m 타원체)이 있고, 풍선끼리 겹치지 않게 한다.
  세로가 긴 이유는 위 드론의 하강 기류(downwash)를 피하기 위해서다.
- 안전한 경로를 끝내 못 찾으면 **모두 감속해 제자리에 멈춘다**(BRAKING). 부딪히는 대신 멈추는 쪽을 택한다.
- 참고 논문: Luis, Vukosavljev, Schoellig, *Online Trajectory Generation With Distributed Model Predictive
  Control for Multi-Robot Motion Planning*, RA-L 2020 ([arXiv:1909.05150](https://arxiv.org/abs/1909.05150)).
  그대로 재현한 것은 아니고, 계산 변수와 충돌 조건을 단순화해 적용했다.

**2. 기본 시나리오**

| 항목 | 값 |
| --- | --- |
| 공간 | 3.6m × 3.6m, 높이 0.2~2.0m |
| 배치 | 반경 1.2m 원 위에 5대, 각자 원의 반대편으로 이동, 높이 1.0m |
| 속도 | 최대 0.5m/s, 가속 0.3 / 감속 0.4 m/s² |
| 끝 | 모두 도착하면 2초 호버 후 자동 착륙 |

같은 공간에 7대를 두면 가운데가 너무 붐벼서 서로 양보하지 못하고 멈췄다. 그래서 5대로 줄였다.

**3. 실제 드론은 어떻게 따라 하나 (mirror)**

```text
경로 계획기 → 시뮬레이션 실행기 ──명령──→ 시뮬레이션 드론 5대
                    │
                    └── mirror 노드 ──같은 명령──→ 실제 드론 (무선 연결된 것만)
```

- **시뮬레이션이 기준**이다. 경로 계획기는 시뮬레이션 드론만 보고 계획한다.
- mirror 노드는 시뮬레이션 드론에게 가는 명령(위치·속도·가속도)을 같은 번호의 실제 드론에게도 보낸다.
  이륙·착륙·비상정지도 시뮬레이션과 같은 순간에 보낸다.
- 실제 드론에 문제가 생기면 **그 드론만 착륙**하고, 시뮬레이션과 다른 드론은 계속한다.
  (위치 정보 끊김, 시뮬레이션 명령 끊김, 명령과 0.3m 넘게 어긋남, 비행 공간 밖으로 나감)

**4. 실제 드론의 위치는 어떻게 아나**

| | `mocap:=true` (기본) | `mocap:=false` |
| --- | --- | --- |
| 위치 출처 | Qualisys 모션캡처 (QTM rigid body 이름 = 드론 이름) | 드론 스스로 추정 (아래쪽 Flow deck 카메라 + 거리 센서) |
| 놓는 위치 | 시뮬레이션 시작점의 실제 좌표에 정확히 | 아무 데나. 놓인 자리가 시작점이 된다 (기수는 +x 방향) |
| 정확도 | 실제 위치 | 출발점 기준 상대 위치. 시간이 지날수록 오차가 쌓인다 |
| 필요 장비 | Qualisys + 같은 네트워크 | Flow/Lighthouse/Loco 덱 (없으면 그 드론은 제외) |

**5. 끄면 어떻게 되나**

Ctrl+C, `docker/mapf.sh stop`, Gazebo·RViz 창 닫기 모두 같은 순서로 끝난다:
시뮬레이션 종료 → 실제 드론 착륙 → 모터 정지 → **실제 드론 전원 끔**(최대 25초).
실제 드론 쪽 프로그램은 시뮬레이션과 따로 돌기 때문에, 시뮬레이션이 갑자기 꺼져도 드론을 내릴 수 있다.

### 명령어

모두 호스트의 `~/crazyflie`에서 실행한다. NVIDIA GPU가 있으면 자동으로 GPU 렌더링을 쓴다.

```bash
# 0. 처음 한 번: 이미지(OSQP 포함) + 패키지 빌드
docker/mapf.sh build
docker/mapf.sh help                              # 전체 명령 목록

# 1. 실제 드론이 잡히는지 확인 (실행 중인 mirror가 없을 때)
docker/mapf.sh scan                              # 드론 / 배터리 / 덱 (약 30초)

# 2. 실행 (터미널 1)
docker/mapf.sh sim                               # 시뮬레이션 5대만
docker/mapf.sh mirror real:=cf5 mocap:=false     # + 실제 cf5가 따라 함 (Flow deck)
docker/mapf.sh mirror real:=cf2 mocap:=true      # + 실제 cf2가 따라 함 (Qualisys)
docker/mapf.sh mirror mocap:=false               # 켜져 있는 드론 자동 선택
MAPF_POWER_OFF=0 docker/mapf.sh mirror real:=cf5 mocap:=false   # 끝나도 드론 전원 유지

# 3. 조작 (터미널 2)
docker/mapf.sh status        # "cf5 WAITING ready (start 가능)"이면 출발 가능
docker/mapf.sh start         # 실제 드론이 있을 때만 필요: 시뮬레이션 + 실제 드론 동시 출발
docker/mapf.sh land          # 전체 착륙
docker/mapf.sh estop 5       # cf5만 강제 정지 (모터 즉시 끔). estop 2 5 / estop all
docker/mapf.sh emergency     # 실행 중인 전체 비상 정지

# 4. 종료
docker/mapf.sh stop          # 또는 터미널 1에서 Ctrl+C
```

**실제 드론 한 대로 비행하는 순서** (예: cf5, 모캡 없이)

```bash
# 터미널 1 (cf5를 평평한 곳에 두고 켠 뒤 손대지 않기, 기수는 +x 방향)
docker/mapf.sh mirror real:=cf5 mocap:=false

# 터미널 2 (터미널 1 로그에 "[mirror] 실기체: cf5"가 나온 뒤)
docker/mapf.sh status    # cf5 WAITING ready (start 가능) 확인
docker/mapf.sh start

# 터미널 3 (만일을 위해 미리 입력만 해두고, 위험하면 Enter)
docker/mapf.sh estop 5
```

- 실제 드론이 연결되면 시뮬레이션도 `start`를 기다린다. 시뮬레이션만 돌릴 때는 바로 출발한다.
- 비상 정지나 추락 후 드론은 잠긴 상태(`Locked, reboot required`)가 된다. 전원을 껐다 켜야 다시 쓸 수 있다.
- 설정 파일: [`config/mirror/`](cf_ws/src/crazyflie_mapf/config/mirror/) (시뮬레이션·속도·공간·안전 기준),
  [`config/hardware/`](cf_ws/src/crazyflie_mapf/config/hardware/) (실제 드론 주소·모션캡처).
  자세한 설명은 [패키지 README](cf_ws/src/crazyflie_mapf/README.md).

### 주의: 모캡 없는 모드는 아직 위험하다 (2026-11-06 시험)

- **cf5가 이륙 후 약 3m까지 올라갔다.** 바닥에서 기다리는 동안 드론의 자체 위치 추정이 흘러가
  (가로 1.5m 이상) 높이도 실제보다 낮게 잘못 잡힌 것으로 보인다. 드론은 "아직 1m가 안 됐다"고 판단해 계속 올라갔다.
- **cf2는 이륙 중 재부팅 2회, 이후 이륙 1.7초 만에 뒤집혔다.** 소프트웨어 명령 전 단계라 드론 자체(프로펠러, 배터리, 덱 접촉) 문제로 본다.
- 아래를 고치기 전까지 `mocap:=false` 비행은 **프로펠러를 빼거나 안전망 안에서만** 시험한다.
  1. 이륙 직전에 위치 추정을 초기화하고 안정될 때까지 기다리기 (지금은 연결할 때 한 번만 함)
  2. 바닥 대기 중 "안정" 기준 강화 (2초에 0.15m → 0.05m)
  3. Flow deck 거리 센서로 **실제 높이**를 감시해 상한을 넘으면 착륙
- 시뮬레이션은 이 PC에서 실제 시간의 약 0.12~0.2배로 느리게 돈다. 실제 드론에 보내는 속도·가속도 값은
  시뮬레이션 시간 기준이라 실제보다 크다. 배속 보정은 아직 넣지 않았다.

## 문제 해결

USB/GPU/X11/의존성 이슈는 [`docker/README.md`](docker/README.md) 참고.
