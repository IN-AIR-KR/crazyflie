# 컨테이너 안에서 빌드

cd /home/kante/crazyflie
./docker/launch.sh mode:=opticalflow backend:=sim


source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

exit

# 모션캡쳐용

# 1. 서버
# 터미널 1: 서버에서 모션캡처 사용 선택
ROS_DOMAIN_ID=73 ros2 launch crazyflie_test launch.py \
  mode:=mocap backend:=cpp rviz:=True


# 2. 키보드 제어 (모션캡쳐 위치 이용)
# 터미널 2: 키보드 제어
ROS_DOMAIN_ID=73 ros2 run crazyflie_test single_cf \
  --mode hover --height 0.5



# --------------------------------------------

# 0. docker start
cd /home/kante/crazyflie
./docker/container_start.sh


# 터미널1에서, 시뮬레이션 실행
cd /home/kante/crazyflie
./docker/launch.sh mode:=opticalflow backend:=sim


# 터미널2에서 비행 명령
cd /home/kante/crazyflie
./docker/cf_basic.sh
source install/setup.bash
ros2 run crazyflie_test 


# ----------------------------------------------
# container 안에 필요한 것들
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rosbridge-suite \
    nodejs npm \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------
# multi drone
cd /home/kante/crazyflie
./docker/container_start.sh                                    # 컨테이너 기동 (한 번만)
./docker/launch.sh mode:=opticalflow_multi backend:=sim         # cf1/cf2/cf3 + RViz


cd /home/kante/crazyflie
./docker/cf_basic.sh          # 컨테이너 진입
source install/setup.bash


# ================================================================
# crazyflie_mapf: 시뮬레이션 5대 + 실기체 따라 하기 (2026-11-06 작업)
# ================================================================
# 모든 명령은 호스트 ~/crazyflie 에서 실행한다. GPU가 있으면 자동으로 GPU 렌더링을 쓴다.
# 자세한 설명: cf_ws/src/crazyflie_mapf/README.md

# ---- 0. 처음 한 번 (이미지에 OSQP 포함 + 패키지 빌드) ----
docker/mapf.sh build
docker/mapf.sh help                         # 전체 명령 목록

# ---- 1. 기체 확인 (mirror를 끈 상태에서) ----
docker/mapf.sh scan                         # 잡히는 기체 / 배터리 / 덱 (약 30초)
docker/mapf.sh scan --no-info               # 기체 목록만 빠르게

# ---- 2. 실행 (터미널 1) ----
docker/mapf.sh sim                          # 시뮬레이션 5대만 (3.6m). 도착하면 자동 착륙
docker/mapf.sh mirror real:=cf5 mocap:=false    # 시뮬 5대 + 실제 cf5가 시뮬 cf5를 따라 함 (Flow deck 위치 추정)
docker/mapf.sh mirror real:=cf2 mocap:=true     # Qualisys 모캡 사용 (QTM rigid body 이름 = cf2)
docker/mapf.sh mirror mocap:=false              # 무선 응답하는 기체를 자동으로 고름
MAPF_POWER_OFF=0 docker/mapf.sh mirror real:=cf5 mocap:=false   # 종료해도 실기체 전원 켜 둠
docker/mapf.sh sim gazebo:=false            # RViz만 (가볍게)
# 컨테이너 안에서 직접: ros2 launch crazyflie_mapf mirror.launch.py real:=cf2 software_rendering:=false

# ---- 3. 조작 (터미널 2) ----
docker/mapf.sh status                       # [fleet] 도착 상태 / [mirror] 실기체 상태
                                            #   "cf5 WAITING ready (start 가능)" 이면 출발 가능
docker/mapf.sh start                        # 실기체가 있을 때만 필요: 시뮬 + 실기체 동시 출발
docker/mapf.sh land                         # 전체 착륙
docker/mapf.sh estop 5                      # cf5만 강제 정지 (모터 즉시 끔, 날던 기체는 떨어짐)
docker/mapf.sh estop 2 5 / estop all        # 여러 대 / 전체
docker/mapf.sh emergency                    # 실행 중인 시뮬+실기체 전체 비상 정지

# ---- 4. 종료 ----
# 터미널 1에서 Ctrl+C, Gazebo/RViz 창 닫기, 또는:
docker/mapf.sh stop
# 실기체가 있으면: 시뮬 종료 -> 실기체 착륙 -> 모터 정지 -> 전원 끔 (최대 25초)
# 비상 정지/추락 후 기체는 "Locked, reboot required": 전원을 껐다 켜야 다시 쓸 수 있다.


# ================================================================
# 방법론 요약 (crazyflie_mapf)
# ================================================================
# [궤적 계획] DMPC - Luis, Vukosavljev, Schoellig, RA-L 2020 (arXiv:1909.05150)를 실용적으로 변형
#   - 기체마다 가속도를 변수로 하는 QP(OSQP)를 5Hz로 푼다. 예측 4초, 간격 0.2초.
#   - 기체끼리 예측 궤적을 최대 8번 교환하며(Gauss-Seidel) 충돌 제약을 추가한다.
#   - 기체 간 안전 영역: 수평 0.52m / 수직 1.1m 타원체 (+여유 0.06m). 연속 구간 충돌 검사.
#   - 안전한 궤적이 없으면 전체 감속(BRAKING) -> 호버. 감속도 정상 추종 중이면
#     직전 기준 궤적에서 시작한다 (측정 고도 편차로 바닥까지 내려가던 문제 수정).
#   - PBS(Ma et al., AAAI 2019)는 priority.py에만 있고 비행에는 쓰지 않는다.
#
# [시나리오] 3.6m x 3.6m 공간, 반경 1.2m 원 위 5대가 반대편으로 교차, 높이 1.0m,
#   속도 0.5 m/s, 가속 0.3 / 감속 0.4 m/s^2. 모두 도착하면 2초 후 자동 착륙.
#   (같은 공간에서 7대는 서로 양보하지 못하고 멈췄다 -> 5대로 줄임)
#
# [mirror 구조] 시뮬레이션이 기준(master), 실기체는 같은 이름의 시뮬 기체를 따라 한다.
#   - 시뮬 실행기가 보내는 full-state 명령(/cfN/cmd_full_state)을 mirror 노드가
#     실기체 서버(/real/cfN/cmd_full_state)로 그대로 전달한다. 계획기는 실기체 상태를 보지 않는다.
#   - 이륙/착륙/비상정지는 시뮬 실행기 단계(/fleet/states)를 보고 똑같이 보낸다.
#   - 실기체 쪽(실기체 서버, 모캡, mirror 노드)은 별도 프로세스(mirror_real.launch.py)라
#     시뮬이 어떻게 꺼져도 살아남아 실기체를 착륙시킨 뒤 끝난다.
#   - 기체별 안전장치: 위치 끊김, 시뮬 명령 끊김, 추종 오차 0.3m 초과, 비행 공간 이탈
#     -> 그 기체만 착륙. 시뮬과 다른 기체는 계속.
#
# [위치 측정] mocap:=true  -> Qualisys rigid body. 시작점(+offset)에 놓여 있어야 이륙.
#            mocap:=false -> 기체 자체 추정(Flow/Lighthouse/Loco 덱 필요). 덱이 없으면 제외.
#              연결 시 추정 위치를 시뮬 시작점으로 초기화, 2초 안정 확인 후 이륙,
#              바닥에서 흘러간 만큼 경로 전체를 평행 이동해서 놓인 자리 기준으로 재현.


# ================================================================
# 2026-11-06 실기체 시험 기록과 주의사항
# ================================================================
# - cf2: 덱 없음 -> Flow deck을 옮겨 달고 전원을 다시 켜야 인식됨.
#   이륙 중 재부팅 2회(배터리 3.9V 근처), 이후 이륙 1.7초 만에 뒤집힘(SUP: Crashed).
#   -> 기체 상태 문제로 판단 (프로펠러 방향/손상, 배터리, 덱 접촉 점검 필요)
# - cf5 (Flow2 + Multiranger): 이륙 후 약 3m까지 상승한 뒤 추정기 발산(ESTKALMAN out of bounds).
#   바닥에서 기다리는 동안 자체 추정 위치가 1.5m 이상 흘렀고, 높이 추정도 틀어진 것으로 추정.
#   => 모캡 없는 모드(mocap:=false)는 아래를 고치기 전까지 프로펠러 없이 또는 안전망 안에서만 시험할 것:
#      1) 이륙 직전에 추정기 초기화 + 안정 확인 (지금은 연결 시 1회만)
#      2) 바닥 대기 중 안정성 기준 강화 (2초 0.15m -> 0.05m)
#      3) Flow deck 거리 센서(실제 높이)로 높이 상한 감시 -> 넘으면 착륙
# - 시뮬은 이 PC에서 실시간의 약 0.12~0.2배로 돈다. 실기체로 전달하는 속도/가속도 값은
#   시뮬 시간 기준이라 실제보다 크다 -> 배속 보정 필요 (미적용).
# - 종료하면 실기체 전원이 꺼진다. USB 충전 중인 기체는 다시 켜질 수 있다.
