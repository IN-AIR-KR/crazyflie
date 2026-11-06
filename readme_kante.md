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
