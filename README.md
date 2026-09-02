# crazyflie

Crazyflie 드론 개발을 위한 통합 저장소. 두 패키지를 git submodule로 묶고, Docker Compose로 팀원 누구나 동일한 개발 환경을 재현할 수 있도록 구성했다.

- [`crazyflie-firmware/`](crazyflie-firmware/) — [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) fork. STM32 임베디드 펌웨어.
- [`cf_ws/src/crazyflie-basic/`](cf_ws/src/crazyflie-basic/) — [IN-AIR-KR/crazyflie-basic](https://github.com/IN-AIR-KR/crazyflie-basic) fork. Crazyswarm2 기반 ROS 2 비행 예제.

## 폴더 구조

```
crazyflie/
├── docker/                        # Docker Compose 환경 (아래 참고)
├── cf_ws/
│   └── src/crazyflie-basic/       # git submodule
└── crazyflie-firmware/            # git submodule
```

## 사전 준비

- Docker Engine + Docker Compose v2
- Linux 호스트, X11/XWayland (GUI: rviz2, cfclient)
- (선택) GPU를 쓸 경우 `nvidia-container-toolkit`
- Crazyradio PA 또는 Crazyflie USB 동글

## 퀵스타트

```bash
git clone --recursive <이 저장소 URL> crazyflie
cd crazyflie

echo "UID=$(id -u)" > docker/.env
echo "GID=$(id -g)" >> docker/.env

xhost +local:docker

docker compose -f docker/compose.yml build
docker compose -f docker/compose.yml up -d
# GPU가 있다면 위 두 줄 대신:
#   docker compose -f docker/compose.yml -f docker/compose.gpu.yml build
#   docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d

docker compose -f docker/compose.yml exec basic bash
```

컨테이너 안에서:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim
```

자세한 서비스별 설정, USB/GPU/X11 트러블슈팅, 알려진 이슈는 [`docker/README.md`](docker/README.md)를 참고.
