# crazyflie

Crazyflie 드론 개발을 위한 통합 저장소. Docker Compose로 팀원 누구나 동일한 개발 환경을 재현할 수 있도록 구성했다.

- [`crazyflie-firmware/`](crazyflie-firmware/) — [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) git submodule. 거의 건드릴 일 없는 STM32 임베디드 펌웨어라 upstream과 연결을 유지한다.
- [`cf_ws/src/crazyflie-basic/`](cf_ws/src/crazyflie-basic/) — [IN-AIR-KR/crazyflie-basic](https://github.com/IN-AIR-KR/crazyflie-basic)에서 가져온 Crazyswarm2 기반 ROS 2 비행 예제. **submodule이 아니라 이 저장소 소속의 일반 파일**이다 — 원본 `crazyflie-basic`과는 연결이 끊겨 있어서, 여기서 무엇을 커밋/push해도 원본에는 영향이 없다. 대신 원본이 나중에 업데이트돼도 자동으로 따라오지 않으니, 필요하면 수동으로 반영해야 한다.

## 폴더 구조

```
crazyflie/
├── docker/                        # Docker Compose 환경 (아래 참고)
├── cf_ws/
│   └── src/crazyflie-basic/       # 일반 폴더 (원본과 분리된 개발용 사본)
└── crazyflie-firmware/            # git submodule
```

## 사전 준비

- Docker Engine + Docker Compose v2
- Linux 호스트, X11/XWayland (GUI: rviz2, cfclient)
- (선택) GPU를 쓸 경우 `nvidia-container-toolkit`
- Crazyradio PA 또는 Crazyflie USB 동글

## 퀵스타트 (처음 하시는 분용, 순서대로 따라하면 됩니다)

### 0. 저장소 받기 (최초 1회)

```bash
git clone --recursive <이 저장소 URL> crazyflie
cd crazyflie
```
`--recursive`는 `crazyflie-firmware`(submodule)를 같이 받기 위한 옵션이다. `cf_ws/src/crazyflie-basic`은 일반 파일이라 별도 submodule 초기화 없이 그냥 딸려온다.

이미 로컬에 폴더가 있다면 이 단계는 건너뛰세요.

### 1. UID/GID 설정 (최초 1회)

컨테이너 안에서 만든 파일이 호스트에서 root 소유가 되지 않도록 내 계정 정보를 넣어둡니다.

```bash
echo "UID=$(id -u)" > docker/.env
echo "GID=$(id -g)" >> docker/.env
```

### 2. X11 GUI 권한 허용 (터미널을 새로 열거나 재부팅할 때마다 1회)

rviz2, cfclient 같은 GUI 프로그램을 컨테이너 안에서 띄우려면 필요합니다.

```bash
xhost +local:docker
```

### 3. 이미지 빌드 + 컨테이너 기동

```bash
docker compose -f docker/compose.yml build
docker compose -f docker/compose.yml up -d
```

GPU가 있는 컴퓨터라면 (선택, `nvidia-container-toolkit` 설치 필요) 위 두 줄 대신:
```bash
docker compose -f docker/compose.yml -f docker/compose.gpu.yml build
docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d
```

### 4. `basic` 컨테이너 접속 → 시뮬레이션으로 동작 확인 (드론 없이 가능)

```bash
docker compose -f docker/compose.yml exec basic bash
```

컨테이너 안에서:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim
```
첫 `colcon build`는 crazyswarm2 전체를 빌드해서 시간이 좀 걸립니다. 완료되면 새 터미널에서 아래 명령으로 rviz2를 띄워 시각화까지 확인해보세요.

```bash
docker compose -f docker/compose.yml exec basic bash -c "source install/setup.bash && rviz2"
```

### 5. `firmware` 컨테이너 접속 → cfclient GUI 확인

```bash
docker compose -f docker/compose.yml exec firmware bash
pixi run cfclient
```
최초 실행 시 `pixi install`이 자동으로 한 번 더 돌아갑니다 (시간이 좀 걸릴 수 있음).

### 6. 실제 드론(Crazyflie)으로 테스트할 때

1. 컴퓨터에 Crazyradio를 꽂습니다.
2. 컨테이너가 이미 실행 중이었다면 컨테이너 안에서 `fix-usb-perms`를 한 번 실행합니다 (새로 띄우는 경우는 자동 처리되어 생략 가능).
3. `firmware` 컨테이너의 cfclient에서 "Scan"으로 라디오/드론을 찾아 연결하고, 필요하면 Bootloader 탭으로 펌웨어를 플래시합니다.
4. `basic` 컨테이너로 실비행 예제를 돌리려면 `cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml`에서 `uri`(라디오 주소)를 실제 드론에 맞게 수정한 뒤 `backend:=cflib`로 launch합니다.

### 7. 끝났으면 정리

```bash
docker compose -f docker/compose.yml down
```

막히는 부분(USB/GPU/X11 등)은 [`docker/README.md`](docker/README.md)의 트러블슈팅 섹션을 참고하세요.
