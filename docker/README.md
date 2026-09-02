# Docker 환경 상세 안내

두 개의 서비스로 구성되어 있다.

| 서비스 | 역할 | 베이스 이미지 |
|---|---|---|
| `firmware` | `crazyflie-firmware` 빌드/플래시, `cfclient` GUI | `ubuntu:24.04` + pixi |
| `basic` | ROS 2 Jazzy 기반 crazyswarm2 비행 예제 (`crazyflie-basic`) | `osrf/ros:jazzy-desktop` |

## GPU 없는 팀원 vs 있는 팀원

GPU가 없어도 두 서비스 모두 완전히 동작한다 (지금 당장은 어느 쪽도 GPU 연산을 쓰지 않음). GPU는 향후 인지/VLN 모듈 연동을 위해 미리 배선만 해둔 상태다.

```bash
# GPU 없는 머신
docker compose -f docker/compose.yml build
docker compose -f docker/compose.yml up -d

# GPU 있는 머신 (nvidia-container-toolkit 설치 필요)
docker compose -f docker/compose.yml -f docker/compose.gpu.yml build
docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d
```

GPU 조합을 쓰는 경우, 먼저 아래로 `nvidia-container-toolkit`이 제대로 동작하는지 확인:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu24.04 nvidia-smi
```

## X11 (GUI) 준비

호스트에서 컨테이너의 X11 접근을 매 로그인/재부팅마다 한 번 허용해야 한다:

```bash
xhost +local:docker
```

## USB / Crazyradio

`privileged: true` + `/dev/bus/usb:/dev/bus/usb` 마운트로 처리한다. 이 레포에는 Crazyradio용 udev 규칙(`99-bitcraze.rules`)이 없어 디바이스 번호가 리플러그마다 바뀌므로, 매 팀원 호스트에 udev 규칙을 설치시키는 대신 privileged 모드로 단순화했다 (컨테이너 격리를 약화시키는 트레이드오프를 감수한 의도적 선택). Crazyradio는 컨테이너 시작 전/후 아무 때나 꽂아도 되고, 재시작 없이 바로 인식된다.

더 강한 격리가 필요하면, 호스트에 표준 Bitcraze `99-bitcraze.rules`를 설치하고 `plugdev` 그룹에 사용자를 추가한 뒤, compose에서 `privileged: true`를 제거하고 `devices:`로 정확한 `/dev/bus/usb/<bus>/<device>` 경로만 지정하는 방식으로 바꿀 수 있다.

**주의**: `privileged: true`는 커널 device cgroup 제약만 풀어줄 뿐, `/dev/bus/usb/*` 장치 파일 자체의 유닉스 권한(대개 `root:root`, 660)까지 바꿔주지는 않는다. 그래서 두 이미지 모두 entrypoint에서 컨테이너 시작 시 `fix-usb-perms`(`sudo chmod -R o+rw /dev/bus/usb`)를 자동 실행해 non-root `dev` 유저도 Crazyradio/Crazyflie USB에 접근할 수 있게 해뒀다. **컨테이너가 이미 떠 있는 상태에서 Crazyradio를 새로 꽂았다면** (최초 chmod 시점을 놓치므로) 컨테이너 안에서 다시 실행:

```bash
fix-usb-perms
```

연결 확인:

```bash
lsusb | grep -i "1915:7777\|Bitcraze"   # Crazyradio PA
```

## UID/GID (bind mount 파일 소유권)

컨테이너 안에서 생성된 파일이 호스트에서 root 소유가 되지 않도록, 빌드 전에 `docker/.env`를 만든다:

```bash
echo "UID=$(id -u)" > docker/.env
echo "GID=$(id -g)" >> docker/.env
```

## 컨테이너 진입

```bash
docker compose -f docker/compose.yml exec firmware bash
docker compose -f docker/compose.yml exec basic bash
```

## 스모크테스트

**basic (시뮬레이션, 하드웨어 불필요):**

```bash
docker compose -f docker/compose.yml exec basic bash
colcon build --symlink-install
source install/setup.bash
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim
```

다른 터미널에서:

```bash
docker compose -f docker/compose.yml exec basic bash -c "source install/setup.bash && rviz2"
```

**firmware (GUI):**

```bash
docker compose -f docker/compose.yml exec firmware bash
pixi run cfclient
```

## 실제 드론(하드웨어) 사용

1. Crazyradio PA(또는 Crazyflie USB 동글)를 호스트에 꽂는다.
2. 컨테이너가 이미 실행 중이었다면 `fix-usb-perms` 실행 (위 USB 섹션 참고). 컨테이너를 새로 띄우는 경우는 entrypoint가 자동으로 처리하므로 생략 가능.
3. firmware 컨테이너에서 cfclient 실행:
   ```bash
   docker compose -f docker/compose.yml exec firmware bash
   pixi run cfclient
   ```
   cfclient GUI에서 "Scan"으로 Crazyradio 인식 및 드론 주소(`radio://0/80/2M/E7E7E7E7E7` 등)를 확인하고 연결한다.
4. 새로 빌드한 펌웨어를 실제 드론에 플래시하려면 cfclient의 Bootloader 탭(또는 CLI `cfloader`)을 사용한다. `crazyflie-firmware`는 `pixi run make`로 빌드하면 `build/` 아래 `.bin`이 생성된다 (호스트의 `crazyflie-firmware/build/`는 `.gitignore` 처리되어 있음).
5. cfclient의 설정(스캔 이력, 로그 설정, 입력장치 매핑 등)은 `cfclient_config` named volume(`/home/dev/.config/cfclient`)에 저장되어 `docker compose down`/`up`을 반복해도 유지된다. 완전히 초기화하려면 `docker compose -f docker/compose.yml down -v`.
6. `basic` 컨테이너로 실비행(mocap/opticalflow) 예제를 돌릴 때는 `cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml`(컨테이너 안에서는 `/workspace/cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml`)에서 `uri`(라디오 주소)와 `initial_position`을 실제 드론에 맞게 수정한 뒤 `backend:=cflib` 또는 `backend:=cpp`로 launch한다 (crazyflie-basic README의 `mode`/`backend` 조합 참고). `basic` 컨테이너도 동일한 `/dev/bus/usb` 마운트 + `fix-usb-perms`로 라디오에 접근한다.

## GPU 확인 (compose.gpu.yml을 얹었을 때)

```bash
docker compose -f docker/compose.yml -f docker/compose.gpu.yml exec basic nvidia-smi
docker compose -f docker/compose.yml -f docker/compose.gpu.yml exec firmware nvidia-smi
```

## 알려진 갭 / 참고사항

- `cflib`은 crazyswarm2의 `crazyflie_server_py`/`flash.py`가 실제로 쓰지만 upstream 어디에도 의존성으로 선언되어 있지 않다. `docker/basic/requirements.txt`에 명시적으로 pin 해뒀다 — 지우지 말 것.
- 호스트의 `cf_ws/build|install|log|cache`는 예전에 ROS 2 Humble로 빌드된 산출물로, 이 Docker 환경에서는 전혀 사용하지 않는다 (컨테이너는 named volume에 자체 빌드 산출물을 갖는다). 안전하게 `rm -rf cf_ws/build cf_ws/install cf_ws/log cf_ws/cache`로 지워도 된다.
- `pixi.toml`/`package.xml` 등 의존성 정의가 바뀌면 이미지를 다시 빌드해야 한다: `docker compose -f docker/compose.yml build --no-cache <서비스명>`.
- colcon 빌드 산출물을 완전히 초기화하려면 `docker compose -f docker/compose.yml down -v` (named volume까지 삭제).

## 리스크

- X11은 Xorg/XWayland 전제. 순수 Wayland 호스트는 별도 소켓 방식이 필요하며 현재 미지원.
- `nvidia-container-toolkit`은 `compose.gpu.yml`을 실제로 쓰는 머신에만 필요하다.
- `privileged: true`는 소규모 신뢰 팀 환경을 전제로 한 의도적 트레이드오프다.
- Rootless Docker, macOS/Windows 호스트는 검증되지 않았다.
