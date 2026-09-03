# crazyflie

Crazyflie 드론 개발 통합 저장소. Docker로 개발 환경을 통일한다.

## 구성

- [`crazyflie-firmware/`](crazyflie-firmware/) — 드론 펌웨어. [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) git submodule.
- [`cf_ws/src/crazyflie-basic/`](cf_ws/src/crazyflie-basic/) — 드론 제어용 ROS 2 패키지 (crazyswarm2 기반). [IN-AIR-KR/crazyflie-basic](https://github.com/IN-AIR-KR/crazyflie-basic) git submodule (`master` 추적).
- [`docker/`](docker/) — 실행 환경 + 편의 스크립트.

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

빌드 후엔 컨테이너 밖에서 `launch.sh`로 바로 실행 + rviz2 시각화까지 한 번에 (rviz2 창을 닫으면 같이 종료됨):
```bash
./docker/launch.sh                    # launch.py 기본값
./docker/launch.sh backend:=sim       # 시뮬레이션
./docker/launch.sh mode:=mocap backend:=cflib   # 실제 드론
```

**실제 드론 사용**:
1. Crazyradio를 꽂는다.
2. `./docker/cfclient.sh`에서 "Scan"으로 드론 연결 (필요하면 Bootloader 탭에서 펌웨어 플래시).
3. `cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml`의 `uri`를 내 드론 주소로 수정 후 `backend:=cflib`로 launch.

**끝내기**:
```bash
docker compose -f docker/compose.yml down
```

## 문제 해결

USB/GPU/X11/의존성 이슈는 [`docker/README.md`](docker/README.md) 참고.
