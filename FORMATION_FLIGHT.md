# 편대비행 (formation flight) 정리

## 위치

프로젝트 전체가 `~/다운로드/crazyflie` 로 옮겨졌습니다 (원래 `~/crazyflie`).

```
~/다운로드/crazyflie/
├── docker/                      실행 스크립트 (원래 있던 것 + 이번에 추가한 2개)
│   ├── container_start.sh       컨테이너 켜기
│   ├── launch.sh                서버 + RViz2(+Gazebo) 실행
│   ├── single_cf.sh             드론 1대 키보드 조종 (원래 있던 것)
│   ├── formation_flight.sh      ← 추가: 대열 유지하며 경로(사각형/직진) 자동 비행
│   └── formation_teleop.sh      ← 추가: 키보드로 이륙/전진/착륙 수동 조종
└── cf_ws/src/crazyflie-basic/   ROS2 패키지 (git submodule, branch: formation-flight)
    └── crazyflie_test/
        ├── config/crazyflies_opticalflow_multi.yaml   몇 대 띄울지 설정
        └── crazyflie_test/
            ├── formation_flight.py   편대 대열(line/v/circle) + 경로(사각형/직진) 자동 비행
            └── formation_teleop.py   키보드 t(이륙)/f(전진)/l(착륙)
```

## 드론 몇 대? → yaml 하나로 결정

`crazyflie_test/config/crazyflies_opticalflow_multi.yaml`

```yaml
cf1: enabled: true   # 지금 보유한 2대 — 월요일 실제 비행에 씀
cf2: enabled: true
cf3: enabled: false  # 정의는 해뒀음, 드론 늘면 true로 켜고 uri만 채우면 됨
cf4: enabled: false
```

코드(`formation_flight.py`, `formation_teleop.py`)는 건드릴 필요 없이, 여기서 `enabled`만 바꾸면 몇 대가 뜰지 자동으로 따라갑니다.

## sim이냐 실기체냐 — 파일은 하나, 자동 감지

`launch.py`에 주는 `backend:=sim` / `backend:=cflib` 인자만 다르고, 내부적으로 어떤 서버(시뮬 서버 vs Crazyradio 드라이버)를 켤지만 바뀝니다. 둘 다 동일한 ROS 인터페이스를 내놓기 때문에, `formation_flight.py`와 `formation_teleop.py`는 코드 수정 없이 그대로 양쪽에서 동작합니다.

- `formation_flight.py`는 실기체인지 자동 감지(`/cf1/pose` 토픽 유무)해서 **absolute 모드**(sim/mocap, 대열 모양 변경 가능) 또는 **broadcast 모드**(실기체, 대열 고정·안전)를 스스로 고릅니다.
- `formation_teleop.py`는 항상 broadcast(상대 이동)만 쓰므로 sim/실기체 구분 없이 동일하게 동작합니다.

## 실행 방법

### 지금 — 시뮬레이션

```bash
cd ~/다운로드/crazyflie/docker
./container_start.sh                              # 컨테이너 켜기
./launch.sh mode:=opticalflow_multi backend:=sim  # 터미널 1: 서버 + RViz2 + Gazebo
```

터미널 2:
```bash
./formation_teleop.sh          # 키보드: t 이륙, f 전진, l 착륙
# 또는
./formation_flight.sh --shape line --path line --side 2.0   # 자동으로 전진→복귀→착륙
```

### 월요일 — 실제 드론 2대

1. Crazyradio 동글 꽂기 → `./container_start.sh`
2. `./cfclient.sh` → Scan으로 드론 2대 연결/배터리 확인
3. `crazyflies_opticalflow_multi.yaml`의 cf1/cf2 `uri`를 실제 주소로 교체
4. 드론 2대를 1m 간격으로 나란히 배치 (바닥에 무늬 있는 곳)
5. 터미널 1: `./launch.sh mode:=opticalflow_multi backend:=cflib`
6. 터미널 2: `./formation_teleop.sh` 또는 `./formation_flight.sh` — 자동으로 broadcast 모드
7. 먼저 `./single_cf.sh`로 1대씩 개별 확인 후 편대로 진행 권장

## 확인된 것 (자동 키입력으로 검증 완료)

- `formation_flight.py`: 2~4대, line/v/circle 대열, 사각형·직진 경로 — 서버 로그 좌표 전부 일치 확인
- `formation_teleop.py`: t → f → f → l 순서로 실행, 서버 로그에 `takeoff` → `go_to(1,0,0,relative=1)` ×2 → `land` 정확히 기록됨
- 실제 하드웨어 비행은 아직 미검증 (월요일 예정)

## 커밋 상태

submodule(`cf_ws/src/crazyflie-basic`)은 `formation-flight` 브랜치, 부모 repo(`crazyflie`)는 `main` 브랜치에 로컬 커밋만 되어 있고 **어디에도 push 안 함** (IN-AIR-KR 원격 저장소는 그대로).
