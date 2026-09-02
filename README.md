# crazyflie

Crazyflie 드론 개발을 위한 통합 저장소. Docker를 이용해서 팀원 누구나 "내 컴퓨터에서는 되는데 저 사람 컴퓨터에서는 안 돼요" 없이 똑같은 개발 환경을 쓸 수 있도록 만들어뒀다.

git이나 Docker를 처음 써보신다면 아래 순서대로 천천히 따라오시면 됩니다. 막히는 부분은 맨 아래 [자주 발생하는 문제](#자주-발생하는-문제)를 참고하세요.

## 이 프로젝트는 뭔가요?

- [`crazyflie-firmware/`](crazyflie-firmware/) — 드론 본체(하드웨어)에 들어가는 펌웨어 코드. [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)(드론 제조사 공식 저장소) 코드를 그대로 가져와서 씀.
- [`cf_ws/src/crazyflie-basic/`](cf_ws/src/crazyflie-basic/) — 컴퓨터에서 드론에게 "이렇게 날아라"라고 명령을 보내는 ROS 2 프로그램. [IN-AIR-KR/crazyflie-basic](https://github.com/IN-AIR-KR/crazyflie-basic)에서 가져온 뒤 이 저장소 소속으로 만들어서, 여기서 자유롭게 수정/개발하면 됩니다 (자세한 이유는 [용어 정리](#git-용어-아주-간단히)의 "submodule" 항목 참고).
- [`docker/`](docker/) — 위 두 프로그램을 실행할 환경(Docker) 설정.

## 처음 보는 용어라면 (아주 간단히)

이 프로젝트를 쓰는 데 필요한 만큼만 짧게 설명합니다. 이미 아시면 건너뛰세요.

### git 용어

- **git**: 코드 변경 이력을 기록/공유하는 도구. "누가 언제 뭘 고쳤는지"를 계속 저장해준다.
- **저장소(repository, repo)**: git이 관리하는 프로젝트 폴더 하나. 지금 보고 있는 `crazyflie` 폴더 전체가 저장소다.
- **clone**: GitHub에 있는 저장소를 내 컴퓨터로 통째로 복사해오는 것. "다운로드"랑 비슷하지만 이력까지 같이 가져온다.
- **commit**: 지금까지 고친 내용을 "이만큼 저장!"하고 기록으로 남기는 것.
- **push**: 내 컴퓨터에 쌓인 commit을 GitHub(온라인 저장소)로 올려서 남들도 볼 수 있게 하는 것.
- **branch**: 원본을 안 건드리고 따로 작업할 수 있는 "복사된 작업 공간". 나중에 준비되면 원본에 합칠 수 있다.
- **submodule**: 저장소 안에 또 다른 저장소를 끼워 넣는 git 기능. `crazyflie-firmware`가 이 방식으로 들어가 있다 — 원본(bitcraze 공식 저장소)과 계속 연결되어 있어서, 원본이 업데이트되면 받아올 수 있다. 반면 `crazyflie-basic`은 submodule이 **아니라** 그냥 이 저장소에 속한 일반 파일이다 — 원본과 연결이 끊겨 있어서 여기서 뭘 고치고 커밋해도 원본 `crazyflie-basic`에는 전혀 영향이 없다.

### Docker 용어

- **Docker**: "이 프로그램을 실행하려면 이런 프로그램들이 이런 버전으로 깔려 있어야 함"이라는 환경을 통째로 포장해서, 아무 컴퓨터에서나 똑같이 돌아가게 해주는 도구.
- **이미지(image)**: 포장된 환경의 설계도/완성본. "우분투 + ROS 2 + 필요한 프로그램들"이 미리 다 깔려있는 상태로 저장된 것.
- **컨테이너(container)**: 이미지를 실제로 실행한 것. 이미지가 "설치 파일"이라면 컨테이너는 그걸 실행해서 켜져 있는 상태.
- **Docker Compose**: 컨테이너 여러 개(이 프로젝트는 `firmware`, `basic` 2개)를 한 번에 관리해주는 도구. `docker compose ...` 명령어로 씀.

## 폴더 구조

```
crazyflie/
├── docker/                        # Docker 환경 설정 (아래 참고)
├── cf_ws/
│   └── src/crazyflie-basic/       # 일반 폴더 (원본과 분리된, 자유롭게 개발할 폴더)
└── crazyflie-firmware/            # git submodule (원본 펌웨어와 계속 연결됨)
```

## 사전 준비

먼저 아래 프로그램들이 설치되어 있어야 합니다. 터미널을 열고 아래 명령을 하나씩 쳐서 버전이 나오면 설치된 것입니다 (안 나오면 [문제 해결](#자주-발생하는-문제) 참고).

```bash
git --version
docker --version
docker compose version
```

- **git**: 대부분 리눅스에 기본 설치되어 있음. 없으면 `sudo apt install git`
- **Docker Engine + Docker Compose v2**: [공식 설치 가이드](https://docs.docker.com/engine/install/) 참고
- **Linux 호스트, X11/XWayland**: rviz2, cfclient 같은 화면(GUI) 프로그램을 띄우기 위해 필요 (Windows/Mac은 미지원)
- (선택) **GPU**를 쓸 경우 [`nvidia-container-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- Crazyradio PA 또는 Crazyflie USB 동글 (실제 드론 없이 시뮬레이션만 해볼 수도 있음)

## 실행 순서 (처음부터 순서대로)

### 0. 저장소 받기 (컴퓨터 한 대당 최초 1회만)

```bash
git clone --recursive <이 저장소 URL> crazyflie
cd crazyflie
```

`git clone`은 GitHub에 있는 이 프로젝트를 내 컴퓨터로 복사해오는 명령입니다. `--recursive` 옵션은 `crazyflie-firmware` submodule까지 같이 받아오라는 뜻입니다 (안 붙이면 그 폴더가 빈 채로 옵니다). `cf_ws/src/crazyflie-basic`은 submodule이 아니라 그냥 파일이라 별도 옵션 없이 자동으로 같이 옵니다.

이미 내 컴퓨터에 이 폴더가 있다면 이 단계는 건너뛰세요.

### 1. 내 계정 정보 설정 (최초 1회)

```bash
echo "UID=$(id -u)" > docker/.env
echo "GID=$(id -g)" >> docker/.env
```

컨테이너 안에서 파일을 만들면 기본적으로 `root`(관리자) 소유로 생성돼서, 나중에 내 계정으로 그 파일을 수정/삭제하기 곤란해질 수 있습니다. 이 명령은 "컨테이너 안에서도 내 계정처럼 동작해라"라고 미리 알려주는 설정입니다. 한 번만 하면 됩니다.

### 2. 화면(GUI) 띄우기 허용 (터미널을 새로 열거나 컴퓨터를 재시작할 때마다 1회)

```bash
xhost +local:docker
```

rviz2, cfclient처럼 창이 뜨는 프로그램을 컨테이너 안에서 실행하려면, 컨테이너가 내 화면에 그림을 그릴 수 있게 허용해줘야 합니다. 이 명령이 그 허용을 해주는 것이고, 컴퓨터를 재시작하면 다시 해줘야 합니다.

### 3. 환경 만들고 켜기

```bash
docker compose -f docker/compose.yml build
docker compose -f docker/compose.yml up -d
```

첫 번째 줄(`build`)은 필요한 프로그램들을 전부 설치한 "이미지"를 만드는 과정이라 처음 한 번은 시간이 꽤 걸립니다 (인터넷에서 여러 파일을 받아옵니다). 두 번째 줄(`up -d`)은 그 이미지로 컨테이너 2개(`firmware`, `basic`)를 백그라운드로 실행합니다. `-d`는 "터미널을 계속 붙잡지 않고 뒤에서 켜둔다"는 뜻입니다.

GPU가 있는 컴퓨터라면 (선택사항, `nvidia-container-toolkit` 설치 필요) 위 두 줄 대신 이렇게 씁니다:

```bash
docker compose -f docker/compose.yml -f docker/compose.gpu.yml build
docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d
```

> 매번 `-f docker/compose.yml`을 치기 귀찮으면 `cd docker`로 들어가서 `-f compose.yml` 부분을 생략하고 써도 됩니다. 이 문서는 저장소 루트 기준으로 통일해서 적었습니다.

### 4. `basic` 컨테이너 들어가서 시뮬레이션으로 확인해보기 (실제 드론 없이 가능)

먼저 컨테이너 "안으로 들어갑니다" (터미널이 내 컴퓨터가 아니라 컨테이너 내부를 조작하게 됨):

```bash
docker compose -f docker/compose.yml exec basic bash
```

컨테이너 안에서:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch crazyflie_test launch.py mode:=opticalflow backend:=sim
```

`colcon build`는 ROS 2 프로그램들을 컴파일하는 과정입니다. 처음 한 번은 crazyswarm2 전체를 빌드해서 시간이 좀 걸립니다 (컴퓨터 성능에 따라 몇 분~십몇 분). `backend:=sim`은 진짜 드론 없이 컴퓨터 안에서 가상으로 시뮬레이션하라는 뜻입니다.

빌드가 끝나고 실행되면, **새 터미널 창을 하나 더 열어서** 아래 명령으로 3D 시각화 화면(rviz2)을 띄워 확인할 수 있습니다:

```bash
docker compose -f docker/compose.yml exec basic bash -c "source install/setup.bash && rviz2"
```

### 5. `firmware` 컨테이너 들어가서 cfclient(드론 제어 프로그램) 확인해보기

```bash
docker compose -f docker/compose.yml exec firmware bash
pixi run cfclient
```

`pixi`는 `firmware` 컨테이너 안에서 프로그램을 설치/실행해주는 도구입니다. 최초 실행 시 필요한 걸 자동으로 한 번 더 설치하느라 시간이 좀 걸릴 수 있습니다. `cfclient`는 실제 드론과 통신하는 GUI 프로그램입니다.

### 6. 실제 드론(Crazyflie)으로 테스트할 때

1. 컴퓨터에 Crazyradio(드론과 무선으로 통신하는 동글)를 꽂습니다.
2. 컨테이너가 이미 켜진 상태에서 새로 꽂았다면, 컨테이너 안에서 아래 명령을 한 번 실행합니다 (컨테이너를 방금 새로 켰다면 자동으로 처리되니 생략 가능):
   ```bash
   fix-usb-perms
   ```
3. `firmware` 컨테이너의 cfclient에서 "Scan" 버튼으로 드론을 찾아 연결합니다. 새 펌웨어를 넣어야 하면 Bootloader 탭에서 플래시합니다.
4. `basic` 컨테이너로 실제 비행을 해보려면, `cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml` 파일을 열어 `uri`(드론 무선 주소) 값을 내 드론에 맞게 고친 뒤 `backend:=cflib`로 launch합니다 (3번 단계와 같은 방법으로 `basic` 컨테이너에 들어가서).

### 7. 다 끝났으면 정리

```bash
docker compose -f docker/compose.yml down
```

켜져 있던 컨테이너 2개를 끄고 정리합니다. 다음에 다시 쓸 때는 3번 단계(`up -d`)부터 시작하면 됩니다 (`build`는 코드/설정이 바뀌지 않았다면 다시 안 해도 됩니다).

## 자주 발생하는 문제

**`git: command not found`**
git이 안 깔려 있는 것입니다. `sudo apt install git`로 설치하세요.

**`docker: command not found`**
Docker가 안 깔려 있는 것입니다. [공식 설치 가이드](https://docs.docker.com/engine/install/)를 따라 설치하세요.

**`permission denied while trying to connect to the Docker daemon socket`**
내 계정이 Docker를 쓸 권한이 없는 것입니다. 아래 명령으로 권한을 추가하고, **로그아웃 후 다시 로그인**(또는 컴퓨터 재시작)하세요:
```bash
sudo usermod -aG docker $USER
```

**컨테이너 안에서 만든 파일이 호스트에서 `root` 소유로 보임**
[1단계](#1-내-계정-정보-설정-최초-1회)를 안 하고 `build`/`up`을 먼저 했을 가능성이 큽니다. `docker/.env` 파일을 만든 뒤 `docker compose -f docker/compose.yml build`로 다시 빌드하세요.

**rviz2/cfclient 창이 안 뜨고 에러가 남 (X11 관련)**
[2단계](#2-화면gui-띄우기-허용-터미널을-새로-열거나-컴퓨터를-재시작할-때마다-1회)의 `xhost +local:docker`를 안 했거나, 재부팅 후 다시 안 한 경우입니다.

**Crazyradio를 꽂았는데 cfclient에서 안 보임**
[6단계](#6-실제-드론crazyflie으로-테스트할-때)의 `fix-usb-perms`를 컨테이너 안에서 실행해보세요. 그래도 안 되면 `lsusb | grep -i "1915:7777\|Bitcraze"`로 컴퓨터가 라디오 자체를 인식하는지 먼저 확인하세요.

이 외 USB/GPU/X11 관련 더 자세한 내용, 서비스별 세부 설정은 [`docker/README.md`](docker/README.md)에 정리되어 있습니다.
