# 포함된 외부 소스

`stage0-mpc`에서 `crazyflie-basic`과 내부 서브모듈을 상위 `crazyflie` 저장소의 일반 파일로 통합했다. 아래는 통합 전 고정된 커밋이며, `main`의 기능 변경은 가져오지 않았다.

이 폴더의 변경은 상위 `crazyflie` 저장소에 커밋한다. 원본 Git 이력은 합치지 않았으며, 라이선스 파일은 그대로 포함했다. 서브모듈 설정 파일(`.gitmodules`)은 제거했다. 경로는 이 문서가 있는 폴더 기준이다.

| 경로 | 원본 저장소 | 기준 커밋 |
| --- | --- | --- |
| `.` | https://github.com/IN-AIR-KR/crazyflie-basic.git | `c905e03767bf2520884bc3c3f13fa5a851e88d1a` |
| `crazyswarm2` | https://github.com/IMRCLab/crazyswarm2.git | `fcf51e27fe675bbdb564a40f4ca1ee4336c062ea` |
| `crazyswarm2/crazyflie_server_cpp/deps/crazyflie_tools` | https://github.com/whoenig/crazyflie_tools.git | `91cd6304e374be554ce463db4ba57f816e7c59f3` |
| `crazyswarm2/crazyflie_server_cpp/deps/crazyflie_tools/crazyflie_cpp` | https://github.com/whoenig/crazyflie_cpp.git | `72996dc8080af4efcace3d65810a0a42ccac9541` |
| `crazyswarm2/crazyflie_server_cpp/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp` | https://github.com/bitcraze/crazyflie-link-cpp.git | `fc2881c464e812c6720cb10cc77914a5689f0d67` |
| `crazyswarm2/crazyflie_server_cpp/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/libusb` | https://github.com/libusb/libusb.git | `683e3cf21ed37d4492c20cea8de810c4d95ae8b6` |
| `crazyswarm2/crazyflie_server_cpp/deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/pybind11` | https://github.com/pybind/pybind11.git | `5b0a6fc2017fcc176545afe3e09c9f9885283242` |
| `motion_capture_tracking` | https://github.com/IMRCLab/motion_capture_tracking.git | `64d3af2456e534cd5e587b98ef2f15dd8ad35e8c` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture` | https://github.com/IMRCLab/libmotioncapture.git | `24321e4c1c923a1bc5d6cecdaa7834f11b79d8f2` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/NatNetSDKCrossplatform` | https://github.com/whoenig/NatNetSDKCrossplatform.git | `0a9b1e532d5d168e3b3569e97670c73181b2577c` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/pybind11` | https://github.com/pybind/pybind11.git | `ee2b5226295d67b690faddd446a329bb2840a1a8` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/qualisys_cpp_sdk` | https://github.com/IMRCLab/qualisys_cpp_sdk.git | `45c9b50101dc7422b921f60cd68f3fb5b60de7cb` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk` | https://github.com/whoenig/vicon-datastream-sdk.git | `a5096f283f484acca98b434c08810cd922551701` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vrpn` | https://github.com/IMRCLab/vrpn.git | `066812ba7f637c0f6e3672a4550620e5a20ca688` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vrpn/submodules/hidapi` | https://github.com/vrpn/libusb-hidapi.git | `8b26e4962175a82205ebc499fdc2682e581e5fbe` |
| `motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vrpn/submodules/jsoncpp` | https://github.com/vrpn/jsoncpp.git | `00b0a1b99216a3d220a25a78a496e7d0b0c4251c` |
| `motion_capture_tracking/motion_capture_tracking/deps/librigidbodytracker` | https://github.com/IMRCLab/librigidbodytracker.git | `020e541b6825595c841f1103bf86e42914d08c9f` |

`jsoncpp/.gitattributes`를 추가해, 기존 서브모듈 경계를 넘어 VRPN의 줄바꿈 규칙이 적용되지 않게 했다. 원본 파일의 Git blob과 실행 권한은 그대로 보존한다.

`crazyflie-firmware`는 기존 서브모듈로 유지한다.
