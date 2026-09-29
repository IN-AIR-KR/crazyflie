"""Stage 1 SIL 노드 — Crazyswarm2 sim 위에서 군집 TinyMPC 를 폐루프로 돌린다.

    # 터미널 1: 서버 (sim 4기 + RViz2 + Gazebo)
    ros2 launch /workspace/stage1/sil/swarm_sim.launch.py
    # 터미널 2: 이 노드
    python3 /workspace/stage1/sil/swarm_mpc_node.py --controller dist --obstacles mixed

Python 시뮬레이션(main.py)과 다른 점 — 이게 SIL 에서 확인하려는 것들이다.
  * 플랜트가 이중적분기가 아니라 Crazyswarm2 sim 의 쿼드로터 동역학 + 펌웨어
    Mellinger 제어기(cffirmware SIL)다. MPC 는 여전히 이중적분기로 예측한다(모델 오차).
  * 상태는 TF(world→cfX) 위치에서 얻고, 속도는 위치 차분으로 추정한다(추정 오차).
  * 제어 주기는 sim 시계(/clock) 기준 20 Hz 를 목표로 하지만, sim 이 "가능한 한 빨리"
    돌기 때문에 이 노드의 계산 시간만큼 실제 주기가 흔들린다(지연). 실제 주기는 로그에 남긴다.
  * 명령은 가속도가 아니라 cmdFullState(다음 스텝 예측 위치·속도 + 가속도 피드포워드).
장애물은 가상이다: 시나리오의 참 궤적을 인식값으로 쓰고, RViz 마커와 Gazebo 모델로 보여준다.

로그(npz)는 sil_eval.py 가 main.py 와 같은 지표로 평가한다.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from controller_tiny import TinyCentralMPC, TinyConfig, TinyDistributedMPC  # noqa: E402
from scenario_sweep import SweepConfig, SweepScenario  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="Stage 1 SIL — Crazyswarm2 sim 위 군집 TinyMPC")
    p.add_argument("--controller", choices=["dist", "central"], default="dist")
    p.add_argument("--obstacles", default="mixed",
                   choices=["none", "crossing", "headon", "static", "mixed"])
    p.add_argument("--height", type=float, default=0.5, help="비행 고도 [m]")
    p.add_argument("--rate", type=float, default=20.0, help="제어 주기 [Hz] (sim 시계)")
    p.add_argument("--vel-alpha", type=float, default=0.5,
                   help="속도 추정 저역통과 계수 (1=차분 그대로)")
    p.add_argument("--log", default=os.path.join(HERE, "sil_log.npz"))
    args, _ = p.parse_known_args()      # --ros-args 등은 rclpy 가 가져간다
    return args


def main() -> None:
    args = parse_args()
    import rclpy
    from rclpy.parameter import Parameter
    from crazyflie_py import Crazyswarm
    from geometry_msgs.msg import Point, Pose
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    # Crazyswarm() 는 서버 서비스(all/takeoff, /crazyflie_server/set_parameters …)가
    # 나타날 때까지 아무 출력 없이 기다린다. 서버가 죽어 있으면 여기서 영원히 멈춘다.
    print("[swarm_mpc] crazyflie_server 서비스 대기 중… (오래 걸리면 서버 터미널에서 "
          "'crazyflie_server ... process has died' 를 확인)", flush=True)
    swarm = Crazyswarm()
    print("[swarm_mpc] 서버 연결됨", flush=True)
    node = swarm.allcfs
    # sim 은 /clock 을 발행한다. 제어 주기·타이밍을 모두 sim 시계로 잰다.
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    th = swarm.timeHelper

    names = sorted(node.crazyfliesByName)            # 이름 순 = 편대 슬롯 순
    cfs = [node.crazyfliesByName[n] for n in names]
    sc = SweepScenario(SweepConfig(n_agents=len(cfs), obstacles=args.obstacles))
    cfg = TinyConfig(dt=1.0 / args.rate)
    ctrl = (TinyDistributedMPC if args.controller == "dist" else TinyCentralMPC)(sc, cfg)
    node.get_logger().info(f"기체 {names} / 제어기 {ctrl.name} / 장애물 {args.obstacles} "
                           f"/ 임무 {sc.duration:.1f}s")

    tf_buf = Buffer()
    TransformListener(tf_buf, node)
    marker_pub = node.create_publisher(MarkerArray, "swarm_mpc/markers", 10)
    gz = node.create_client(SetEntityPose, "/world/empty/set_pose")

    def read_positions() -> tuple[np.ndarray, float] | None:
        pos = np.zeros((len(names), 3))
        stamp = 0.0
        for i, n in enumerate(names):
            try:
                tr = tf_buf.lookup_transform("world", n, rclpy.time.Time())
            except TransformException:
                return None
            t = tr.transform.translation
            pos[i] = (t.x, t.y, t.z)
            stamp = max(stamp, tr.header.stamp.sec + 1e-9 * tr.header.stamp.nanosec)
        return pos, stamp

    # ---------------------------------------------------------------- 대기 / 이륙
    node.get_logger().info("TF(world→cfX) 대기 중…")
    while rclpy.ok() and read_positions() is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info("TF 수신 — 이륙")
    node.takeoff(targetHeight=args.height, duration=2.5)
    th.sleep(3.5)

    # ---------------------------------------------------------------- 폐루프
    steps = int(round(sc.duration * args.rate))
    m = len(names)
    k_obs = len(sc.obstacle_list)
    L = {key: [] for key in ("t", "states", "refs", "inputs", "obs_pos", "solve_ms",
                             "iterations", "urgent", "failed", "ctrl_dt", "wall_ms", "z")}
    prev_p, prev_t = None, None
    vel = np.zeros((m, 2))
    t0 = th.time()
    for n in range(steps):
        th.sleepForRate(args.rate)
        if not rclpy.ok():
            break
        got = read_positions()
        if got is None:
            continue
        pos, _ = got
        t_now = th.time()
        t = t_now - t0
        if prev_p is not None and t_now > prev_t:
            raw = (pos[:, :2] - prev_p) / (t_now - prev_t)
            vel = args.vel_alpha * raw + (1.0 - args.vel_alpha) * vel
        ctrl_dt = float("nan") if prev_t is None else t_now - prev_t
        prev_p, prev_t = pos[:, :2].copy(), t_now
        state = np.hstack([pos[:, :2], vel])

        w0 = time.perf_counter()
        u = ctrl.step(state, t)
        wall = (time.perf_counter() - w0) * 1e3
        nxt = ctrl.reference_setpoint
        if nxt is None:
            nxt = state
        for i, cf in enumerate(cfs):
            cf.cmdFullState([nxt[i, 0], nxt[i, 1], args.height], [nxt[i, 2], nxt[i, 3], 0.0],
                            [u[i, 0], u[i, 1], 0.0], 0.0, [0.0, 0.0, 0.0])

        # 가상 장애물: 로그 + RViz + Gazebo
        obs = np.full((k_obs, 2), np.nan)
        ma = MarkerArray()
        for k, o in enumerate(sc.obstacle_list):
            p = o.position(t)
            mk = Marker()
            mk.header.frame_id = "world"
            mk.header.stamp = node.get_clock().now().to_msg()
            mk.ns, mk.id, mk.type = "obstacle", k, Marker.CYLINDER
            if p is None:
                mk.action = Marker.DELETE
            else:
                obs[k] = p
                mk.action = Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(p[0]), float(p[1]), args.height
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = 2 * o.radius
                mk.scale.z = 2 * args.height
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.9, 0.3, 0.2, 0.8
                if gz.service_is_ready():
                    req = SetEntityPose.Request()
                    req.entity = Entity(name=f"obs{k}", type=Entity.MODEL)
                    req.pose = Pose()
                    req.pose.position.x, req.pose.position.y, req.pose.position.z = float(p[0]), float(p[1]), args.height
                    req.pose.orientation.w = 1.0
                    gz.call_async(req)
            ma.markers.append(mk)
        if ctrl.plan is not None:                       # 예측 궤적
            for i in range(m):
                mk = Marker()
                mk.header.frame_id = "world"
                mk.ns, mk.id, mk.type, mk.action = "plan", i, Marker.LINE_STRIP, Marker.ADD
                mk.pose.orientation.w = 1.0
                mk.scale.x = 0.015
                mk.color.g, mk.color.b, mk.color.a = 0.8, 1.0, 0.9
                mk.points = [Point(x=float(q[0]), y=float(q[1]), z=args.height) for q in ctrl.plan[i]]
                ma.markers.append(mk)
        marker_pub.publish(ma)

        L["t"].append(t)
        L["states"].append(state)
        L["refs"].append(sc.reference(t))
        L["inputs"].append(u)
        L["obs_pos"].append(obs)
        L["solve_ms"].append(ctrl.solve_ms)
        L["iterations"].append(ctrl.iterations)
        L["urgent"].append(ctrl.slack)
        L["failed"].append(ctrl.failed)
        L["ctrl_dt"].append(ctrl_dt)
        L["wall_ms"].append(wall)
        L["z"].append(pos[:, 2])

    # ---------------------------------------------------------------- 착륙 / 저장
    for cf in cfs:
        cf.notifySetpointsStop()
    node.land(targetHeight=0.04, duration=2.5)
    th.sleep(3.0)
    np.savez(args.log, **{k: np.asarray(v) for k, v in L.items()},
             obs_rad=np.array([o.radius for o in sc.obstacle_list]),
             controller=ctrl.name, obstacles=args.obstacles, rate=args.rate,
             iter_hist=np.concatenate(ctrl.iter_hist) if ctrl.iter_hist else np.zeros(0),
             filtered=ctrl.filtered, unsolved=ctrl.unsolved, resets=ctrl.resets)
    node.get_logger().info(f"로그 저장: {args.log}")


if __name__ == "__main__":
    main()
