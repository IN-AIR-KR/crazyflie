"""ROS adapter: copy simulated robots' commands onto real Crazyflies served under /<namespace>."""

import argparse
import json
import re
import sys
import time

from .functions import MirrorSupervisor


def options(args=None):
    from rclpy.utilities import remove_ros_args
    parser = argparse.ArgumentParser()
    parser.add_argument("--mirror", required=True, help="mirror.yaml")
    parser.add_argument("--config", required=True, help="planner.yaml of the simulation")
    parser.add_argument("--robots", required=True, help="robots.yaml of the simulation")
    parser.add_argument("--real", required=True, help="comma-separated robots to mirror")
    parser.add_argument("--namespace", default="real")
    parser.add_argument("--onboard", action="store_true",
                        help="no mocap: robots fly on their onboard estimate, initialised at their simulated start")
    return parser.parse_args(remove_ros_args(sys.argv)[1:] if args is None else args)


def mirror_settings(mirror_path, planner):
    from ..common.trajectory import read_mapping
    data = read_mapping(mirror_path)
    keys = ("offset", "bounds", "home_tolerance", "settle_time", "max_tracking_error", "tracking_grace",
            "pose_timeout", "command_timeout", "sim_timeout")
    missing = [key for key in keys if key not in data]
    if missing:
        raise ValueError(f"mirror.yaml에 {missing} 항목이 필요합니다.")
    settings = {key: data[key] for key in keys}
    for key in ("takeoff_height", "takeoff_duration", "land_height", "land_duration"):
        settings[key] = planner[key]
    return settings


class MirrorNode:
    def __init__(self, node, supervisor, namespace):
        from crazyflie_interfaces.msg import FullState
        from crazyflie_interfaces.srv import Land, NotifySetpointsStop, Takeoff
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        from std_srvs.srv import Empty

        self.node, self.supervisor = node, supervisor
        self.settings = supervisor.settings
        self._srv = dict(Takeoff=Takeoff, Land=Land, NotifySetpointsStop=NotifySetpointsStop, Empty=Empty)
        self._String = String
        ns = "/" + namespace.strip("/")
        self.publishers, self.clients, self.subscriptions = {}, {}, []
        for name in supervisor.robots:
            self.publishers[name] = node.create_publisher(FullState, f"{ns}/{name}/cmd_full_state", 1)
            self.clients[name] = dict(
                takeoff=node.create_client(Takeoff, f"{ns}/{name}/takeoff"),
                land=node.create_client(Land, f"{ns}/{name}/land"),
                stop=node.create_client(NotifySetpointsStop, f"{ns}/{name}/notify_setpoints_stop"))
            self.subscriptions.append(node.create_subscription(
                FullState, f"/{name}/cmd_full_state", lambda msg, robot=name: self._command(robot, msg), 1))
            self.subscriptions.append(node.create_subscription(
                PoseStamped, f"{ns}/{name}/pose", lambda msg, robot=name: self._pose(robot, msg), qos_profile_sensor_data))
        self.emergency_client = node.create_client(Empty, f"{ns}/all/emergency")
        if supervisor.onboard:
            # The estimate is initialised at the simulated start by the real server on connect
            # (firmware_params from mirror.launch.py); the readiness check confirms it from the pose.
            for name in supervisor.robots:
                supervisor.aligned(name)
            homes = ", ".join(f"{n} {tuple(round(v, 2) for v in supervisor.homes[n])}" for n in supervisor.robots)
            node.get_logger().info(f"no mocap: onboard estimate starts at the simulated start ({homes}); place each robot there facing +x")
        self.subscriptions.append(node.create_subscription(String, "/fleet/states", self._states, qos_profile_sensor_data))
        self.status_pub = node.create_publisher(String, "/mirror/status", 10)
        self.timer = node.create_timer(0.02, self._check)
        self.status_timer = node.create_timer(0.5, self._status)
        self.reported = {}
        self.closing_at = None
        node.get_logger().info(f"mirroring {', '.join(supervisor.robots)} onto {ns}/*")

    @staticmethod
    def now():
        return time.monotonic()

    def _pose(self, name, message):
        p = message.pose.position
        self.supervisor.pose(name, (p.x, p.y, p.z), self.now())

    def _states(self, message):
        try:
            phase = json.loads(message.data)["executor"]["phase"]
        except (ValueError, KeyError, TypeError):
            return
        self._run(self.supervisor.sim_phase(phase, self.now()))

    def _command(self, name, message):
        p = message.pose.position
        target, actions = self.supervisor.command(name, (p.x, p.y, p.z), self.now())
        self._run(actions)
        if target is None:
            return
        message.pose.position.x, message.pose.position.y, message.pose.position.z = target
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.publishers[name].publish(message)

    def _check(self):
        now = self.now()
        self._run(self.supervisor.check(now))
        if self.closing_at is None and self.supervisor.over(now):
            log = self.node.get_logger()
            flown = self.supervisor.flown()
            if flown:
                # Landed already (check() landed them when the simulation went away); this makes
                # sure no motor keeps spinning before the real server is closed.
                log.warning(f"simulation ended: {', '.join(flown)} landed, stopping motors")
                self._call(self.emergency_client, self._srv["Empty"].Request(), "motor stop")
            else:
                log.info("simulation ended: no real robot flew")
            self.closing_at = now
        elif self.closing_at is not None and now - self.closing_at > 0.5:
            import rclpy
            rclpy.try_shutdown()  # the real launch treats this node as required and stops the server

    @staticmethod
    def _duration(seconds):
        from builtin_interfaces.msg import Duration
        return Duration(sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))

    def _call(self, client, request, label):
        if not client.service_is_ready():
            self.node.get_logger().error(f"{label}: real server service is not available")
            return
        client.call_async(request)

    def _run(self, actions):
        log = self.node.get_logger()
        s = self.settings
        for action in actions:
            kind = action[0]
            if kind == "emergency":
                log.error("EMERGENCY mirrored to every real Crazyflie")
                self._call(self.emergency_client, self._srv["Empty"].Request(), "emergency")
                continue
            name = action[1]
            robot = self.supervisor.robots[name]
            if kind == "takeoff":
                request = self._srv["Takeoff"].Request(group_mask=0, height=float(s["takeoff_height"] + self.supervisor.offset[2]),
                                                       duration=self._duration(s["takeoff_duration"]))
                shift = robot["shift"]
                extra = f" (onboard estimate {abs(shift[0]):.2f}/{abs(shift[1]):.2f} m off its start: path shifted to where it sits)" if any(shift) else ""
                log.info(f"{name}: takeoff with its simulated twin{extra}")
                self._call(self.clients[name]["takeoff"], request, f"{name} takeoff")
            elif kind == "stop":
                log.warning(f"{name}: landing ({robot['reason']})")
                self._call(self.clients[name]["stop"], self._srv["NotifySetpointsStop"].Request(group_mask=0, remain_valid_millisecs=100), f"{name} stop")
            elif kind == "land":
                if robot["streaming_since"] is None:
                    log.warning(f"{name}: landing ({robot['reason']})")
                request = self._srv["Land"].Request(group_mask=0, height=float(s["land_height"] + self.supervisor.offset[2]),
                                                    duration=self._duration(s["land_duration"]))
                self._call(self.clients[name]["land"], request, f"{name} land")

    def _status(self):
        now = self.now()
        data = self.supervisor.snapshot(now)
        self.status_pub.publish(self._String(data=json.dumps(data, allow_nan=False)))
        log = self.node.get_logger()
        for name, robot in data["robots"].items():
            line = (robot["state"], robot["reason"] or robot["blocked"])
            # Compare without numbers so a changing distance or countdown is logged once.
            key = (line[0], re.sub(r"[0-9.]+", "#", line[1] or ""))
            if self.reported.get(name) != key:
                self.reported[name] = key
                if robot["state"] == "DROPPED":
                    log.error(f"{name}: not flown ({robot['reason']}); the simulation continues")
                elif robot["state"] == "WAITING":
                    log.info(f"{name}: " + ("ready" if not robot["blocked"] else f"not ready - {robot['blocked']}"))


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from ..common.trajectory import load_dmpc_settings, load_robot_positions

    opt = options(args)
    planner = load_dmpc_settings(opt.config)
    homes = load_robot_positions(opt.robots)
    names = [name for name in opt.real.split(",") if name]
    settings = mirror_settings(opt.mirror, planner)
    settings["onboard"] = opt.onboard
    supervisor = MirrorSupervisor(homes, names, settings)
    rclpy.init(args=args)
    node = rclpy.create_node("mirror")
    try:
        MirrorNode(node, supervisor, opt.namespace)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
