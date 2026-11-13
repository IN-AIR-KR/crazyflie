"""Decide what each mirrored real Crazyflie does, from the simulation's commands. No ROS here.

The simulation is the master: all robots fly in SIL and the DMPC never sees real states.
A real robot only copies its simulated twin (same takeoff, same full-state setpoints, same land).
Each real robot is guarded on its own; a guard trip lands only that robot and the simulation
continues. Times are one monotonic wall clock supplied by the caller.
"""

import math
from collections import deque

WAITING, FLYING, STOPPING, LANDING, DROPPED, EMERGENCY = (
    "WAITING", "FLYING", "STOPPING", "LANDING", "DROPPED", "EMERGENCY")
STREAMING_PHASES = ("AIRBORNE", "BRAKING_FOR_LAND")
LANDING_PHASES = ("LAND_HANDOFF", "LANDING", "LANDED", "TAKEOFF_FAILED")
# Gap between notify_setpoints_stop and land, as in the simulation executor's LAND_HANDOFF.
HANDOFF = 0.3


def _distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class MirrorSupervisor:
    def __init__(self, homes, names, settings):
        """homes: simulated start positions; names: real robots to mirror; settings: mirror + planner keys."""
        missing = set(names) - set(homes)
        if missing:
            raise ValueError(f"mirrored robots are not simulated: {sorted(missing)}")
        self.settings = settings
        self.offset = tuple(float(v) for v in settings["offset"])
        low, high = settings["bounds"]
        self.low, self.high = tuple(low), tuple(high)
        self.homes = {name: self.real(homes[name]) for name in names}
        # onboard: no mocap; the robot's own estimate is reset to its simulated start before takeoff.
        self.onboard = bool(settings.get("onboard", False))
        self.robots = {name: dict(state=WAITING, reason=None, pose=None, pose_at=None, target=None,
                                  command_at=None, streaming_since=None, land_at=None, landing_since=None,
                                  near_since=None, history=deque(), shift=(0.0, 0.0, 0.0),
                                  aligned=not self.onboard) for name in names}
        self.phase = None
        self.phase_at = None
        self.started_at = None

    def real(self, position):
        return tuple(p + o for p, o in zip(position, self.offset))

    def inside(self, position):
        return all(lo <= p <= hi for p, lo, hi in zip(position, self.low, self.high))

    def pose(self, name, position, now):
        if name in self.robots:
            robot = self.robots[name]
            robot["pose"], robot["pose_at"] = tuple(position), now
            if self.onboard:
                # No absolute position: keep the last settle_time of estimates to judge steadiness.
                history = robot["history"]
                history.append((now, robot["pose"]))
                while len(history) > 2 and now - history[1][0] >= self.settings["settle_time"]:
                    history.popleft()
            elif _distance(robot["pose"], self.homes[name]) <= self.settings["home_tolerance"]:
                if robot["near_since"] is None:
                    robot["near_since"] = now
            else:
                robot["near_since"] = None

    def aligned(self, name):
        """The onboard estimate of `name` was reset to its simulated start."""
        self.robots[name]["aligned"] = True
        self.robots[name]["near_since"] = None

    def _fresh_pose(self, robot, now):
        return robot["pose_at"] is not None and now - robot["pose_at"] <= self.settings["pose_timeout"]

    def ready(self, name, now):
        """Why a waiting robot may not take off, or None if it may."""
        robot = self.robots[name]
        if not robot["aligned"]:
            return "aligning the onboard estimate to the simulated start"
        if not self._fresh_pose(robot, now):
            return "no fresh real pose"
        error = _distance(robot["pose"], self.homes[name])
        if not self.onboard and error > self.settings["home_tolerance"]:
            return f"real pose is {error:.2f} m from its simulated start (+offset)"
        if self.onboard:
            # Steady = moved at most home_tolerance over the last settle_time. A slow flow-deck
            # creep on the ground passes; a diverging estimate (no positioning deck) never does.
            history = robot["history"]
            span = now - history[0][0] if history else 0.0
            if span < self.settings["settle_time"]:
                return f"waiting for a steady pose ({span:.1f}/{self.settings['settle_time']:.1f} s)"
            moved = max(_distance(p, robot["pose"]) for _, p in history)
            if moved > self.settings["home_tolerance"]:
                return f"onboard estimate is not steady ({moved:.2f} m in {span:.1f} s)"
            return None
        # A diverging estimate can pass the distance check for an instant (e.g. right after a reset).
        steady = now - robot["near_since"] if robot["near_since"] is not None else 0.0
        if steady < self.settings["settle_time"]:
            return f"waiting for a steady pose ({steady:.1f}/{self.settings['settle_time']:.1f} s)"
        return None

    def _land(self, name, now, reason, actions):
        robot = self.robots[name]
        robot["reason"] = reason
        if robot["streaming_since"] is not None:
            # Hand the firmware back to the high-level commander before landing.
            actions.append(("stop", name))
            robot["state"], robot["land_at"] = STOPPING, now + HANDOFF
        else:
            actions.append(("land", name))
            robot["state"], robot["landing_since"] = LANDING, now

    def over(self, now):
        """True once the simulation is gone and every robot that flew has finished landing.

        The real side outlives the simulation on purpose: whatever stops the simulation (Ctrl+C,
        a closed window, a crash), the guards in check() land the robots first; then the mirror
        stops their motors and exits. A simulation that never appears ends the session too.
        """
        if self.started_at is None:
            self.started_at = now
        if self.phase_at is not None:
            gone = now - self.phase_at > self.settings["sim_timeout"]
        else:
            gone = now - self.started_at > self.settings.get("sim_start_timeout", 60.0)
        if not gone:
            return False
        for robot in self.robots.values():
            if robot["state"] in (FLYING, STOPPING):
                return False
            if robot["state"] == LANDING and now - robot["landing_since"] < self.settings["land_duration"] + 1.0:
                return False
        return True

    def flown(self):
        """Robots that took off in this session (their motors are stopped at the end)."""
        return [name for name, robot in self.robots.items() if robot["landing_since"] is not None]

    def sim_phase(self, phase, now):
        """React to the simulated executor phase (from /fleet/states). Returns actions."""
        actions = []
        previous, self.phase, self.phase_at = self.phase, phase, now
        if phase == "EMERGENCY":
            if any(r["state"] != EMERGENCY for r in self.robots.values()):
                actions.append(("emergency",))
                for robot in self.robots.values():
                    robot["state"], robot["reason"] = EMERGENCY, "simulation emergency"
            return actions
        if phase == "TAKEOFF" and previous != "TAKEOFF":
            for name, robot in self.robots.items():
                if robot["state"] != WAITING:
                    continue
                reason = self.ready(name, now)
                if reason:
                    robot["state"], robot["reason"] = DROPPED, reason
                else:
                    robot["state"] = FLYING
                    if self.onboard:
                        # Where the robot believes it is now becomes its simulated start: the
                        # ground creep of the estimate is carried into every forwarded setpoint.
                        home, pose = self.homes[name], robot["pose"]
                        robot["shift"] = (pose[0] - home[0], pose[1] - home[1], 0.0)
                    actions.append(("takeoff", name))
        if phase in LANDING_PHASES:
            for name, robot in self.robots.items():
                if robot["state"] == FLYING:
                    self._land(name, now, f"simulation {phase}", actions)
        return actions

    def command(self, name, position, now):
        """A simulated full-state setpoint for `name`. Returns the real target position, or None to drop it."""
        robot = self.robots.get(name)
        if robot is None or robot["state"] != FLYING or self.phase not in STREAMING_PHASES:
            return None, []
        target = tuple(t + d for t, d in zip(self.real(position), robot["shift"]))
        if not self.inside(target):
            actions = []
            self._land(name, now, f"setpoint {tuple(round(v, 2) for v in target)} outside real bounds", actions)
            return None, actions
        if robot["streaming_since"] is None:
            robot["streaming_since"] = now
        robot["target"], robot["command_at"] = target, now
        return target, []

    def check(self, now):
        """Periodic guards. Returns actions."""
        actions = []
        s = self.settings
        for name, robot in self.robots.items():
            if robot["state"] == STOPPING and now >= robot["land_at"]:
                actions.append(("land", name))
                robot["state"], robot["landing_since"] = LANDING, now
            if robot["state"] != FLYING:
                continue
            if self.phase_at is None or now - self.phase_at > s["sim_timeout"]:
                self._land(name, now, "simulation states lost", actions)
            elif not self._fresh_pose(robot, now):
                self._land(name, now, "real pose lost", actions)
            elif robot["streaming_since"] is not None and now - robot["command_at"] > s["command_timeout"]:
                self._land(name, now, "simulated setpoint stream stopped", actions)
            elif (robot["streaming_since"] is not None and now - robot["streaming_since"] >= s["tracking_grace"]
                  and _distance(robot["pose"], robot["target"]) > s["max_tracking_error"]):
                error = _distance(robot["pose"], robot["target"])
                self._land(name, now, f"tracking error {error:.2f} m", actions)
        return actions

    def snapshot(self, now):
        robots = {}
        for name, robot in self.robots.items():
            robots[name] = dict(state=robot["state"], reason=robot["reason"], shift=robot["shift"],
                                blocked=self.ready(name, now) if robot["state"] == WAITING else None,
                                pose=robot["pose"], target=robot["target"])
        return dict(version=1, sim_phase=self.phase, mode=self.settings.get("mode", "onboard" if self.onboard else "mocap"), robots=robots)
