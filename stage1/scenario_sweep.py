"""시나리오 — 이동 편대 스윕 커버리지 + 동적/정적 장애물 (Crazyflie 실내 규모).

편대: 진행 방향(y)에 수직인 횡대(line abreast). 기체 간격 = 센서 폭 − 겹침.
      편대가 지나간 띠(swath) 폭 = M·간격 이 한 번에 덮이는 폭이다.
경로: 영역을 y 방향 레인으로 나눈 부스트로페돈(lawnmower). 레인 끝에서 x 로
      한 swath 만큼 옆걸음한다. 횡대 방향과 옆걸음 방향이 같아서 편대를
      회전시킬 필요가 없다 — 참조 c_i(t) 가 단순한 평행이동이 된다.
속도: 구간마다 코사인 램프로 0→v→0. 모서리에서 속도가 0 이라 참조가 C¹ 이고
      Qi 식 (4)의 피드포워드 v̇_ci 도 해석적으로 정의된다.

제어기가 이 객체에서 보는 것은 reference_horizon(), obstacles(), side_hint() 뿐이다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Phase:
    name: str
    t_start: float
    t_end: float


@dataclass
class Obstacle:
    """등속 장애물. v=0 이면 정적. t_appear 이전에는 존재하지 않는다."""
    p0: np.ndarray
    v: np.ndarray
    radius: float
    t_appear: float = 0.0
    label: str = ""

    def position(self, t: float) -> np.ndarray | None:
        if t < self.t_appear:
            return None
        return self.p0 + self.v * (t - self.t_appear)


@dataclass
class SweepConfig:
    n_agents: int = 4
    width: float = 4.8            # 영역 x 크기 [m]
    height: float = 4.0           # 영역 y 크기 [m]
    sensor_radius: float = 0.35   # 하향 센서 footprint 반지름 [m]
    spacing: float = 0.6          # 기체 간격 [m] (< 2·sensor_radius 면 띠가 겹친다)
    speed: float = 0.4            # 스윕 속도 [m/s]
    ramp: float = 1.5             # 구간 가감속 시간 [s]
    lead_in: float = 0.8          # 영역 밖에서 시작/끝나는 여유 [m]
    t_form: float = 6.0           # 시작점에서 편대를 만드는 시간 [s]
    t_tail: float = 4.0           # 끝난 뒤 정지 유지 시간 [s]
    seed: int = 0
    obstacles: str = "mixed"      # none | crossing | headon | static | mixed
    obs_speed: float = 0.4
    obs_radius: float = 0.2


class SweepScenario:
    def __init__(self, cfg: SweepConfig) -> None:
        self.cfg = cfg
        self.n_agents = int(cfg.n_agents)
        m = self.n_agents
        self._offsets = np.stack([(np.arange(m) - 0.5 * (m - 1)) * cfg.spacing,
                                  np.zeros(m)], axis=1)
        self.swath = m * cfg.spacing
        self.n_lanes = int(np.ceil(cfg.width / self.swath - 1e-9))
        self.region = (0.0, cfg.width, 0.0, cfg.height)

        # 편대 중심 웨이포인트 (부스트로페돈)
        y_lo, y_hi = -cfg.lead_in, cfg.height + cfg.lead_in
        pts = []
        for lane in range(self.n_lanes):
            x = self.swath * (lane + 0.5)
            ys = (y_lo, y_hi) if lane % 2 == 0 else (y_hi, y_lo)
            pts += [np.array([x, ys[0]]), np.array([x, ys[1]])]
        self._wp = np.array(pts)
        # 구간별 (시작시각, 길이, 지속시간)
        self._seg_t0 = []
        t = cfg.t_form
        for a, b in zip(self._wp[:-1], self._wp[1:]):
            length = float(np.linalg.norm(b - a))
            dur = self._segment_duration(length)
            self._seg_t0.append((t, length, dur))
            t += dur
        self.t_sweep_end = t
        self.duration = t + cfg.t_tail
        self.obstacle_list = self._make_obstacles()

    # ------------------------------------------------------------ 속도 프로파일
    def _segment_duration(self, length: float) -> float:
        v, tr = self.cfg.speed, self.cfg.ramp
        d_ramp = v * tr               # 가속 + 감속 구간 거리 합 (각각 v·tr/2)
        if length <= d_ramp:          # 짧은 구간: 램프만으로 끝낸다
            return 2.0 * np.sqrt(length * tr / v)
        return tr + length / v        # 가속 tr + 등속 (L - v·tr)/v + 감속 tr

    def _profile(self, tau: float, length: float, dur: float) -> tuple[float, float, float]:
        """구간 안에서의 (거리, 속력, 가속도). 코사인 램프 가속-등속-감속."""
        v, tr = self.cfg.speed, self.cfg.ramp
        if length <= v * tr:          # 짧은 구간은 속도 상한을 낮춰 같은 모양으로
            tr = 0.5 * dur
            v = length / tr
        tau = min(max(tau, 0.0), dur)
        cruise_end = dur - tr

        def up(x: float) -> tuple[float, float, float]:
            # 0→v 코사인 램프, x∈[0, tr]
            s = v * 0.5 * (x - tr / np.pi * np.sin(np.pi * x / tr))
            sp = v * 0.5 * (1.0 - np.cos(np.pi * x / tr))
            ac = v * 0.5 * np.pi / tr * np.sin(np.pi * x / tr)
            return s, sp, ac

        if tau < tr:
            return up(tau)
        if tau <= cruise_end:
            s_up, _, _ = up(tr)
            return s_up + v * (tau - tr), v, 0.0
        s_dn, sp_dn, ac_dn = up(dur - tau)     # 감속은 가속을 시간 반전한 것
        return length - s_dn, sp_dn, -ac_dn

    def center(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """편대 중심의 (위치, 속도, 가속도)."""
        if t <= self.cfg.t_form:
            return self._wp[0].copy(), np.zeros(2), np.zeros(2)
        for k, (t0, length, dur) in enumerate(self._seg_t0):
            if t <= t0 + dur:
                a, b = self._wp[k], self._wp[k + 1]
                d = (b - a) / length
                s, sp, ac = self._profile(t - t0, length, dur)
                return a + d * s, d * sp, d * ac
        return self._wp[-1].copy(), np.zeros(2), np.zeros(2)

    # ------------------------------------------------------------ 참조 (Qi 와 공용)
    def reference(self, t: float) -> np.ndarray:
        cp, cv, _ = self.center(t)
        ref = np.zeros((self.n_agents, 4))
        ref[:, :2] = self._offsets + cp
        ref[:, 2:] = cv
        return ref

    def reference_accel(self, t: float) -> np.ndarray:
        return np.tile(self.center(t)[2], (self.n_agents, 1))

    def reference_horizon(self, t0: float, dt: float, knots: int) -> np.ndarray:
        return np.stack([self.reference(t0 + k * dt) for k in range(knots)])

    def side_hint(self, i: int) -> np.ndarray:
        """정면 충돌 때 비킬 쪽 — 편대 안에서 자기가 있는 쪽. 가운데면 +x."""
        off = self._offsets[i] - self._offsets.mean(axis=0)
        return off if np.linalg.norm(off) > 1e-9 else np.array([1.0, 0.0])

    # ------------------------------------------------------------ 장애물
    def _aim(self, agent: int, t_hit: float, v: np.ndarray, t_appear: float) -> np.ndarray:
        """t_hit 에 기체 agent 의 참조 위치를 정확히 지나가도록 출발점을 잡는다."""
        target = self.reference(t_hit)[agent, :2]
        return target - v * (t_hit - t_appear)

    def _make_obstacles(self) -> list[Obstacle]:
        c = self.cfg
        kind = c.obstacles
        out: list[Obstacle] = []
        lane0 = self._seg_t0[0]
        lane2 = self._seg_t0[2] if len(self._seg_t0) > 2 else lane0
        if kind in ("crossing", "mixed"):
            # 레인 1 중간에 +x 로 편대를 가로지름 — 안쪽 기체(1) 를 정조준
            t_hit = lane0[0] + 0.5 * lane0[2]
            v = np.array([c.obs_speed, 0.0])
            out.append(Obstacle(self._aim(1, t_hit, v, t_hit - 6.0), v, c.obs_radius,
                                t_hit - 6.0, "crossing"))
        if kind in ("headon", "mixed"):
            # 레인 2 에서 편대와 정면으로 마주 오는 장애물 — 기체 2 를 정조준
            t_hit = lane2[0] + 0.45 * lane2[2]
            v = np.array([0.0, c.obs_speed])
            out.append(Obstacle(self._aim(2, t_hit, v, t_hit - 6.0), v, c.obs_radius,
                                t_hit - 6.0, "head-on"))
        if kind in ("static", "mixed"):
            # 레인 1 의 기체 3 궤적 위에 놓인 기둥
            t_hit = lane0[0] + 0.8 * lane0[2]
            out.append(Obstacle(self._aim(3, t_hit, np.zeros(2), 0.0), np.zeros(2),
                                c.obs_radius, 0.0, "static"))
        return out

    def obstacles(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """제어기가 인식하는 장애물 (위치, 속도, 반지름). 현재는 참값 = 인식값."""
        ps, vs, rs = [], [], []
        for o in self.obstacle_list:
            p = o.position(t)
            if p is not None:
                ps.append(p)
                vs.append(o.v)
                rs.append(o.radius)
        if not ps:
            return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0)
        return np.array(ps), np.array(vs), np.array(rs)

    # ------------------------------------------------------------ 초기 조건/단계
    def initial_states(self) -> np.ndarray:
        """시작점 근처 이륙 위치. 편대 간격보다 촘촘하지 않게 줄지어 흩어 둔다."""
        rng = np.random.default_rng(self.cfg.seed)
        start = self._wp[0] + np.array([0.0, -0.8])
        m = self.n_agents
        xs = (np.arange(m) - 0.5 * (m - 1)) * 0.5
        rng.shuffle(xs)                       # 이륙 순서를 섞어 편대 형성 중 교차를 만든다
        st = np.zeros((m, 4))
        st[:, 0] = start[0] + xs
        st[:, 1] = start[1] + rng.uniform(-0.2, 0.2, m)
        return st

    def phases(self) -> list[Phase]:
        out = [Phase("Formation", 0.0, self.cfg.t_form)]
        for k, (t0, _, dur) in enumerate(self._seg_t0):
            out.append(Phase("Lane" if k % 2 == 0 else "Shift", t0, t0 + dur))
        out.append(Phase("Hold", self.t_sweep_end, self.duration))
        return out
