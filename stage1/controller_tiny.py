"""제어기 — TinyMPC 알고리즘(tinyadmm.py)으로 푸는 군집 MPC. 분산형 / 중앙집중형.

Qi 식 (10)  U = k1·u^f − k2·Σu^c − k3·Σu^o  에서
    u^f (편대 추종, 식 4)   → 비용  Σ‖x_k − c_k‖²_Q + ‖u_k‖²_R   (+ 종단 P_f)
    u^c (기체 간, 식 5)     → 제약  이동 BVC 반공간            (constraints.py)
    u^o (장애물, 식 7~9)    → 제약  CPA 기반 시변 반공간        (constraints.py)
    clip(u_max)            → 제약  ‖u‖ ≤ u_max, ‖v‖ ≤ v_max (정팔각형)
로 옮긴다. 편대 추종 게인 K_1^f, K_2^f 의 역할은 Q, R 이 맡는다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

import constraints as cs
from model import NU, NX, double_integrator, stacked
from tinyadmm import _OCT_A, C8, AdmmSettings, TinyADMM, dykstra_halfspaces, project_octagon


@dataclass
class TinyConfig:
    horizon: int = 30              # knot 수 N (예측 길이 = (N-1)·dt)
    dt: float = 0.05
    Q: tuple[float, ...] = (10.0, 10.0, 1.0, 1.0)   # [px, py, vx, vy]
    R: tuple[float, ...] = (0.1, 0.1)
    rho: float = 20.0
    max_iter: int = 100            # 한 주기 ADMM 반복 예산 (MCU 에서도 고정 예산으로 쓴다)
    tol: float = 1e-2              # 원시·쌍대 잔차 기준 (1e-3 과 폐루프 결과 차이 없음, 반복 수만 2배)
    u_max: float = 2.0             # ‖u‖ 한계 [m/s²]
    v_max: float = 1.0             # ‖v‖ 한계 [m/s]
    r_safe: float = 0.3            # 기체 간 최소 거리 [m]
    r_agent: float = 0.05          # 장애물 여유 계산용 기체 반지름 [m]
    margin: float = 0.1            # 장애물 추가 여유 [m]
    R_p: float = 1.5               # 장애물 감지 반경 [m]
    R_comm: float = 3.0            # 이웃으로 보는 거리 [m] (분산형)
    t_recover: float = 0.5         # 여유 부족을 메우는 목표 시간 [s]
    moving_bvc: bool = True        # False 면 정적 BVC (v̄=0) — 절제 실험용
    a_brake_frac: float = 0.5      # 제약 보정에 쓰는 제동 가속도 = frac·u_max
    rate_frac: float = 0.9         # 제약이 요구할 수 있는 최대 속도 = frac·v_max
    s_frac: float = 0.5            # 여유 부족을 메우는 램프 기울기 상한 = frac·v_max


class _SwarmMPC:
    """두 구조가 공유하는 부분: 참조 궤적, 장애물 인식, 기록용 필드."""

    name = "base"

    def __init__(self, scenario, cfg: TinyConfig) -> None:
        self.sc = scenario
        self.cfg = cfg
        self.M = int(scenario.n_agents)
        self.N = int(cfg.horizon)
        self.t_knots = np.arange(self.N) * cfg.dt
        self.s_max = cfg.s_frac * cfg.v_max
        self.rate_max = cfg.rate_frac * cfg.v_max * C8
        self.a_brake = cfg.a_brake_frac * cfg.u_max * C8
        self.settings = AdmmSettings(rho=cfg.rho, max_iter=cfg.max_iter,
                                     abs_pri_tol=cfg.tol, abs_dua_tol=cfg.tol)
        ang = 2.0 * np.pi * np.arange(self.M) / self.M
        self._fb = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        # 기록용 (main.simulate 가 읽는다)
        self.solve_ms = float("nan")
        self.iterations = float("nan")
        self.slack = 0.0            # 이 제어기에서는 '긴급 제약 수'를 기록한다
        self.failed = False
        self.unsolved = 0           # 반복 예산 안에 수렴 판정을 못 받은 (주기·기체) 수
        self.resets = 0             # 발산 감지로 솔버 상태를 비운 횟수
        self.filtered = 0           # 1-스텝 안전 필터가 입력을 고친 (주기·기체) 수
        self.n_constraints = 0
        self.plan: np.ndarray | None = None   # (M, N, 4) 직전 예측 궤적

    def _obstacle_rows(self, i, p, v, obs_p, obs_v, obs_r) -> list[cs.HalfSpace]:
        rows: list[cs.HalfSpace] = []
        hint = self.sc.side_hint(i)
        for k in range(len(obs_r)):
            if np.linalg.norm(p - obs_p[k]) > self.cfg.R_p:
                continue                     # 감지 범위 밖 (Qi 식 6 의 거리 조건)
            clear = float(obs_r[k]) + self.cfg.r_agent + self.cfg.margin
            rows.append(cs.obstacle_halfspace(p, v, obs_p[k], obs_v[k], clear,
                                              self.cfg.t_recover, self.s_max, hint,
                                              self.a_brake, self.rate_max))
        return rows

    def _sane(self, u: np.ndarray, xs: np.ndarray, x0: np.ndarray) -> bool:
        """ADMM 이 발산했는지. 예측 궤적이 물리적으로 갈 수 없는 곳까지 가면 버린다."""
        if not (np.all(np.isfinite(u)) and np.all(np.isfinite(xs))):
            return False
        reach = self.cfg.v_max * self.N * self.cfg.dt * 3.0 + 1.0
        return float(np.max(np.abs(xs - x0[None, :]))) < reach

    def _safety_filter(self, u: np.ndarray, p0: np.ndarray, v0: np.ndarray,
                       rows: list[tuple[np.ndarray, float, float, float, float]]
                       ) -> tuple[np.ndarray, bool]:
        """1-스텝 안전 필터 — ADMM 해를 제동거리 barrier 반공간에 투영한다.

        왜 필요한가: ADMM 을 반복 예산에서 끊으면 원시 해 u 가 제약을 덜 지킨 채 나온다.
        편대 형성 중 기체가 서로 가로지를 때 실제로 안전거리 위반이 관측됐고, 반복을
        3배로 늘려도 없어지지 않았다(수렴이 느린 게 아니라 이미 늦게 감속한 상태에서
        제동 여유 완화가 위반을 허용). 그래서 MPC 가 무엇을 내든 마지막에 한 번 거른다.

        각 회피 반공간(n, b0_safe, b1)에 대해 constraints.barrier_sigma_min 이 주는
        n'u ≥ σ_min 과 입력·속도 팔각형을 u 공간에서 교집합으로 두고 최소 수정 투영한다.
        분산형은 2차원, 중앙집중형은 2M 차원. MCU 에서도 수 µs 수준.

        rows: (u 계수 c (2M',), g0, r0, a_b, σ_cap) — 제약은 c'u ≥ σ_min(g0, r0).
        """
        dt, m = self.cfg.dt, u.shape[0]
        d = 2 * m
        a_list, b_list = [], []
        for c, g0, r0, a_b, cap in rows:
            sig = cs.barrier_sigma_min(g0, r0, dt, a_b, cap)
            if sig > -cap:                                    # 실제로 제약하는 행만
                a_list.append(-c[None, :])
                b_list.append(np.array([-sig]))
        cu, cv = self.cfg.u_max * C8, self.cfg.v_max * C8
        for i in range(m):
            blk = np.zeros((8, d))
            blk[:, 2 * i:2 * i + 2] = _OCT_A
            a_list += [blk, blk * dt]                         # 입력 / 속도(k=1) 팔각형
            b_list += [np.full(8, cu), cv - _OCT_A @ v0[i]]
        a = np.vstack(a_list)
        b = np.concatenate(b_list)
        z = u.reshape(-1)
        if np.all(a @ z <= b + 1e-9):
            return u, False
        out = dykstra_halfspaces(z, a, b, np.ones(len(b), dtype=bool), sweeps=50)
        return out.reshape(m, 2), True

    def _brake(self, v: np.ndarray) -> np.ndarray:
        """해를 못 믿을 때의 대체 입력: 최대 감속으로 정지."""
        return project_octagon(-v / self.cfg.dt, self.cfg.u_max)

    @property
    def reference_setpoint(self) -> np.ndarray | None:
        """다음 주기의 (M, 4) 예측 상태 — ROS 에서 cmdFullState 로 보낼 값."""
        return None if self.plan is None else self.plan[:, 1, :].copy()


# ====================================================================== 분산형
class TinyDistributedMPC(_SwarmMPC):
    """기체마다 자기 MPC 를 푼다 (Crazyflie 온보드에 올라갈 형태).

    기체 i 가 쓰는 정보: 자기 상태, 이웃(R_comm 안)의 현재 위치·속도, 감지 반경
    안의 장애물 위치·속도. Qi 의 분산 구조와 교환 정보량이 같다(기체당 16 B).
    모든 기체가 같은 시각의 정보로 동시에 푼다(동기식) — 비동기/지연은 SIL 에서 본다.

    계산은 기체별로 독립이지만, 파이썬 오버헤드를 줄이려고 M 개 문제를 배치로 한 번에
    돌린다. 반복 수·수렴 여부는 기체별로 따로 센다(각 기체의 MCU 가 쓸 반복 수).
    """

    name = "tiny-dist"

    def __init__(self, scenario, cfg: TinyConfig) -> None:
        super().__init__(scenario, cfg)
        a, b = double_integrator(cfg.dt)
        self.solver = TinyADMM(a, b, cfg.Q, cfg.R, self.N, self.M, self.settings,
                               cfg.u_max, cfg.v_max, vel_pairs=[(2, 3)], u_pairs=[(0, 1)])
        self.iter_hist: list[np.ndarray] = []

    def _rows_for(self, i: int, state: np.ndarray, obs) -> list[cs.HalfSpace]:
        cfg = self.cfg
        p, v = state[i, :2], state[i, 2:]
        rows: list[cs.HalfSpace] = []
        for j in range(self.M):
            if j == i or np.linalg.norm(p - state[j, :2]) > cfg.R_comm:
                continue
            pair_id = min(i, j) * self.M + max(i, j)
            fb = self._fb[pair_id % self.M] * (1.0 if i < j else -1.0)
            rows.append(cs.pair_halfspace_distributed(
                p, v, state[j, :2], state[j, 2:], cfg.r_safe, cfg.t_recover,
                self.s_max, fb, self.a_brake, self.rate_max, cfg.moving_bvc))
        rows += self._obstacle_rows(i, p, v, *obs)
        return rows

    def step(self, state: np.ndarray, t: float) -> np.ndarray:
        m, n = self.M, self.N
        refs = np.swapaxes(self.sc.reference_horizon(t, self.cfg.dt, n), 0, 1)  # (M, N, 4)
        obs = self.sc.obstacles(t)
        per_agent = [self._rows_for(i, state, obs) for i in range(m)]
        c_max = max(1, max(len(r) for r in per_agent))

        # knot 별 반공간:  n'p_k ≥ b0 + b1·t_k   →   [-n, 0, 0]·x_k ≤ -(b0 + b1·t_k)
        Ha = np.zeros((m, n, c_max, NX))
        hb = np.zeros((m, n, c_max))
        act = np.zeros((m, n, c_max), dtype=bool)
        urgent = 0
        for i, rows in enumerate(per_agent):
            for c, h in enumerate(rows):
                Ha[i, :, c, 0:2] = -h.n
                hb[i, :, c] = -(h.b0 + h.b1 * self.t_knots)
                act[i, :, c] = True
                urgent += int(h.urgent)

        t0 = time.perf_counter()
        sol = self.solver.solve(state, refs, Ha, hb, act)
        elapsed = (time.perf_counter() - t0) * 1e3
        u = sol["u"][:, 0, :]
        xs = sol["x"]

        self.failed = False
        if not all(self._sane(u[i], xs[i], state[i]) for i in range(m)):
            # 한 기체라도 발산하면 배치 전체 상태를 비우고 한 번 더 (쌍대변수 초기화)
            self.solver.reset()
            self.resets += 1
            sol = self.solver.solve(state, refs, Ha, hb, act)
            u, xs = sol["u"][:, 0, :], sol["x"]
        u_out = np.zeros((m, NU))
        plan = xs.copy()
        for i in range(m):
            if self._sane(u[i], xs[i], state[i]):
                p_i, v_i = state[i, :2], state[i, 2:]
                cap = self.cfg.u_max * C8
                rows = [(h.n, float(h.n @ p_i) - h.b0_safe, float(h.n @ v_i) - h.b1,
                         self.a_brake, cap) for h in per_agent[i]]
                ui, changed = self._safety_filter(u[i][None], p_i[None], v_i[None], rows)
                u_out[i] = ui[0]
                self.filtered += int(changed)
            else:
                self.failed = True
                u_out[i] = self._brake(state[i, 2:])
                plan[i] = np.repeat(state[i][None, :], n, axis=0)

        self.plan = plan
        self.solve_ms = elapsed / m            # 기체 하나 몫 (배치 시간을 나눈 근사)
        self.iterations = float(np.max(sol["iters"]))
        self.iter_hist.append(sol["iters"].copy())
        self.unsolved += int(np.sum(~sol["solved"]))
        self.slack = float(urgent)
        self.n_constraints = int(sum(len(r) for r in per_agent))
        return u_out


# ==================================================================== 중앙집중형
class TinyCentralMPC(_SwarmMPC):
    """전 기체를 쌓은 하나의 문제 (상태 4M, 입력 2M).

    기체 간 제약을 두 궤적에 동시에 거므로 중점 가정이 필요 없고 덜 보수적이다.
    대신 한 곳에서 전원의 정보를 모아야 하고, 문제 크기와 반공간 수가 M² 로 는다.
    분산형과 같은 솔버·같은 제약 기하를 써서 '구조 차이'만 비교되게 했다.
    """

    name = "tiny-central"

    def __init__(self, scenario, cfg: TinyConfig) -> None:
        super().__init__(scenario, cfg)
        m = self.M
        a, b = stacked(cfg.dt, m)
        self.solver = TinyADMM(a, b, np.tile(cfg.Q, m), np.tile(cfg.R, m), self.N, 1,
                               self.settings, cfg.u_max, cfg.v_max,
                               vel_pairs=[(NX * i + 2, NX * i + 3) for i in range(m)],
                               u_pairs=[(NU * i, NU * i + 1) for i in range(m)])
        self.iter_hist: list[np.ndarray] = []

    def step(self, state: np.ndarray, t: float) -> np.ndarray:
        cfg, m, n = self.cfg, self.M, self.N
        refs = self.sc.reference_horizon(t, cfg.dt, n).reshape(n, m * NX)[None]
        obs_p, obs_v, obs_r = self.sc.obstacles(t)
        rows_a: list[np.ndarray] = []      # (4M,) 계수
        rows_b: list[np.ndarray] = []      # (N,) 우변
        barrier: list[tuple] = []          # 안전 필터용 (u 계수, g0, r0, a_b, σ_cap)
        cap = cfg.u_max * C8
        urgent = 0
        for i in range(m):
            for j in range(i + 1, m):
                h = cs.pair_halfspace_central(i, j, state[i, :2], state[i, 2:],
                                              state[j, :2], state[j, 2:], cfg.r_safe,
                                              cfg.t_recover, self.s_max,
                                              self._fb[(i * m + j) % m],
                                              self.a_brake, self.rate_max)
                row = np.zeros(NX * m)
                row[NX * i:NX * i + 2] = -h.n
                row[NX * j:NX * j + 2] = h.n
                rows_a.append(row)
                rows_b.append(-(h.b0 + h.b1 * self.t_knots))
                urgent += int(h.urgent)
                cu_ = np.zeros(NU * m)
                cu_[NU * i:NU * i + 2], cu_[NU * j:NU * j + 2] = h.n, -h.n
                dp, dv = state[i, :2] - state[j, :2], state[i, 2:] - state[j, 2:]
                barrier.append((cu_, float(h.n @ dp) - h.b0_safe, float(h.n @ dv) - h.b1,
                                2.0 * self.a_brake, 2.0 * cap))
            for h in self._obstacle_rows(i, state[i, :2], state[i, 2:], obs_p, obs_v, obs_r):
                row = np.zeros(NX * m)
                row[NX * i:NX * i + 2] = -h.n
                rows_a.append(row)
                rows_b.append(-(h.b0 + h.b1 * self.t_knots))
                urgent += int(h.urgent)
                cu_ = np.zeros(NU * m)
                cu_[NU * i:NU * i + 2] = h.n
                barrier.append((cu_, float(h.n @ state[i, :2]) - h.b0_safe,
                                float(h.n @ state[i, 2:]) - h.b1, self.a_brake, cap))
        c = len(rows_a)
        Ha = np.broadcast_to(np.array(rows_a)[None, None], (1, n, c, NX * m)).copy()
        hb = np.array(rows_b).T[None]                          # (1, N, C)
        act = np.ones((1, n, c), dtype=bool)

        x0 = state.reshape(1, -1)
        t0 = time.perf_counter()
        sol = self.solver.solve(x0, refs, Ha, hb, act)
        if not self._sane(sol["u"][0, 0], sol["x"][0], x0[0]):
            self.solver.reset()
            self.resets += 1
            sol = self.solver.solve(x0, refs, Ha, hb, act)
        self.solve_ms = (time.perf_counter() - t0) * 1e3
        self.iterations = float(sol["iters"][0])
        self.iter_hist.append(sol["iters"].copy())
        self.unsolved += int(not sol["solved"][0])
        self.slack = float(urgent)
        self.n_constraints = c

        self.failed = not self._sane(sol["u"][0, 0], sol["x"][0], x0[0])
        if self.failed:
            self.plan = np.repeat(state[:, None, :], n, axis=1)
            return np.stack([self._brake(state[i, 2:]) for i in range(m)])
        u = sol["u"][0, 0].reshape(m, NU)
        self.plan = np.swapaxes(sol["x"][0].reshape(n, m, NX), 0, 1)
        u, changed = self._safety_filter(u, state[:, :2], state[:, 2:], barrier)
        self.filtered += int(changed)
        return u
