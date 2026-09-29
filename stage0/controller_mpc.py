"""제어기 B — 중앙집중 MPC (제안 방식).

Qi 식 (10)의 가중합을 QP 로 대체한다. 편대 추종은 비용함수로, 충돌·장애물 회피는
제약으로 옮긴다. 분산화는 다음 단계로 미루고 전 기체를 하나의 문제로 푼다
(상호성 문제 — 두 기체가 서로 반대로 선형화해 제약이 어긋나는 것 — 을 피하려고).

결정변수 배치 (모두 연속 블록):
    x_i[k]   i=0..M-1, k=0..N     : 상태  (M*(N+1)*4)
    u_i[k]   i=0..M-1, k=0..N-1   : 입력  (M*N*2)
    s_p[k]   쌍 p,   k=1..N       : 충돌 제약 슬랙 (쌍당 1개를 두 반공간이 공유)
    s_o[i,k] i,       k=1..N      : 장애물 제약 슬랙
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from itertools import combinations

import numpy as np
import osqp
import scipy.sparse as sp

from dynamics import NU, NX, DoubleIntegrator2D
from scenario import Scenario

INF = np.inf
EPS = 1e-6


@dataclass
class MpcConfig:
    horizon: int = 20
    dt: float = 0.05
    Q: tuple[float, ...] = (10.0, 10.0, 1.0, 1.0)
    R: tuple[float, ...] = (0.1, 0.1)
    Qf_scale: float = 10.0
    u_max: float = 3.0
    v_max: float = 2.0
    R_c: float = 1.8
    r_o: float = 0.5
    margin: float = 0.2
    # 회피 제약은 슬랙 + 큰 페널티로 둔다. 하드로 두면 선형화 기준점이 나쁜 주기에
    # QP 가 비실현이 되고, ADMM 은 비실현을 깔끔하게 알려주지 않아 그냥 발산한다.
    # 페널티가 충분히 크면 실현 가능한 동안 슬랙은 0 이라 하드 제약과 같이 동작하고,
    # 불가능한 순간에도 죽지 않고 "얼마나 못 지켰는지"가 숫자로 남는다.
    slack_lin: float = 5.0e3
    slack_quad: float = 1.0e2
    eps_abs: float = 1.0e-4
    eps_rel: float = 1.0e-4
    max_iter: int = 6000
    polish: bool = True
    settings: dict = field(default_factory=dict)


class CentralizedMPC:
    """step(state, t) -> (M, 2) 가속도 명령. QiController 와 같은 인터페이스."""

    name = "mpc"

    def __init__(self, scenario: Scenario, cfg: MpcConfig, n_agents: int) -> None:
        self.sc = scenario
        self.cfg = cfg
        self.M = int(n_agents)
        self.N = int(cfg.horizon)
        self.dyn = DoubleIntegrator2D(cfg.dt)

        self.pairs: list[tuple[int, int]] = list(combinations(range(self.M), 2))
        self.n_pairs = len(self.pairs)

        self._layout()
        self._build_cost()
        self._build_constraints()
        self._setup_solver()

        self._P_hat: np.ndarray | None = None   # (M, N+1, 2) 선형화 기준 궤적
        self._last_u = np.zeros((self.M, NU))
        self._next_state: np.ndarray | None = None
        self.solve_ms = float("nan")
        self.iterations = float("nan")
        self.slack = 0.0
        self.failed = False
        self.fail_count = 0

    # ============================================================ 인덱싱
    def _layout(self) -> None:
        m, n = self.M, self.N
        self.n_state = m * (n + 1) * NX
        self.n_input = m * n * NU
        self.n_sp = self.n_pairs * n
        self.n_so = m * n
        self.off_u = self.n_state
        self.off_sp = self.off_u + self.n_input
        self.off_so = self.off_sp + self.n_sp
        self.n_var = self.off_so + self.n_so

    def ix(self, i: int, k: int) -> int:
        return (i * (self.N + 1) + k) * NX

    def iu(self, i: int, k: int) -> int:
        return self.off_u + (i * self.N + k) * NU

    def isp(self, p: int, k: int) -> int:
        return self.off_sp + p * self.N + (k - 1)

    def iso(self, i: int, k: int) -> int:
        return self.off_so + i * self.N + (k - 1)

    # ============================================================ 비용
    def _build_cost(self) -> None:
        """P 는 전 주기 불변(대각)이다. 참조궤적만 q 로 매 주기 바뀐다."""
        q_w = np.asarray(self.cfg.Q, dtype=float)
        r_w = np.asarray(self.cfg.R, dtype=float)
        self._Wt = np.tile(q_w, (self.N + 1, 1))
        self._Wt[self.N] = q_w * self.cfg.Qf_scale       # 종단 가중 Qf
        diag = np.zeros(self.n_var)
        diag[: self.n_state] = np.tile(self._Wt.reshape(-1), self.M) * 2.0
        diag[self.off_u : self.off_u + self.n_input] = np.tile(r_w, self.M * self.N) * 2.0
        diag[self.off_sp :] = 2.0 * self.cfg.slack_quad
        self.P = sp.diags(diag, format="csc")

    def _cost_linear(self, t: float) -> np.ndarray:
        """q = -2·W·c. (x-c)'W(x-c) 를 OSQP 의 ½z'Pz + q'z 로 옮긴 형태."""
        refs = self.sc.reference_horizon(t, self.cfg.dt, self.N)     # (N+1, M, 4)
        refs_t = np.swapaxes(refs, 0, 1)                             # (M, N+1, 4)
        q = np.zeros(self.n_var)
        q[: self.n_state] = (-2.0 * self._Wt[None, :, :] * refs_t).reshape(-1)
        q[self.off_sp :] = self.cfg.slack_lin
        return q

    # ============================================================ 제약
    def _build_constraints(self) -> None:
        m, n = self.M, self.N
        A, B = self.dyn.A, self.dyn.B
        rows: list[int] = []
        cols: list[int] = []
        vals: list[float] = []
        lo: list[float] = []
        up: list[float] = []
        r = 0

        def add(row: int, col: int, val: float) -> None:
            rows.append(row)
            cols.append(col)
            vals.append(val)

        # (0) 초기 조건 — 등식
        self.ic_rows = np.arange(r, r + m * NX)
        for i in range(m):
            for c in range(NX):
                add(r, self.ix(i, 0) + c, 1.0)
                lo.append(0.0)
                up.append(0.0)
                r += 1

        # (a) 동역학 — 등식. 계수가 상수라 이후 갱신이 필요 없다.
        for i in range(m):
            for k in range(n):
                for c in range(NX):
                    add(r, self.ix(i, k + 1) + c, 1.0)
                    for cc in range(NX):
                        if A[c, cc] != 0.0:
                            add(r, self.ix(i, k) + cc, -float(A[c, cc]))
                    for cc in range(NU):
                        if B[c, cc] != 0.0:
                            add(r, self.iu(i, k) + cc, -float(B[c, cc]))
                    lo.append(0.0)
                    up.append(0.0)
                    r += 1

        # (b) 입력 박스
        # ★ 성분별 박스는 대각선 방향에서 √2 배를 과대 허용한다 (|ax|,|ay|<=u_max 는
        #   ||u||<=√2·u_max 를 허용). 엄밀히 하려면 2-노름 원뿔 제약이 필요하고,
        #   그건 OSQP(QP)로는 못 하며 SOCP 솔버 또는 다각형 근사가 필요하다.
        #   Qi 의 성분별 clip 과 같은 근사라 baseline 과 조건이 대등하다.
        for i in range(m):
            for k in range(n):
                for c in range(NU):
                    add(r, self.iu(i, k) + c, 1.0)
                    lo.append(-self.cfg.u_max)
                    up.append(self.cfg.u_max)
                    r += 1

        # (b') 속도 박스 — k=0 은 초기조건으로 고정되어 있으므로 제외한다.
        #      측정 속도가 한계를 넘은 순간 문제 전체가 비실현이 되기 때문이다.
        for i in range(m):
            for k in range(1, n + 1):
                for c in (2, 3):
                    add(r, self.ix(i, k) + c, 1.0)
                    lo.append(-self.cfg.v_max)
                    up.append(self.cfg.v_max)
                    r += 1

        # (c) 기체 간 충돌 — buffered Voronoi cell.
        #     계수 n_ij[k] 가 매 주기 바뀌므로 자리만 잡아두고 값은 나중에 덮는다.
        self.pair_row_i = np.zeros((self.n_pairs, n), dtype=int)
        self.pair_row_j = np.zeros((self.n_pairs, n), dtype=int)
        pair_pos_key: list[tuple[int, int, int, int]] = []
        for p, (i, j) in enumerate(self.pairs):
            for kk in range(n):
                k = kk + 1
                add(r, self.ix(i, k) + 0, 1.0)
                add(r, self.ix(i, k) + 1, 1.0)
                add(r, self.isp(p, k), 1.0)      # +슬랙: 하한을 완화
                self.pair_row_i[p, kk] = r
                lo.append(-INF)
                up.append(INF)
                r += 1
                add(r, self.ix(j, k) + 0, 1.0)
                add(r, self.ix(j, k) + 1, 1.0)
                add(r, self.isp(p, k), -1.0)     # -슬랙: 상한을 완화
                self.pair_row_j[p, kk] = r
                lo.append(-INF)
                up.append(INF)
                r += 1
                pair_pos_key.append((p, kk, i, j))

        # (d) 장애물
        self.obs_row = np.zeros((m, n), dtype=int)
        for i in range(m):
            for kk in range(n):
                k = kk + 1
                add(r, self.ix(i, k) + 0, 1.0)
                add(r, self.ix(i, k) + 1, 1.0)
                add(r, self.iso(i, k), 1.0)
                self.obs_row[i, kk] = r
                lo.append(-INF)
                up.append(INF)
                r += 1

        # (e) 슬랙 비음수
        for v in range(self.off_sp, self.n_var):
            add(r, v, 1.0)
            lo.append(0.0)
            up.append(INF)
            r += 1

        self.n_row = r
        self.A0 = sp.csc_matrix((vals, (rows, cols)), shape=(r, self.n_var))
        self._A_data0 = self.A0.data.copy()
        self._lo0 = np.asarray(lo, dtype=float)
        self._up0 = np.asarray(up, dtype=float)

        # CSC 자료배열 안에서 (행, 열) 이 어디인지 미리 찾아둔다.
        # 매 주기 A 를 다시 만들지 않고 이 위치만 덮어써야 osqp.update(Ax=...) 가
        # 같은 희소 구조를 유지한다.
        pos: dict[tuple[int, int], int] = {}
        for c in range(self.n_var):
            for pp in range(self.A0.indptr[c], self.A0.indptr[c + 1]):
                pos[(int(self.A0.indices[pp]), c)] = pp
        self._pair_data = np.zeros((self.n_pairs, n, 4), dtype=int)
        for (p, kk, i, j) in pair_pos_key:
            k = kk + 1
            ri, rj = int(self.pair_row_i[p, kk]), int(self.pair_row_j[p, kk])
            self._pair_data[p, kk] = [
                pos[(ri, self.ix(i, k) + 0)], pos[(ri, self.ix(i, k) + 1)],
                pos[(rj, self.ix(j, k) + 0)], pos[(rj, self.ix(j, k) + 1)],
            ]
        self._obs_data = np.zeros((m, n, 2), dtype=int)
        for i in range(m):
            for kk in range(n):
                k = kk + 1
                ro = int(self.obs_row[i, kk])
                self._obs_data[i, kk] = [pos[(ro, self.ix(i, k) + 0)],
                                         pos[(ro, self.ix(i, k) + 1)]]

        # 선형화 기준점이 겹쳐 방향이 정의되지 않을 때 쓰는 결정론적 대체 방향.
        # 무작위로 두면 교착에서 매 주기 방향이 바뀌어 진동한다.
        ang_p = 2.0 * np.pi * np.arange(max(self.n_pairs, 1)) / max(self.n_pairs, 1)
        self._fb_pair = np.stack([np.cos(ang_p), np.sin(ang_p)], axis=1)
        ang_a = 2.0 * np.pi * np.arange(m) / m
        self._fb_agent = np.stack([np.cos(ang_a), np.sin(ang_a)], axis=1)
        self._pi = np.array([p[0] for p in self.pairs], dtype=int) if self.n_pairs else np.zeros(0, int)
        self._pj = np.array([p[1] for p in self.pairs], dtype=int) if self.n_pairs else np.zeros(0, int)

    # ============================================================ 솔버
    def _setup_solver(self) -> None:
        settings = dict(
            eps_abs=self.cfg.eps_abs,
            eps_rel=self.cfg.eps_rel,
            max_iter=self.cfg.max_iter,
            polish=self.cfg.polish,
            warm_start=True,
            verbose=False,
        )
        settings.update(self.cfg.settings)
        q0 = np.zeros(self.n_var)
        # osqp 0.6.x 와 1.x 의 생성 방식이 다르다. 둘 다 받아준다.
        prob = None
        legacy = False
        try:
            prob = osqp.OSQP()
            legacy = hasattr(prob, "setup")
        except TypeError:
            legacy = False
        if legacy:
            prob.setup(P=self.P, q=q0, A=self.A0,
                       l=self._lo0.copy(), u=self._up0.copy(), **settings)
        else:
            prob = osqp.OSQP(P=self.P, q=q0, A=self.A0,
                             l=self._lo0.copy(), u=self._up0.copy(), **settings)
        self.prob = prob

    # ============================================================ 한 주기
    def step(self, state: np.ndarray, t: float) -> np.ndarray:
        n = self.N
        if self._P_hat is None:
            # 첫 주기에는 예측 궤적이 없으므로 현재 위치를 N+1 번 복제해 쓴다.
            self._P_hat = np.repeat(state[:, None, :2], n + 1, axis=1)
        p_hat = self._P_hat

        q = self._cost_linear(t)
        a_data = self._A_data0.copy()
        lo = self._lo0.copy()
        up = self._up0.copy()

        lo[self.ic_rows] = state.reshape(-1)
        up[self.ic_rows] = state.reshape(-1)

        # --- 충돌 회피 반공간 (buffered Voronoi cell)
        # 서로의 중점에서 각자 R_c/2 씩 물러나므로 두 허용영역이 겹치지 않는다.
        # 각자 n'(p_i - p̂_j) >= R_c 로 걸면 과잉 보수적이고 상호 일관성도 깨진다.
        if self.n_pairs:
            pi = p_hat[self._pi][:, 1:, :]              # (P, N, 2)
            pj = p_hat[self._pj][:, 1:, :]
            diff = pi - pj
            nrm = np.linalg.norm(diff, axis=2)
            safe = nrm > EPS
            nvec = np.where(safe[..., None], diff / np.maximum(nrm, EPS)[..., None],
                            self._fb_pair[:, None, :])
            mid = 0.5 * (pi + pj)
            nm = np.einsum("pkc,pkc->pk", nvec, mid)
            lo[self.pair_row_i] = nm + 0.5 * self.cfg.R_c
            up[self.pair_row_j] = nm - 0.5 * self.cfg.R_c
            a_data[self._pair_data[..., 0]] = nvec[..., 0]
            a_data[self._pair_data[..., 1]] = nvec[..., 1]
            a_data[self._pair_data[..., 2]] = nvec[..., 0]
            a_data[self._pair_data[..., 3]] = nvec[..., 1]

        # --- 장애물 반공간. 예측 시점 k 의 장애물 위치를 그대로 쓴다.
        o_pos, o_act = self.sc.obstacle_horizon(t, self.cfg.dt, n)
        o_k = o_pos[1:]                                  # (N, 2)
        act_k = o_act[1:]
        diff_o = p_hat[:, 1:, :] - o_k[None, :, :]       # (M, N, 2)
        nrm_o = np.linalg.norm(diff_o, axis=2)
        safe_o = nrm_o > EPS
        n_o = np.where(safe_o[..., None], diff_o / np.maximum(nrm_o, EPS)[..., None],
                       self._fb_agent[:, None, :])
        bound = np.einsum("mkc,kc->mk", n_o, o_k) + self.cfg.r_o + self.cfg.margin
        # 비활성 스텝은 행을 지우는 대신 하한을 -inf 로 눌러 무력화한다
        # (희소 구조를 바꾸면 osqp.update 를 못 쓴다).
        lo[self.obs_row] = np.where(act_k[None, :], bound, -INF)
        a_data[self._obs_data[..., 0]] = n_o[..., 0]
        a_data[self._obs_data[..., 1]] = n_o[..., 1]

        self.prob.update(q=q, l=lo, u=up, Ax=a_data)
        t0 = time.perf_counter()
        res = self.prob.solve()
        self.solve_ms = (time.perf_counter() - t0) * 1e3

        status = str(getattr(res.info, "status", "unknown"))
        self.iterations = float(getattr(res.info, "iter", float("nan")))
        z = getattr(res, "x", None)
        ok = ("solved" in status.lower()) and z is not None and np.all(np.isfinite(z))
        if not ok:
            # 죽지 않는다: 직전 입력을 유지하고 경고만 남긴다.
            self.failed = True
            self.fail_count += 1
            print(f"[mpc] t={t:6.2f}s solve 실패 ({status}) — 직전 입력 유지")
            self._next_state = self.dyn.step(state, self._last_u)
            return self._last_u.copy()

        self.failed = False
        x_sol = z[: self.n_state].reshape(self.M, n + 1, NX)
        u_sol = z[self.off_u : self.off_u + self.n_input].reshape(self.M, n, NU)
        self.slack = float(np.max(z[self.off_sp :])) if self.n_var > self.off_sp else 0.0

        # 다음 주기의 선형화 기준점 = 이번 해를 한 스텝 shift (warm start 겸용)
        nxt = np.empty((self.M, n + 1, 2))
        nxt[:, :n, :] = x_sol[:, 1:, :2]
        nxt[:, n, :] = x_sol[:, n, :2]
        self._P_hat = nxt

        self._last_u = u_sol[:, 0, :].copy()
        self._next_state = x_sol[:, 1, :].copy()
        return self._last_u.copy()

    @property
    def reference_setpoint(self) -> np.ndarray | None:
        """다음 주기의 (M, 4) 상태 예측 — ROS 에서 cmdFullState 에 넘길 값."""
        return self._next_state
