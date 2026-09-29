"""제어기 A — Qi et al. (RA-L 2022) 가중합 제어 (baseline).

식 (4) 편대 추종 + 식 (5) 충돌 회피 + 식 (7)~(9) split-merge 조향을
식 (10) 으로 가중합한 뒤 u_max 로 clip 한다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dynamics import DoubleIntegrator2D
from scenario import Scenario


@dataclass
class QiGains:
    """논문 실험값을 기본값으로 둔다. 값이 명시되지 않은 항은 주석에 근거를 적었다."""
    # --- 식 (4) 편대 추종. 논문 실험값 (위치, 속도 성분)
    k1f_p: float = -2.5
    k1f_v: float = -2.8
    k2f_p: float = 1.0
    k2f_v: float = 1.2
    # --- 식 (5) 기체 간 충돌 회피. 논문 실험값
    R_c: float = 1.8
    k1c: float = 40.0
    k2c: float = 2.0
    # --- 식 (6)~(9) 장애물 감지와 조향.
    R_p: float = 5.0          # 감지 반경 (논문값)
    theta_p: float = np.pi / 2  # 시야각 ±90°. 논문은 부채꼴만 정의하고 값은 안 준다
    R_o: float = 3.0          # 조향이 걸리는 거리 (논문값 없음 — R_p 보다 작게)
    k1_obs: float = 1.0
    k2_obs: float = 0.5
    kxv: float = 2.0          # 측방 조향 세기
    kyv: float = -1.0         # 논문: -Y 비행이므로 k_y^v < 0
    use_fov: bool = True
    # --- 식 (10) 가중합. 논문 실험값
    k1d: float = 1.0
    k2d: float = 2.0
    k3d: float = 1.0
    u_max: float = 3.0


class QiController:
    """step(state, t) -> (M, 2) 가속도 명령."""

    name = "qi"

    def __init__(self, scenario: Scenario, gains: QiGains, dt: float) -> None:
        self.sc = scenario
        self.g = gains
        self.dyn = DoubleIntegrator2D(dt)
        self._next_state: np.ndarray | None = None
        # MPC 와 인터페이스를 맞추기 위한 기록용. 실제로 쓰이지 않는다.
        self.solve_ms = float("nan")
        self.iterations = float("nan")
        self.slack = float("nan")
        self.failed = False

    # ------------------------------------------------------------------
    def step(self, state: np.ndarray, t: float) -> np.ndarray:
        m = state.shape[0]
        ref = self.sc.reference(t)
        err = state - ref                    # s_i - c_i

        u_f = self._formation(err, self.sc.reference_accel(t), m)
        u_c = self._collision(state, m)
        u_o = self._obstacle(state, t, m)

        # Qi et al. 식 (10)
        u = self.g.k1d * u_f - self.g.k2d * u_c - self.g.k3d * u_o

        # 논문이 하는 그대로의 성분별 clip.
        # ★ baseline 의 약점: 성분별로 자르므로 합력의 방향이 틀어지고, 속도 한계는
        #   애초에 고려 대상이 아니다. MPC 쪽은 이 둘을 제약으로 정식화한다.
        u = np.clip(u, -self.g.u_max, self.g.u_max)

        self._next_state = self.dyn.step(state, u)
        return u

    @property
    def reference_setpoint(self) -> np.ndarray | None:
        """다음 주기의 (M, 4) 상태 예측.

        ROS 로 넘어가면 cmdFullState(pos, vel, acc, ...) 가 가속도가 아니라
        상태 참조를 요구한다. 두 제어기 모두 이 값을 노출해 두어 통합 때
        인터페이스를 다시 손대지 않게 했다.
        """
        return self._next_state

    # ------------------------------------------------------------------
    def _formation(self, err: np.ndarray, ref_acc: np.ndarray, m: int) -> np.ndarray:
        """Qi et al. 식 (4).

        w_ij = 1 (완전 연결). Σ_{j∈U} w_ij[(s_j-c_j)-(s_i-c_i)] 는 j=i 항이 0 이므로
        Σ_j e_j - M·e_i 로 한 번에 계산된다.
        """
        u = self.g.k1f_p * err[:, :2] + self.g.k1f_v * err[:, 2:]
        coupling = err.sum(axis=0)[None, :] - m * err
        u = u + self.g.k2f_p * coupling[:, :2] + self.g.k2f_v * coupling[:, 2:]
        return u + ref_acc  # v̇_ci 피드포워드

    def _collision(self, state: np.ndarray, m: int) -> np.ndarray:
        """Qi et al. 식 (5) — Hooke 법칙 + damping.

        식 (10) 에서 -k2d 로 들어가므로, 여기서는 "멀어질 방향의 반대"인
        p_ji/||p_ji|| 방향으로 만든다. 부호를 두 번 뒤집지 않도록 주의.
        """
        u = np.zeros((m, 2))
        for i in range(m):
            for j in range(m):
                if i == j:
                    continue
                p_ij = state[i, :2] - state[j, :2]
                dist = float(np.linalg.norm(p_ij))
                if dist >= self.g.R_c or dist < 1e-9:
                    continue
                # 스프링: 가까울수록 (R_c - d) 가 커진다
                u[i] += self.g.k1c * (self.g.R_c - dist) * (-p_ij / dist)
                # 댐퍼: 상대속도까지 보므로 빠르게 접근하면 미리 강하게 반응한다
                u[i] += self.g.k2c * (state[i, 2:] - state[j, 2:])
        return u

    def _obstacle(self, state: np.ndarray, t: float, m: int) -> np.ndarray:
        """Qi et al. 식 (6)~(9) — 부채꼴 감지 + split 조향.

        논문 식 (8)은 표기가 느슨해(차원이 맞지 않는다) 그대로 옮길 수 없다.
        여기서는 성질만 보존한다: 감지 반경 안에 들면 "드론이 이미 있는 쪽"으로
        측방(x) 을 더 밀어 대형을 쪼개고(split), 종방향(y)은 k_y^v<0 으로 전진
        성분을 유지한다. 식 (10)의 -k3d 부호를 고려해 u_o 는 회피 방향의 반대다.
        """
        u = np.zeros((m, 2))
        obs = self.sc.obstacle_position(t)
        if obs is None:
            return u
        for i in range(m):
            rel = state[i, :2] - obs          # 장애물 -> 드론
            dist = float(np.linalg.norm(rel))
            if dist > self.g.R_p:            # 식 (6) 첫 조건
                continue
            if self.g.use_fov and not self._in_fov(state[i], rel):
                continue                      # 식 (6) 둘째 조건
            sx = 1.0 if rel[0] >= 0.0 else -1.0
            sy = 1.0 if rel[1] >= 0.0 else -1.0
            # 정면 충돌(rel[0]==0)에서 sign 이 0 이 되어 조향이 사라지는 교착을
            # 막으려고 sign 을 ±1 로만 둔다 (0 을 +1 로 흡수).
            f_x = -sx * self.g.kxv * max(self.g.R_o - abs(rel[0]), 0.0)
            f_y = -sy * self.g.kyv
            u[i] = self.g.k1_obs * np.array([f_x, f_y]) + self.g.k2_obs * state[i, 2:]
        return u

    def _in_fov(self, state_i: np.ndarray, rel: np.ndarray) -> bool:
        """식 (6) 의 각도 조건. 비행 방향 기준 ±theta_p 안에 장애물이 있는가."""
        vel = state_i[2:]
        if float(np.linalg.norm(vel)) < 0.05:
            return True  # 거의 정지 상태면 비행 방향이 정의되지 않아 통과시킨다
        heading = np.arctan2(vel[1], vel[0])
        bearing = np.arctan2(-rel[1], -rel[0])   # 드론 -> 장애물
        diff = (bearing - heading + np.pi) % (2.0 * np.pi) - np.pi
        return abs(diff) <= self.g.theta_p
