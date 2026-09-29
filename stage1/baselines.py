"""비교 기준선 — Stage 0 의 제어기와 시나리오를 Stage 1 인터페이스에 맞춘다.

Stage 0 코드는 고치지 않고 가져다 쓴다(정식화 검증 결과를 그대로 기준으로 삼으려고).
    QiMulti        : Qi 식 (10) 제어기. Stage 0 은 장애물 1개만 보므로 식 (10) 의
                     Σ_k u^o_ik 를 장애물 여러 개로 확장했다.
    qi_gains_for   : 실내 Crazyflie 규모로 Qi 게인을 옮기는 규칙.
    Qi14Adapter    : Stage 0 의 Qi Fig.14 시나리오에 Stage 1 제어기가 요구하는
                     obstacles()/side_hint()/knot 수 규약을 붙인다.
"""
from __future__ import annotations

import os
import sys

import numpy as np

_STAGE0 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stage0")
if _STAGE0 not in sys.path:
    sys.path.append(_STAGE0)      # 맨 뒤에 둬야 stage1 의 metrics/plots 가 가려지지 않는다

from controller_qi import QiController, QiGains  # noqa: E402  (stage0)
from scenario import Scenario as Qi14Scenario    # noqa: E402  (stage0)


def qi_gains_for(r_act: float, R_p: float, R_o: float, u_max: float) -> QiGains:
    """논문 게인을 길이 척도 L = r_act / 1.8 로 옮긴다.

    거리에 비례하는 항(식 5 의 스프링 (R_c − d), 식 9 의 (R_o − |Δx|))은 거리가 L 배로
    줄면 힘도 L 배로 줄므로 게인을 1/L 배 한다. 속도·편대 게인(단위 s⁻¹, s⁻²)은 척도와
    무관해 그대로 둔다. 이 규칙 외의 튜닝은 하지 않았다 — 기준선을 일부러 약하게도,
    따로 강하게도 만들지 않으려는 것이다.
    """
    g = QiGains()
    scale = r_act / g.R_c
    return QiGains(R_c=r_act, k1c=g.k1c / scale, R_p=R_p, R_o=R_o,
                   kxv=g.kxv / scale, u_max=u_max)


class _OneObstacle:
    """부모 _obstacle 이 보는 sc.obstacle_position(t) 를 장애물 하나로 고정하는 창."""

    def __init__(self, p: np.ndarray) -> None:
        self._p = p

    def obstacle_position(self, t: float) -> np.ndarray:
        return self._p


class QiMulti(QiController):
    """Qi 제어기 + 장애물 여러 개 (식 10 의 Σ_k u^o_ik).

    장애물마다 Stage 0 의 단일 장애물 로직(식 6~9)을 그대로 돌려 더한다.
    """

    def _obstacle(self, state: np.ndarray, t: float, m: int) -> np.ndarray:
        obs_p, _, _ = self.sc.obstacles(t)
        u = np.zeros((m, 2))
        sc = self.sc
        try:
            for o in obs_p:
                self.sc = _OneObstacle(o)
                u += super()._obstacle(state, t, m)
        finally:
            self.sc = sc
        return u


class Qi14Adapter:
    """Stage 0 Qi Fig.14 시나리오 → Stage 1 제어기 인터페이스.

    Stage 0 의 reference_horizon(t, dt, horizon) 은 horizon+1 개를 돌려주고,
    Stage 1 은 knot 수 N 을 받는다. 장애물은 1개(속도·반지름은 시나리오 내부값).
    """

    def __init__(self, sc: Qi14Scenario) -> None:
        self.inner = sc
        self.n_agents = sc.n_agents
        self.duration = sc.duration

    def reference(self, t):
        return self.inner.reference(t)

    def reference_accel(self, t):
        return self.inner.reference_accel(t)

    def reference_horizon(self, t0, dt, knots):
        return self.inner.reference_horizon(t0, dt, knots - 1)

    def obstacles(self, t):
        p = self.inner.obstacle_position(t)
        if p is None:
            return np.zeros((0, 2)), np.zeros((0, 2)), np.zeros(0)
        return p[None], self.inner._obs_vel[None], np.array([self.inner.obs_radius])

    def side_hint(self, i):
        off = self.inner.offsets[i]
        return off if np.linalg.norm(off) > 1e-9 else np.array([1.0, 0.0])

    def initial_states(self):
        return self.inner.initial_states()

    def phases(self):
        return self.inner.phases()
