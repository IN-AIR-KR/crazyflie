"""시나리오 정의 — 편대 목표, 동적 장애물, 초기 조건.

Qi et al. Fig. 14 를 4단계로 재현한다.
    Formation 0~10s   흩어진 초기 위치에서 편대 형성
    Forward  10~25s   편대를 유지하며 -y 방향 전진
    Obstacle 25~35s   동적 장애물이 편대를 가로질러 통과
    Recovery 35~50s   편대 복귀

편대 목표 c_i(t) 는 나중에 커버리지 기반으로 갈아끼울 자리라 Scenario.reference()
하나로 분리해 뒀다. 제어기는 이 함수만 보고 동작한다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Phase:
    """플롯 음영과 구간별 지표 계산에 쓰는 단계 경계."""
    name: str
    t_start: float
    t_end: float


class Scenario:
    """편대 목표 궤적 + 장애물 궤적 + 초기 조건을 한 묶음으로 들고 있는 객체."""

    def __init__(
        self,
        n_agents: int = 4,
        side: float = 3.0,
        beta: float = 0.0,
        v_forward: float = -1.0,
        t_form: float = 10.0,
        t_fwd_end: float = 25.0,
        t_obs_end: float = 35.0,
        duration: float = 50.0,
        ramp: float = 1.0,
        obs_speed: float = 1.0,
        obs_radius: float = 0.5,
        seed: int = 0,
        spawn_half: float = 5.0,
        spawn_min_sep: float = 2.5,
    ) -> None:
        self.n_agents = int(n_agents)
        self.side = float(side)
        self.beta = float(beta)
        self.v_forward = float(v_forward)
        self.t_form = float(t_form)
        self.t_fwd_end = float(t_fwd_end)
        self.t_obs_end = float(t_obs_end)
        self.duration = float(duration)
        # 편대 중심 속도를 계단으로 주면 v̇_ci (식 4의 피드포워드)가 충격함수가 되어
        # 구현이 불가능하다. 1초 램프로 매끄럽게 올려 v̇_ci 를 해석적으로 정의한다.
        self.ramp = float(ramp)
        self.obs_speed = float(obs_speed)
        self.obs_radius = float(obs_radius)
        self.seed = int(seed)
        self.spawn_half = float(spawn_half)
        self.spawn_min_sep = float(spawn_min_sep)

        self._offsets = self._make_offsets()

        # 장애물은 "편대 선두 기체의 정면"을 노리게 배치한다. 중심을 노리면
        # 다이아몬드 대형 사이로 그냥 빠져나가 회피가 필요 없는 시나리오가 된다.
        self.t_obs_start = self.t_fwd_end
        t_cross = 0.5 * (self.t_obs_start + self.t_obs_end)
        lead = int(np.argmin(self._offsets[:, 1]))  # -y 로 가장 앞선 기체
        target = self.center(t_cross)[0] + self._offsets[lead]
        self._obs_vel = np.array([self.obs_speed, 0.0])  # +x 로 횡단
        self._obs_p0 = target - self._obs_vel * (t_cross - self.t_obs_start)

    # ================================================== 편대 목표
    def _make_offsets(self) -> np.ndarray:
        """Qi et al. p.1717 의 정사각형 대형.

            pc_xi = β + α cos(2π(-i)/M),  pc_yi = α sin(2π(-i)/M),  α = L/√2

        M=4 면 논문과 같은 한 변 L 의 정사각형(45° 회전한 다이아몬드)이 된다.
        M 을 일반화해 두어 --agents 로 기체 수를 바꿔도 돌아가게 했다.
        """
        m = self.n_agents
        alpha = self.side / np.sqrt(2.0)
        idx = np.arange(1, m + 1)
        ang = 2.0 * np.pi * (-idx) / m
        return np.stack([self.beta + alpha * np.cos(ang), alpha * np.sin(ang)], axis=1)

    @property
    def offsets(self) -> np.ndarray:
        return self._offsets.copy()

    def center(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """편대 중심의 (위치, 속도, 가속도).

        t_form 부터 ramp 초에 걸쳐 코사인 smoothstep 으로 v_forward 까지 올린다.
        속도가 연속이므로 식 (4)의 v̇_ci 를 해석적으로 줄 수 있다.
        """
        v = self.v_forward
        r = self.ramp
        if t <= self.t_form:
            return np.zeros(2), np.zeros(2), np.zeros(2)
        tau = (t - self.t_form) / r
        if tau < 1.0:
            s = 0.5 * (1.0 - np.cos(np.pi * tau))
            pos_y = v * r * 0.5 * (tau - np.sin(np.pi * tau) / np.pi)
            vel_y = v * s
            acc_y = v * 0.5 * np.pi / r * np.sin(np.pi * tau)
        else:
            pos_y = v * (t - self.t_form - 0.5 * r)
            vel_y = v
            acc_y = 0.0
        return (np.array([0.0, pos_y]),
                np.array([0.0, vel_y]),
                np.array([0.0, acc_y]))

    def reference(self, t: float) -> np.ndarray:
        """(M, 4) 목표 편대 상태 c_i(t) = [p_ci, v_ci]."""
        cpos, cvel, _ = self.center(t)
        ref = np.zeros((self.n_agents, 4))
        ref[:, :2] = self._offsets + cpos
        ref[:, 2:] = cvel
        return ref

    def reference_accel(self, t: float) -> np.ndarray:
        """(M, 2) 식 (4)의 피드포워드 항 v̇_ci."""
        _, _, cacc = self.center(t)
        return np.tile(cacc, (self.n_agents, 1))

    def reference_horizon(self, t0: float, dt: float, horizon: int) -> np.ndarray:
        """(N+1, M, 4) — MPC 가 참조궤적으로 쓰는 미래 편대 목표.

        MPC 는 여기서 "대형이 앞으로 어디로 갈지"를 미리 알기 때문에
        Qi 가중합보다 유리하다. 비교할 때 이 비대칭을 기억할 것.
        """
        return np.stack([self.reference(t0 + k * dt) for k in range(horizon + 1)])

    # ================================================== 장애물
    def obstacle_position(self, t: float) -> np.ndarray | None:
        """활성 구간 안이면 (2,) 위치, 밖이면 None.

        t_obs_end 는 플롯 단계 라벨일 뿐이고, 장애물은 등장한 뒤 끝까지 남는다.
        아직 5m 안에 있는 장애물이 갑자기 사라지면 회복 구간 판정이 왜곡되고,
        MPC 쪽에서는 제약이 순간적으로 풀려 비물리적인 급기동이 나온다.
        """
        if t < self.t_obs_start:
            return None
        return self._obs_p0 + self._obs_vel * (t - self.t_obs_start)

    def obstacle_horizon(self, t0: float, dt: float,
                         horizon: int) -> tuple[np.ndarray, np.ndarray]:
        """(N+1, 2) 예측 위치와 (N+1,) 활성 플래그.

        비활성 스텝도 자리를 비워두지 않고 마지막 유효값으로 채운다. MPC 의 제약
        행렬 구조를 고정해야 osqp.update() 로 재사용할 수 있기 때문이다 —
        비활성 스텝은 하한을 -inf 로 눌러 행 자체를 무력화한다.
        """
        pos = np.zeros((horizon + 1, 2))
        active = np.zeros(horizon + 1, dtype=bool)
        fallback = self._obs_p0
        for k in range(horizon + 1):
            p = self.obstacle_position(t0 + k * dt)
            if p is None:
                pos[k] = fallback
            else:
                pos[k] = p
                active[k] = True
                fallback = p
        return pos, active

    # ================================================== 초기 조건
    def initial_states(self) -> np.ndarray:
        """(M, 4) 흩어진 초기 위치. 시드 고정으로 재현 가능하다.

        최소 간격을 R_c(=1.8) 보다 크게 잡는다. t=0 에 이미 안전거리를 위반하면
        MPC 의 충돌 제약이 첫 주기부터 비실현이 되어 비교가 무의미해진다.
        """
        rng = np.random.default_rng(self.seed)
        pts: list[np.ndarray] = []
        for _ in range(10000):
            if len(pts) == self.n_agents:
                break
            cand = rng.uniform(-self.spawn_half, self.spawn_half, 2)
            if all(np.linalg.norm(cand - p) >= self.spawn_min_sep for p in pts):
                pts.append(cand)
        if len(pts) < self.n_agents:
            raise RuntimeError("초기 위치 샘플링 실패 — spawn_half 를 키울 것")
        states = np.zeros((self.n_agents, 4))
        states[:, :2] = np.array(pts)
        return states

    # ================================================== 단계
    def phases(self) -> list[Phase]:
        return [
            Phase("Formation", 0.0, self.t_form),
            Phase("Forward", self.t_form, self.t_fwd_end),
            Phase("Obstacle", self.t_fwd_end, self.t_obs_end),
            Phase("Recovery", self.t_obs_end, self.duration),
        ]
