"""이중적분기(double integrator) 모델과 영차 홀드 이산화.

Qi et al. 식 (1)의 병진 부분에 보조 제어입력 U_i^d 를 대입하면
    ṗ = v,  v̇ = U_i^d
즉 정확히 이중적분기가 된다. 이 파일은 그 사실을 한 곳에 못 박아 두는 곳이다.
두 제어기가 같은 모델을 공유해야 비교가 성립하므로 모델은 여기서만 정의한다.
"""
from __future__ import annotations

import numpy as np

NX: int = 4  # 상태 [px, py, vx, vy]
NU: int = 2  # 입력 [ax, ay]


class DoubleIntegrator2D:
    """수평면 2D 이중적분기. 고도는 하위 제어기가 잡는다고 가정한다."""

    def __init__(self, dt: float) -> None:
        if dt <= 0.0:
            raise ValueError("dt 는 양수여야 한다")
        self.dt: float = float(dt)
        d = self.dt
        # 영차 홀드 이산화. 입력이 한 주기 동안 일정하다는 가정이라
        # B 의 위쪽 블록이 dt²/2 (등가속도 변위)가 된다.
        self.A: np.ndarray = np.array(
            [[1.0, 0.0, d, 0.0],
             [0.0, 1.0, 0.0, d],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]], dtype=float)
        self.B: np.ndarray = np.array(
            [[0.5 * d * d, 0.0],
             [0.0, 0.5 * d * d],
             [d, 0.0],
             [0.0, d]], dtype=float)

    def step(self, states: np.ndarray, inputs: np.ndarray) -> np.ndarray:
        """전 기체를 한 번에 전진시킨다.

        전역 상태를 두지 않으려고 in-place 갱신 대신 새 배열을 돌려준다.

        Args:
            states: (M, 4) 현재 상태.
            inputs: (M, 2) 가속도 명령.
        Returns:
            (M, 4) 다음 스텝 상태.
        """
        return states @ self.A.T + inputs @ self.B.T
