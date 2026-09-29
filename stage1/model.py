"""예측 모델 — 2D 이중적분기.

Qi et al. 식 (1)의 병진 부분에 보조 제어입력 U_i^d 를 대입하면 ṗ = v, v̇ = U_i^d,
즉 이중적분기다. Stage 0 dynamics.py 와 같은 모델이며, 시뮬레이터의 '실제 플랜트'와
MPC 의 예측 모델이 같다(모델 오차는 SIL 단계에서 다룬다).
고도는 하위 제어기가 일정하게 잡는다고 가정한다.
"""
from __future__ import annotations

import numpy as np

NX: int = 4          # 상태 [px, py, vx, vy]
NU: int = 2          # 입력 [ax, ay]


def double_integrator(dt: float) -> tuple[np.ndarray, np.ndarray]:
    """영차 홀드 이산화. 입력이 한 주기 동안 일정하므로 위치에 dt²/2 가 붙는다."""
    if dt <= 0.0:
        raise ValueError("dt 는 양수여야 한다")
    a = np.array([[1.0, 0.0, dt, 0.0],
                  [0.0, 1.0, 0.0, dt],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0]])
    b = np.array([[0.5 * dt * dt, 0.0],
                  [0.0, 0.5 * dt * dt],
                  [dt, 0.0],
                  [0.0, dt]])
    return a, b


def stacked(dt: float, n_agents: int) -> tuple[np.ndarray, np.ndarray]:
    """중앙집중형용: 기체 M 대를 블록 대각으로 쌓은 (A, B). 상태 [x_0, …, x_{M-1}]."""
    a4, b4 = double_integrator(dt)
    return np.kron(np.eye(n_agents), a4), np.kron(np.eye(n_agents), b4)


def step(states: np.ndarray, inputs: np.ndarray, dt: float) -> np.ndarray:
    """(M, 4) 상태를 한 주기 전진시킨다."""
    a, b = double_integrator(dt)
    return states @ a.T + inputs @ b.T
