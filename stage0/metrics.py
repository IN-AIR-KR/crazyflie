"""평가 지표 — 두 제어기에 같은 기준을 적용한다.

지표를 제어기 안이 아니라 로그에서 계산하는 이유: 제어기가 자기 성적을 매기면
정의가 서로 달라져 비교가 안 된다. 로그만 보고 계산해야 공정하다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SimLog:
    """시뮬레이션 기록. 모든 배열의 첫 축은 시간."""
    name: str
    t: np.ndarray            # (T,)
    states: np.ndarray       # (T, M, 4)
    refs: np.ndarray         # (T, M, 4)
    inputs: np.ndarray       # (T, M, 2)
    obstacle: np.ndarray     # (T, 2) — 비활성 구간은 nan
    solve_ms: np.ndarray     # (T,) — Qi 는 nan
    iterations: np.ndarray   # (T,) — Qi 는 nan
    slack: np.ndarray        # (T,) — MPC 만
    failed: np.ndarray       # (T,) bool


def pairwise_distances(states: np.ndarray) -> np.ndarray:
    """(T, M, 4) -> (T, n_pairs) 기체 간 거리."""
    pos = states[:, :, :2]
    diff = pos[:, :, None, :] - pos[:, None, :, :]
    dist = np.linalg.norm(diff, axis=3)
    iu = np.triu_indices(states.shape[1], k=1)
    return dist[:, iu[0], iu[1]]


def obstacle_distances(states: np.ndarray, obstacle: np.ndarray) -> np.ndarray:
    """(T, M) 기체-장애물 거리. 장애물 비활성 구간은 nan."""
    return np.linalg.norm(states[:, :, :2] - obstacle[:, None, :], axis=2)


def compute(log: SimLog, R_c: float, R_p: float, r_o: float,
            eps_recovery: float = 0.3) -> dict[str, float]:
    m = log.states.shape[1]
    d_pair = pairwise_distances(log.states)
    d_obs = obstacle_distances(log.states, log.obstacle)
    pos_err = np.linalg.norm(log.states[:, :, :2] - log.refs[:, :, :2], axis=2)  # (T,M)
    speed = np.linalg.norm(log.states[:, :, 2:], axis=2)

    # 회피 구간 = 장애물이 활성이고, 어느 기체든 감지 반경 안에 든 스텝.
    # 두 제어기에 같은 마스크를 쓰려고 "제어기가 반응했는지"가 아니라
    # "기하학적으로 감지 범위인지"로 정의한다.
    # 비활성 구간은 nan 이므로 먼저 마스크로 걸러야 nanmin 이 경고를 내지 않는다.
    obs_valid = ~np.isnan(log.obstacle[:, 0])
    d_obs_min = np.full(log.t.shape, np.inf)
    if np.any(obs_valid):
        d_obs_min[obs_valid] = np.min(d_obs[obs_valid], axis=1)
    detected = d_obs_min <= R_p

    out: dict[str, float] = {}
    out["min_inter_distance"] = float(np.min(d_pair)) if d_pair.size else float("nan")
    out["collision_count"] = float(np.sum(np.min(d_pair, axis=1) < R_c)) if d_pair.size else 0.0
    out["obstacle_violation"] = float(np.sum(d_obs_min < r_o))

    if np.any(detected) and d_pair.size:
        out["max_deviation"] = float(np.max(d_pair[detected]))
    else:
        out["max_deviation"] = float("nan")

    # T_recovery: 장애물이 감지 범위를 벗어난 시점부터 전 기체가 편대로 복귀할 때까지.
    if np.any(detected):
        i_leave = int(np.max(np.where(detected)[0]))
        t_leave = float(log.t[i_leave])
        back = np.all(pos_err < eps_recovery, axis=1)
        after = np.where(back[i_leave:])[0]
        out["T_recovery"] = float(log.t[i_leave + after[0]] - t_leave) if after.size else float("nan")
    else:
        out["T_recovery"] = float("nan")

    out["formation_rmse"] = float(np.sqrt(np.mean(pos_err ** 2)))
    out["max_speed"] = float(np.max(speed))
    out["solve_time_mean"] = float(np.nanmean(log.solve_ms)) if np.any(~np.isnan(log.solve_ms)) else float("nan")
    out["solve_time_max"] = float(np.nanmax(log.solve_ms)) if np.any(~np.isnan(log.solve_ms)) else float("nan")
    out["iterations_mean"] = float(np.nanmean(log.iterations)) if np.any(~np.isnan(log.iterations)) else float("nan")
    out["iterations_max"] = float(np.nanmax(log.iterations)) if np.any(~np.isnan(log.iterations)) else float("nan")
    out["max_slack"] = float(np.nanmax(log.slack)) if np.any(~np.isnan(log.slack)) else float("nan")
    out["solve_failures"] = float(np.sum(log.failed))
    return out


METRIC_ORDER = [
    "min_inter_distance", "max_deviation", "collision_count", "obstacle_violation",
    "T_recovery", "formation_rmse", "max_speed",
    "solve_time_mean", "solve_time_max", "iterations_mean", "iterations_max",
    "max_slack", "solve_failures",
]

METRIC_UNITS = {
    "min_inter_distance": "m", "max_deviation": "m", "collision_count": "steps",
    "obstacle_violation": "steps", "T_recovery": "s", "formation_rmse": "m",
    "max_speed": "m/s", "solve_time_mean": "ms", "solve_time_max": "ms",
    "iterations_mean": "", "iterations_max": "", "max_slack": "m",
    "solve_failures": "steps",
}
