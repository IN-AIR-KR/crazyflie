"""평가 지표 — 로그만 보고 계산한다 (Stage 0 과 같은 원칙: 제어기가 자기 성적을 매기지 않는다).

안전  : 기체 간 최소 거리, 장애물 표면까지 최소 여유, 위반 스텝 수
임무  : 스윕 커버리지(실제 / 계획), 편대 오차, 회피 후 복귀 시간
계산  : 반복 수 분포, 수렴 실패, 솔버 재설정, 한 주기 계산 시간(PC 파이썬 기준)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SimLog:
    name: str
    t: np.ndarray              # (T,)
    states: np.ndarray         # (T, M, 4)
    refs: np.ndarray           # (T, M, 4)
    inputs: np.ndarray         # (T, M, 2)
    obs_pos: np.ndarray        # (T, K, 2) — 아직 안 나타난 장애물은 nan
    obs_rad: np.ndarray        # (K,)
    solve_ms: np.ndarray       # (T,)
    iterations: np.ndarray     # (T,) 이 주기에 기체들이 쓴 최대 반복 수
    urgent: np.ndarray         # (T,) 목표 여유를 이미 못 지키는 제약 수
    failed: np.ndarray         # (T,) bool
    extra: dict = field(default_factory=dict)


# ------------------------------------------------------------------ 커버리지
class CoverageGrid:
    """영역을 격자로 나누고, 기체 센서 footprint(원) 안에 한 번이라도 든 칸을 센다."""

    def __init__(self, region: tuple[float, float, float, float], res: float = 0.05) -> None:
        x0, x1, y0, y1 = region
        self.xs = np.arange(x0 + 0.5 * res, x1, res)
        self.ys = np.arange(y0 + 0.5 * res, y1, res)
        self.X, self.Y = np.meshgrid(self.xs, self.ys, indexing="xy")
        self.res = res

    def covered(self, positions: np.ndarray, radius: float) -> np.ndarray:
        """positions: (S, 2) 방문 위치 → (ny, nx) bool."""
        mask = np.zeros(self.X.shape, dtype=bool)
        r2 = radius * radius
        for p in positions:
            # 원을 덮는 사각 창만 계산 (전체 격자 대비 수백 배 빠르다)
            ix0 = max(int((p[0] - radius - self.xs[0]) / self.res), 0)
            ix1 = min(int((p[0] + radius - self.xs[0]) / self.res) + 2, len(self.xs))
            iy0 = max(int((p[1] - radius - self.ys[0]) / self.res), 0)
            iy1 = min(int((p[1] + radius - self.ys[0]) / self.res) + 2, len(self.ys))
            if ix0 >= ix1 or iy0 >= iy1:
                continue
            dx = self.X[iy0:iy1, ix0:ix1] - p[0]
            dy = self.Y[iy0:iy1, ix0:ix1] - p[1]
            mask[iy0:iy1, ix0:ix1] |= dx * dx + dy * dy <= r2
        return mask


def coverage_series(log: SimLog, grid: CoverageGrid, radius: float,
                    every: int = 10) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """시간에 따른 커버리지 비율과 최종 격자. every 스텝마다 누적 비율을 기록."""
    mask = np.zeros(grid.X.shape, dtype=bool)
    ts, ratio = [], []
    pos = log.states[:, :, :2]
    for k in range(0, len(log.t), every):
        chunk = pos[k:k + every].reshape(-1, 2)
        mask |= grid.covered(chunk, radius)
        ts.append(log.t[min(k + every, len(log.t)) - 1])
        ratio.append(mask.mean())
    return np.array(ts), np.array(ratio), mask


# ------------------------------------------------------------------ 기하
def pair_distances(states: np.ndarray) -> np.ndarray:
    pos = states[:, :, :2]
    d = np.linalg.norm(pos[:, :, None] - pos[:, None], axis=3)
    iu = np.triu_indices(states.shape[1], k=1)
    return d[:, iu[0], iu[1]]


def obstacle_surface_distance(log: SimLog) -> np.ndarray:
    """(T, M, K) 기체 중심 → 장애물 표면 거리. 장애물이 없으면 inf."""
    if log.obs_pos.shape[1] == 0:
        return np.full(log.states.shape[:2] + (1,), np.inf)
    d = np.linalg.norm(log.states[:, :, None, :2] - log.obs_pos[:, None], axis=3)
    d = d - log.obs_rad[None, None]
    return np.where(np.isnan(d), np.inf, d)


# ------------------------------------------------------------------ 종합
def compute(log: SimLog, *, r_safe: float, r_agent: float, clearance: float, R_p: float,
            eps_recovery: float, coverage: dict | None = None,
            iter_hist: np.ndarray | None = None) -> dict[str, float]:
    out: dict[str, float] = {}
    dp = pair_distances(log.states)
    ds = obstacle_surface_distance(log)
    ds_min = ds.min(axis=(1, 2))                       # (T,)
    err = np.linalg.norm(log.states[:, :, :2] - log.refs[:, :, :2], axis=2)   # (T, M)
    speed = np.linalg.norm(log.states[:, :, 2:], axis=2)
    acc = np.linalg.norm(log.inputs, axis=2)

    out["min_inter_distance"] = float(dp.min()) if dp.size else float("nan")
    out["pair_violation"] = float(np.sum(dp.min(axis=1) < r_safe - 1e-3)) if dp.size else 0.0
    out["min_obs_surface"] = float(ds_min.min())
    out["obs_contact"] = float(np.sum(ds_min < r_agent))               # 물리적 접촉
    out["obs_clearance_violation"] = float(np.sum(ds_min < clearance - 1e-3))

    # 회피 구간: 어느 기체든 장애물 감지 반경 안. 그 구간의 최대 편대 이탈.
    near = ds_min <= R_p
    out["max_deviation"] = float(err[near].max()) if np.any(near) else float("nan")
    # 복귀 시간: 장애물마다 최근접 순간부터 전 기체가 편대 오차 eps 안으로 돌아올 때까지.
    ok = np.all(err < eps_recovery, axis=1)
    rec = []
    for k in range(ds.shape[2]):
        dk = ds[:, :, k].min(axis=1)
        if not np.any(dk <= R_p):
            continue
        j = int(np.argmin(dk))
        after = np.where(ok[j:])[0]
        rec.append(float(log.t[j + after[0]] - log.t[j]) if after.size else float("nan"))
    out["T_recovery_max"] = float(np.nanmax(rec)) if rec and not np.all(np.isnan(rec)) else float("nan")

    moving = np.linalg.norm(log.refs[:, :, 2:], axis=2).max(axis=1) > 1e-3
    out["formation_rmse"] = float(np.sqrt(np.mean(err[moving] ** 2))) if np.any(moving) else float("nan")
    out["max_speed"] = float(speed.max())
    out["max_accel"] = float(acc.max())

    if coverage is not None:
        out["coverage"] = float(coverage["actual"])
        out["coverage_plan"] = float(coverage["plan"])
        out["coverage_loss"] = float(coverage["plan"] - coverage["actual"])

    sm = log.solve_ms[~np.isnan(log.solve_ms)]
    out["solve_ms_mean"] = float(sm.mean()) if sm.size else float("nan")
    out["solve_ms_p99"] = float(np.percentile(sm, 99)) if sm.size else float("nan")
    if iter_hist is not None and iter_hist.size:
        out["iters_mean"] = float(iter_hist.mean())
        out["iters_p95"] = float(np.percentile(iter_hist, 95))
        out["iters_max"] = float(iter_hist.max())
    out["urgent_steps"] = float(np.sum(log.urgent > 0))
    out["fallback_steps"] = float(np.sum(log.failed))
    for k, v in log.extra.items():
        out[k] = float(v)
    return out


METRICS = [
    ("min_inter_distance", "m"), ("pair_violation", "steps"),
    ("min_obs_surface", "m"), ("obs_clearance_violation", "steps"), ("obs_contact", "steps"),
    ("max_deviation", "m"), ("T_recovery_max", "s"), ("formation_rmse", "m"),
    ("coverage", "ratio"), ("coverage_plan", "ratio"), ("coverage_loss", "ratio"),
    ("max_speed", "m/s"), ("max_accel", "m/s2"),
    ("solve_ms_mean", "ms"), ("solve_ms_p99", "ms"),
    ("iters_mean", ""), ("iters_p95", ""), ("iters_max", ""),
    ("unsolved_ratio", ""), ("filtered_ratio", ""), ("resets", ""), ("urgent_steps", "steps"), ("fallback_steps", "steps"),
]
