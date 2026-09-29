"""Stage 1 진입점 — TinyMPC 군집 제어의 시뮬레이션 검증.

    python3 main.py                                   Crazyflie 스윕 + 장애물, 전 제어기
    python3 main.py --obstacles none                  장애물 없이 (커버리지 기준)
    python3 main.py --controllers qi dist             일부만
    python3 main.py --profile qi14                    Qi Fig.14 재현 (Stage 0 과 같은 조건)
    python3 main.py --seeds 0 1 2 3 4 --no-plot       여러 시드 통계

제어기
    qi            Qi et al. 식 (10) 가중합 (baseline)
    dist          분산 TinyMPC (기체별, 이동 BVC)            ← 제안
    central       중앙집중 TinyMPC (같은 솔버·같은 제약 기하)
    dist-static   분산 TinyMPC, 정적 BVC (v̄=0)               ← 절제 실험
    osqp          Stage 0 의 중앙집중 OSQP MPC (qi14 프로파일만)
"""
from __future__ import annotations

import argparse
import csv
import os
from dataclasses import replace
from typing import Any

import numpy as np

import metrics as mt
from baselines import Qi14Adapter, Qi14Scenario, QiGains, QiMulti, qi_gains_for
from controller_tiny import TinyCentralMPC, TinyConfig, TinyDistributedMPC
from model import step
from scenario_sweep import SweepConfig, SweepScenario

# ------------------------------------------------------------------ 프로파일
# cf   : 실내 Crazyflie 규모. 값의 근거는 README 의 "파라미터" 절.
# qi14 : Stage 0 (main.py DEFAULTS) 와 같은 값 — Qi 논문 실외 규모.
PROFILES: dict[str, dict[str, Any]] = {
    "cf": dict(
        tiny=TinyConfig(),
        qi=qi_gains_for(r_act=0.45, R_p=1.5, R_o=1.0, u_max=2.0),
        eps_recovery=0.1,
        controllers=["qi", "dist", "central", "dist-static"],
    ),
    "qi14": dict(
        tiny=TinyConfig(horizon=21, u_max=3.0, v_max=2.0, r_safe=1.8, r_agent=0.0,
                        margin=0.2, R_p=5.0, R_comm=1e9),
        qi=QiGains(R_c=1.8, R_p=5.0, u_max=3.0),
        eps_recovery=0.3,
        controllers=["qi", "osqp", "dist", "central"],
    ),
}


def make_scenario(profile: str, args) -> Any:
    if profile == "cf":
        return SweepScenario(SweepConfig(n_agents=args.agents, seed=args.seed,
                                         obstacles=args.obstacles))
    sc = Qi14Scenario(n_agents=args.agents, seed=args.seed,
                      spawn_min_sep=max(1.8 + 0.7, 2.0))
    return Qi14Adapter(sc)


def make_controller(kind: str, sc, prof: dict[str, Any], dt: float):
    tiny: TinyConfig = replace(prof["tiny"], dt=dt)
    if kind == "qi":
        return QiMulti(sc, prof["qi"], dt)
    if kind == "dist":
        return TinyDistributedMPC(sc, tiny)
    if kind == "dist-static":
        c = TinyDistributedMPC(sc, replace(tiny, moving_bvc=False))
        c.name = "dist-static"
        return c
    if kind == "central":
        return TinyCentralMPC(sc, tiny)
    if kind == "osqp":
        if not isinstance(sc, Qi14Adapter):
            raise ValueError("osqp 는 Stage 0 시나리오(qi14)에서만 돈다")
        from controller_mpc import CentralizedMPC, MpcConfig   # stage0
        c = CentralizedMPC(sc.inner, MpcConfig(horizon=tiny.horizon - 1, dt=dt,
                                                  u_max=tiny.u_max, v_max=tiny.v_max,
                                                  R_c=tiny.r_safe, r_o=sc.inner.obs_radius,
                                                  margin=tiny.margin), sc.n_agents)
        c.name = "osqp"
        return c
    raise ValueError(f"알 수 없는 제어기: {kind}")


def obstacle_track(sc, t: float, k_total: int) -> np.ndarray:
    """(K, 2) 참 장애물 위치 — 아직 없으면 nan. 지표·그래프용."""
    out = np.full((k_total, 2), np.nan)
    if isinstance(sc, SweepScenario):
        for k, o in enumerate(sc.obstacle_list):
            p = o.position(t)
            if p is not None:
                out[k] = p
    else:
        p, _, _ = sc.obstacles(t)
        out[:len(p)] = p
    return out


def simulate(ctrl, sc, dt: float) -> mt.SimLog:
    steps = int(round(sc.duration / dt))
    m = sc.n_agents
    k_total = len(sc.obstacle_list) if isinstance(sc, SweepScenario) else 1
    obs_rad = (np.array([o.radius for o in sc.obstacle_list]) if isinstance(sc, SweepScenario)
               else np.array([sc.inner.obs_radius]))
    T = np.zeros(steps)
    X = np.zeros((steps, m, 4))
    Rf = np.zeros((steps, m, 4))
    U = np.zeros((steps, m, 2))
    O = np.full((steps, k_total, 2), np.nan)
    ms, it, urg = (np.full(steps, np.nan) for _ in range(3))
    fail = np.zeros(steps, dtype=bool)
    plans = []

    x = sc.initial_states()
    print(f"[{ctrl.name}] {steps} 스텝 ({sc.duration:.1f}s @ {1/dt:.0f}Hz)")
    for n in range(steps):
        t = n * dt
        T[n], X[n], Rf[n] = t, x, sc.reference(t)
        O[n] = obstacle_track(sc, t, k_total)
        u = ctrl.step(x, t)
        U[n] = u
        ms[n], it[n] = ctrl.solve_ms, ctrl.iterations
        # osqp(Stage 0)의 slack 은 m 단위 슬랙이라 '긴급 제약 수'와 뜻이 달라 기록하지 않는다
        urg[n] = 0.0 if ctrl.name in ("osqp", "qi") else ctrl.slack
        fail[n] = bool(ctrl.failed)
        plan = getattr(ctrl, "plan", None)
        plans.append(None if plan is None else np.asarray(plan)[:, :, :2].copy())
        x = step(x, u, dt)

    log = mt.SimLog(ctrl.name, T, X, Rf, U, O, obs_rad, ms, it, urg, fail)
    if hasattr(ctrl, "iter_hist"):
        log.extra["unsolved_ratio"] = ctrl.unsolved / max(np.size(ctrl.iter_hist), 1)
        log.extra["resets"] = ctrl.resets
        log.extra["filtered_ratio"] = ctrl.filtered / max(np.size(ctrl.iter_hist), 1)
    log.plans = plans
    log.iter_hist = np.concatenate(ctrl.iter_hist) if getattr(ctrl, "iter_hist", None) else None
    return log


def evaluate(log: mt.SimLog, sc, prof: dict[str, Any]) -> tuple[dict[str, float], dict]:
    tiny: TinyConfig = prof["tiny"]
    cov = None
    extra: dict = {}
    if isinstance(sc, SweepScenario):
        grid = mt.CoverageGrid(sc.region)
        r = sc.cfg.sensor_radius
        ts, ratio, mask = mt.coverage_series(log, grid, r)
        plan_log = mt.SimLog("plan", log.t, log.refs, log.refs, log.inputs, log.obs_pos,
                             log.obs_rad, log.solve_ms, log.iterations, log.urgent, log.failed)
        _, ratio_plan, mask_plan = mt.coverage_series(plan_log, grid, r)
        cov = {"actual": ratio[-1], "plan": ratio_plan[-1]}
        extra = {"grid": grid, "cov_t": ts, "cov_ratio": ratio, "mask": mask, "mask_plan": mask_plan}
    clearance = tiny.r_agent + tiny.margin
    res = mt.compute(log, r_safe=tiny.r_safe, r_agent=tiny.r_agent, clearance=clearance,
                     R_p=tiny.R_p, eps_recovery=prof["eps_recovery"], coverage=cov,
                     iter_hist=log.iter_hist)
    return res, extra


def fmt(v: float) -> str:
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return "-"
    if np.isinf(v):
        return "inf"
    if abs(v - round(v)) < 1e-9 and abs(v) < 1e6:
        return f"{int(round(v))}"
    return f"{v:.3f}"


def print_table(results: dict[str, dict[str, float]]) -> None:
    names = list(results)
    w = max(len(k) for k, _ in mt.METRICS) + 2
    head = f"{'metric':<{w}}{'unit':<7}" + "".join(f"{n:>13}" for n in names)
    print("\n" + "=" * len(head) + "\n" + head + "\n" + "-" * len(head))
    for key, unit in mt.METRICS:
        vals = [results[n].get(key, float("nan")) for n in names]
        if all(isinstance(v, float) and np.isnan(v) for v in vals):
            continue
        print(f"{key:<{w}}{unit:<7}" + "".join(f"{fmt(v):>13}" for v in vals))
    print("=" * len(head))


def write_csv(rows: list[dict[str, Any]], path: str) -> None:
    keys = ["profile", "obstacles", "seed", "controller"] + [k for k, _ in mt.METRICS]
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> None:
    p = argparse.ArgumentParser(description="Stage 1 — TinyMPC 군집 스윕 커버리지 + 동적 장애물")
    p.add_argument("--profile", choices=list(PROFILES), default="cf")
    p.add_argument("--controllers", nargs="+", default=None)
    p.add_argument("--obstacles", default="mixed",
                   choices=["none", "crossing", "headon", "static", "mixed"])
    p.add_argument("--agents", type=int, default=4)
    p.add_argument("--seeds", type=int, nargs="+", default=[0])
    p.add_argument("--dt", type=float, default=0.05)
    p.add_argument("--outdir", default="results")
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args()

    prof = PROFILES[args.profile]
    kinds = args.controllers or prof["controllers"]
    os.makedirs(args.outdir, exist_ok=True)
    rows: list[dict[str, Any]] = []

    for seed in args.seeds:
        args.seed = seed
        logs, results, extras = {}, {}, {}
        for kind in kinds:
            sc = make_scenario(args.profile, args)      # 제어기마다 새로 — 상태 공유 방지
            ctrl = make_controller(kind, sc, prof, args.dt)
            log = simulate(ctrl, sc, args.dt)
            res, extra = evaluate(log, sc, prof)
            extra["summary"] = res
            logs[ctrl.name], results[ctrl.name], extras[ctrl.name] = log, res, extra
            rows.append({"profile": args.profile, "obstacles": args.obstacles, "seed": seed,
                         "controller": ctrl.name, **res})
        print(f"\n--- profile={args.profile} obstacles={args.obstacles} seed={seed}")
        print_table(results)
        if not args.no_plot and seed == args.seeds[0]:
            import plots   # matplotlib 은 그래프를 그릴 때만 불러온다
            sc = make_scenario(args.profile, args)
            plots.save_all(logs, extras, sc, prof, args.outdir)
            print(f"그래프 저장: {args.outdir}/*.png")

    path = os.path.join(args.outdir, f"metrics_{args.profile}_{args.obstacles}.csv")
    write_csv(rows, path)
    print(f"지표 저장: {path}")
    if len(args.seeds) > 1:
        summarize(rows)


def summarize(rows: list[dict[str, Any]]) -> None:
    """시드별 결과를 제어기마다 평균/최악으로 요약."""
    names = sorted({r["controller"] for r in rows}, key=lambda n: [r["controller"] for r in rows].index(n))
    keys = [("min_inter_distance", min), ("min_obs_surface", min), ("pair_violation", max),
            ("obs_clearance_violation", max), ("coverage", np.mean), ("coverage_loss", max),
            ("formation_rmse", np.mean), ("iters_p95", max), ("fallback_steps", max)]
    print("\n=== 시드 요약 (min/max 는 최악값, mean 은 평균) ===")
    print(f"{'metric':<26}{'agg':<6}" + "".join(f"{n:>13}" for n in names))
    for k, agg in keys:
        vals = []
        for n in names:
            v = [r[k] for r in rows if r["controller"] == n and k in r and not np.isnan(r[k])]
            vals.append(float(agg(v)) if v else float("nan"))
        print(f"{k:<26}{agg.__name__:<6}" + "".join(f"{fmt(v):>13}" for v in vals))


if __name__ == "__main__":
    main()
