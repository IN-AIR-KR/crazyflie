"""SIL 로그 평가 — main.py 와 같은 지표·그래프로 Python 시뮬레이션과 비교한다.

    python3 sil/sil_eval.py sil/sil_log.npz [--outdir results_sil]

ROS 없이 돈다(numpy/matplotlib 만). 추가로 SIL 특유의 값을 출력한다:
실제 제어 주기(sim 시계)의 분포, 노드 계산 시간, 고도 유지 오차.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

import metrics as mt  # noqa: E402
from controller_tiny import TinyConfig  # noqa: E402
from main import fmt  # noqa: E402
from scenario_sweep import SweepConfig, SweepScenario  # noqa: E402


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("log")
    p.add_argument("--outdir", default=os.path.join(HERE, "..", "results_sil"))
    p.add_argument("--height", type=float, default=0.5)
    a = p.parse_args()
    d = np.load(a.log)
    name = f"sil-{str(d['controller'])}"
    log = mt.SimLog(name, d["t"], d["states"], d["refs"], d["inputs"], d["obs_pos"], d["obs_rad"],
                    d["solve_ms"], d["iterations"], d["urgent"], d["failed"].astype(bool))
    log.iter_hist = d["iter_hist"]
    n_solve = max(len(d["iter_hist"]), 1)
    log.extra = {"filtered_ratio": float(d["filtered"]) / n_solve,
                 "unsolved_ratio": float(d["unsolved"]) / n_solve,
                 "resets": float(d["resets"])}
    sc = SweepScenario(SweepConfig(n_agents=log.states.shape[1], obstacles=str(d["obstacles"])))
    tiny = TinyConfig()
    grid = mt.CoverageGrid(sc.region)
    ts, ratio, mask = mt.coverage_series(log, grid, sc.cfg.sensor_radius)
    plan = mt.SimLog("plan", log.t, log.refs, log.refs, log.inputs, log.obs_pos, log.obs_rad,
                     log.solve_ms, log.iterations, log.urgent, log.failed)
    _, ratio_p, mask_p = mt.coverage_series(plan, grid, sc.cfg.sensor_radius)
    res = mt.compute(log, r_safe=tiny.r_safe, r_agent=tiny.r_agent,
                     clearance=tiny.r_agent + tiny.margin, R_p=tiny.R_p, eps_recovery=0.1,
                     coverage={"actual": ratio[-1], "plan": ratio_p[-1]}, iter_hist=log.iter_hist)

    print(f"\n=== {name}  (obstacles={str(d['obstacles'])}, {len(log.t)} 주기) ===")
    for key, unit in mt.METRICS:
        if key in res:
            print(f"  {key:<26}{unit:<7}{fmt(res[key]):>10}")
    cdt = d["ctrl_dt"][~np.isnan(d["ctrl_dt"])]
    print(f"  {'ctrl_dt (sim) mean/p99':<26}{'s':<7}{cdt.mean():>10.4f} / {np.percentile(cdt, 99):.4f}"
          f"   (목표 {1/float(d['rate']):.4f})")
    print(f"  {'node wall ms mean/p99':<26}{'ms':<7}{d['wall_ms'].mean():>10.2f} / {np.percentile(d['wall_ms'], 99):.2f}")
    print(f"  {'altitude error max':<26}{'m':<7}{np.abs(d['z'] - a.height).max():>10.3f}")

    os.makedirs(a.outdir, exist_ok=True)
    import plots
    extras = {name: {"grid": grid, "cov_t": ts, "cov_ratio": ratio, "mask": mask,
                     "mask_plan": mask_p, "summary": res}}
    prof = {"tiny": tiny, "eps_recovery": 0.1}
    log.plans = None
    plots.save_all({name: log}, extras, sc, prof, a.outdir)
    print(f"그래프: {a.outdir}")


if __name__ == "__main__":
    main()
