"""Stage 0 진입점 — Qi 가중합 제어와 MPC 재정식화를 같은 시나리오에서 비교한다.

    python3 main.py                      두 제어기 모두
    python3 main.py --controller qi       baseline 만
    python3 main.py --controller mpc      MPC 만
    python3 main.py --agents 4 --horizon 20 --dt 0.05
    python3 main.py --no-plot             지표만

이 단계의 목적은 정식화 검증이다. MCU 실시간성은 다루지 않는다.
"""
from __future__ import annotations

import argparse
import csv
import os
from typing import Any, Protocol

import numpy as np

import metrics as mt
from controller_qi import QiController, QiGains
from dynamics import DoubleIntegrator2D
from scenario import Scenario

# 모든 파라미터를 여기 모아 둔다. CLI 로 덮어쓸 수 있다.
DEFAULTS: dict[str, Any] = {
    # --- 시뮬레이션
    "agents": 4,
    "horizon": 20,
    "dt": 0.05,
    "duration": 50.0,
    "seed": 0,
    "controller": "both",
    "outdir": "results",
    # --- 시나리오
    "side": 3.0,            # 정사각 대형의 한 변 L [m]
    "beta": 0.0,            # 대형 중심의 x 오프셋
    "v_forward": -1.0,      # Forward 단계의 -y 전진 속도 [m/s]
    "ramp": 1.0,            # 전진 속도 상승 시간 [s] (v̇_ci 를 유한하게 두려고)
    "t_form": 10.0,
    "t_fwd_end": 25.0,
    "t_obs_end": 35.0,
    "obs_speed": 1.0,
    "r_o": 0.5,             # 장애물 반지름 [m]
    "margin": 0.2,          # MPC 장애물 여유 [m]
    # --- 공통 한계
    "u_max": 3.0,           # [m/s^2]
    "v_max": 2.0,           # [m/s]  (MPC 만 제약으로 강제, Qi 는 고려 안 함)
    "R_c": 1.8,             # 기체 간 안전거리 [m]
    "R_p": 5.0,             # 장애물 감지 반경 [m]
    "eps_recovery": 0.3,    # 편대 복귀 판정 [m]
    # --- MPC 비용
    "Q": (10.0, 10.0, 1.0, 1.0),
    "R": (0.1, 0.1),
    "Qf_scale": 10.0,
    "slack_lin": 5.0e3,
    "slack_quad": 1.0e2,
}


class Controller(Protocol):
    name: str
    solve_ms: float
    iterations: float
    slack: float
    failed: bool

    def step(self, state: np.ndarray, t: float) -> np.ndarray: ...


def make_scenario(cfg: dict[str, Any]) -> Scenario:
    return Scenario(
        n_agents=cfg["agents"], side=cfg["side"], beta=cfg["beta"],
        v_forward=cfg["v_forward"], t_form=cfg["t_form"], t_fwd_end=cfg["t_fwd_end"],
        t_obs_end=cfg["t_obs_end"], duration=cfg["duration"], ramp=cfg["ramp"],
        obs_speed=cfg["obs_speed"], obs_radius=cfg["r_o"], seed=cfg["seed"],
        spawn_min_sep=max(cfg["R_c"] + 0.7, 2.0),
    )


def make_controller(kind: str, sc: Scenario, cfg: dict[str, Any]) -> Controller:
    if kind == "qi":
        gains = QiGains(R_c=cfg["R_c"], R_p=cfg["R_p"], u_max=cfg["u_max"])
        return QiController(sc, gains, cfg["dt"])
    if kind == "mpc":
        # osqp/scipy 가 없는 환경에서도 --controller qi 는 돌아가야 하므로 늦게 import 한다.
        from controller_mpc import CentralizedMPC, MpcConfig

        mcfg = MpcConfig(
            horizon=cfg["horizon"], dt=cfg["dt"], Q=cfg["Q"], R=cfg["R"],
            Qf_scale=cfg["Qf_scale"], u_max=cfg["u_max"], v_max=cfg["v_max"],
            R_c=cfg["R_c"], r_o=cfg["r_o"], margin=cfg["margin"],
            slack_lin=cfg["slack_lin"], slack_quad=cfg["slack_quad"],
        )
        return CentralizedMPC(sc, mcfg, cfg["agents"])
    raise ValueError(f"알 수 없는 제어기: {kind}")


def simulate(controller: Controller, sc: Scenario, cfg: dict[str, Any]) -> mt.SimLog:
    """두 제어기가 같은 인터페이스를 갖기 때문에 이 루프를 공유할 수 있다."""
    dt = float(cfg["dt"])
    steps = int(round(cfg["duration"] / dt))
    dyn = DoubleIntegrator2D(dt)
    m = cfg["agents"]

    t_arr = np.zeros(steps)
    states = np.zeros((steps, m, 4))
    refs = np.zeros((steps, m, 4))
    inputs = np.zeros((steps, m, 2))
    obstacle = np.full((steps, 2), np.nan)
    solve_ms = np.full(steps, np.nan)
    iters = np.full(steps, np.nan)
    slack = np.full(steps, np.nan)
    failed = np.zeros(steps, dtype=bool)

    x = sc.initial_states()
    print(f"[{controller.name}] {steps} 스텝 ({cfg['duration']:.0f}s @ {1/dt:.0f}Hz) 시작")
    for n in range(steps):
        t = n * dt
        t_arr[n] = t
        states[n] = x
        refs[n] = sc.reference(t)
        obs = sc.obstacle_position(t)
        if obs is not None:
            obstacle[n] = obs
        u = controller.step(x, t)
        inputs[n] = u
        solve_ms[n] = controller.solve_ms
        iters[n] = controller.iterations
        slack[n] = controller.slack
        failed[n] = bool(controller.failed)
        x = dyn.step(x, u)
    return mt.SimLog(controller.name, t_arr, states, refs, inputs, obstacle,
                     solve_ms, iters, slack, failed)


def fmt(value: float) -> str:
    if value is None or (isinstance(value, float) and np.isnan(value)):
        return "-"
    if abs(value - round(value)) < 1e-9 and abs(value) < 1e6:
        return f"{int(round(value))}"
    return f"{value:.3f}"


def print_summary(results: dict[str, dict[str, float]], cfg: dict[str, Any]) -> None:
    names = list(results)
    width = max(len(k) for k in mt.METRIC_ORDER) + 2
    print("\n" + "=" * (width + 8 + 14 * len(names)))
    head = f"{'metric':<{width}}{'unit':<8}" + "".join(f"{n:>14}" for n in names)
    print(head)
    print("-" * len(head))
    for key in mt.METRIC_ORDER:
        if all(np.isnan(results[n].get(key, np.nan)) for n in names):
            continue
        row = f"{key:<{width}}{mt.METRIC_UNITS.get(key, ''):<8}"
        row += "".join(f"{fmt(results[n].get(key, float('nan'))):>14}" for n in names)
        print(row)
    print("=" * len(head))

    if len(names) == 2 and "qi" in results and "mpc" in results:
        qi, mpc = results["qi"], results["mpc"]
        nominal = cfg["side"]
        print("\n[가설 점검] 결과가 반대로 나와도 그대로 보고한다.")
        checks = [
            (f"Qi 회복 시간이 수 초 단위 (논문 5s)", qi["T_recovery"],
             lambda v: not np.isnan(v) and 0.5 <= v <= 20.0),
            (f"Qi 최대 이탈이 평상시 간격({nominal:.1f}m)의 2배 이상", qi["max_deviation"],
             lambda v: not np.isnan(v) and v >= 2.0 * nominal),
            ("MPC 기체 간 충돌 0", mpc["collision_count"], lambda v: v == 0),
            ("MPC 장애물 침범 0", mpc["obstacle_violation"], lambda v: v == 0),
            ("MPC 최대 이탈 < Qi 최대 이탈", mpc["max_deviation"],
             lambda v: not np.isnan(v) and not np.isnan(qi["max_deviation"])
             and v < qi["max_deviation"]),
        ]
        for label, value, ok in checks:
            mark = "OK  " if ok(value) else "FAIL"
            print(f"  [{mark}] {label}  (측정 {fmt(value)})")


def write_csv(results: dict[str, dict[str, float]], path: str) -> None:
    names = list(results)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["metric", "unit", *names])
        for key in mt.METRIC_ORDER:
            w.writerow([key, mt.METRIC_UNITS.get(key, ""),
                        *[results[n].get(key, "") for n in names]])


def parse_args() -> dict[str, Any]:
    p = argparse.ArgumentParser(description="Stage 0 — 군집 MPC 정식화 검증")
    p.add_argument("--controller", choices=["both", "qi", "mpc"],
                   default=DEFAULTS["controller"])
    p.add_argument("--agents", type=int, default=DEFAULTS["agents"])
    p.add_argument("--horizon", type=int, default=DEFAULTS["horizon"])
    p.add_argument("--dt", type=float, default=DEFAULTS["dt"])
    p.add_argument("--duration", type=float, default=DEFAULTS["duration"])
    p.add_argument("--seed", type=int, default=DEFAULTS["seed"])
    p.add_argument("--outdir", default=DEFAULTS["outdir"])
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args()
    cfg = dict(DEFAULTS)
    cfg.update({k: v for k, v in vars(args).items() if k != "no_plot"})
    cfg["no_plot"] = args.no_plot
    return cfg


def main() -> None:
    cfg = parse_args()
    os.makedirs(cfg["outdir"], exist_ok=True)
    kinds = ["qi", "mpc"] if cfg["controller"] == "both" else [cfg["controller"]]

    logs: dict[str, mt.SimLog] = {}
    results: dict[str, dict[str, float]] = {}
    for kind in kinds:
        sc = make_scenario(cfg)       # 제어기마다 새로 만들어 상태 공유를 막는다
        ctrl = make_controller(kind, sc, cfg)
        log = simulate(ctrl, sc, cfg)
        logs[kind] = log
        results[kind] = mt.compute(log, R_c=cfg["R_c"], R_p=cfg["R_p"],
                                   r_o=cfg["r_o"], eps_recovery=cfg["eps_recovery"])

    print_summary(results, cfg)
    csv_path = os.path.join(cfg["outdir"], "metrics.csv")
    write_csv(results, csv_path)
    print(f"\n지표 저장: {csv_path}")

    if cfg["no_plot"]:
        return
    import plots  # matplotlib 을 --no-plot 경로에서 아예 불러오지 않으려고 늦게 import

    sc = make_scenario(cfg)
    phases = sc.phases()
    out = cfg["outdir"]
    plots.plot_trajectory(logs, phases, os.path.join(out, "trajectory.png"),
                          R_p=cfg["R_p"], r_o=cfg["r_o"])
    plots.plot_inter_distance(logs, phases, os.path.join(out, "inter_distance.png"),
                              R_c=cfg["R_c"])
    plots.plot_velocity(logs, phases, os.path.join(out, "velocity.png"),
                        v_max=cfg["v_max"])
    plots.plot_formation_error(logs, phases, os.path.join(out, "formation_error.png"),
                               eps=cfg["eps_recovery"])
    if "mpc" in logs:
        plots.plot_solve_time(logs["mpc"], os.path.join(out, "solve_time.png"),
                              dt=cfg["dt"])
    print(f"그래프 저장: {out}/*.png")


if __name__ == "__main__":
    main()
