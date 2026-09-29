"""그래프 생성 — 두 제어기를 나란히 놓고 본다.

축 라벨은 영어로 둔다. 컨테이너에 한글 폰트가 없어 한글을 쓰면 □ 로 깨진다.
"""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # PNG 로만 저장한다. X11 불필요.

import matplotlib.pyplot as plt
import numpy as np

from metrics import SimLog, obstacle_distances, pairwise_distances
from scenario import Phase

PHASE_COLOR = {
    "Formation": "#e8f0fb",
    "Forward": "#eef7ea",
    "Obstacle": "#fdecea",
    "Recovery": "#f4eefa",
}
AGENT_COLOR = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd",
               "#ff7f0e", "#8c564b", "#17becf", "#7f7f7f"]


def _shade(ax, phases: list[Phase], label: bool = False) -> None:
    for ph in phases:
        ax.axvspan(ph.t_start, ph.t_end, color=PHASE_COLOR.get(ph.name, "#eeeeee"),
                   zorder=0, label=ph.name if label else None)


def _color(i: int) -> str:
    return AGENT_COLOR[i % len(AGENT_COLOR)]


def plot_trajectory(logs: dict[str, SimLog], phases: list[Phase], path: str,
                    R_p: float, r_o: float) -> None:
    names = list(logs)
    fig, axes = plt.subplots(1, len(names), figsize=(6.0 * len(names), 9.5), squeeze=False)
    for ax, name in zip(axes[0], names):
        log = logs[name]
        m = log.states.shape[1]
        d_obs = obstacle_distances(log.states, log.obstacle)
        detected = np.nanmin(d_obs, axis=1) <= R_p
        detected = np.where(np.isnan(np.nanmin(d_obs, axis=1)), False, detected)
        for i in range(m):
            ax.plot(log.states[:, i, 0], log.states[:, i, 1], color=_color(i), lw=1.2,
                    label=f"agent {i+1}")
            if np.any(detected):  # 회피 구간만 굵게
                ax.plot(log.states[detected, i, 0], log.states[detected, i, 1],
                        color=_color(i), lw=3.0, alpha=0.6)
            ax.plot(log.states[0, i, 0], log.states[0, i, 1], "o", color=_color(i), ms=5)
            ax.plot(log.refs[:, i, 0], log.refs[:, i, 1], color=_color(i), lw=0.6,
                    ls=":", alpha=0.8)
        ok = ~np.isnan(log.obstacle[:, 0])
        if np.any(ok):
            ax.plot(log.obstacle[ok, 0], log.obstacle[ok, 1], "k--", lw=1.0,
                    label="obstacle path")
            for idx in np.where(ok)[0][::40]:
                ax.add_patch(plt.Circle(log.obstacle[idx], r_o, color="k", alpha=0.18))
        ax.set_title(f"{name}  (dotted = formation reference)")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="upper right")
    fig.suptitle("XY trajectory")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_inter_distance(logs: dict[str, SimLog], phases: list[Phase], path: str,
                        R_c: float) -> None:
    names = list(logs)
    fig, axes = plt.subplots(len(names), 1, figsize=(10.0, 3.2 * len(names)),
                             squeeze=False, sharex=True)
    for ax, name in zip(axes[:, 0], names):
        log = logs[name]
        _shade(ax, phases, label=(name == names[0]))
        d = pairwise_distances(log.states)
        for c in range(d.shape[1]):
            ax.plot(log.t, d[:, c], lw=0.9)
        ax.axhline(R_c, color="r", ls="--", lw=1.2, label=f"$R_c$ = {R_c} m")
        ax.set_ylabel("inter-agent dist [m]")
        ax.set_title(name)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=3, loc="upper right")
    axes[-1, 0].set_xlabel("t [s]")
    fig.suptitle("Inter-agent distance (Qi Fig. 14(b))")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_velocity(logs: dict[str, SimLog], phases: list[Phase], path: str,
                  v_max: float) -> None:
    names = list(logs)
    fig, axes = plt.subplots(len(names), 1, figsize=(10.0, 3.2 * len(names)),
                             squeeze=False, sharex=True)
    for ax, name in zip(axes[:, 0], names):
        log = logs[name]
        _shade(ax, phases, label=(name == names[0]))
        spd = np.linalg.norm(log.states[:, :, 2:], axis=2)
        for i in range(spd.shape[1]):
            ax.plot(log.t, spd[:, i], color=_color(i), lw=0.9, label=f"agent {i+1}")
        ax.axhline(v_max, color="r", ls="--", lw=1.2, label=f"$v_{{max}}$ = {v_max} m/s")
        ax.set_ylabel("speed [m/s]")
        ax.set_title(name)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=3, loc="upper right")
    axes[-1, 0].set_xlabel("t [s]")
    fig.suptitle("Speed (Qi Fig. 14(a))")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_formation_error(logs: dict[str, SimLog], phases: list[Phase], path: str,
                         eps: float) -> None:
    names = list(logs)
    fig, axes = plt.subplots(len(names), 1, figsize=(10.0, 3.2 * len(names)),
                             squeeze=False, sharex=True)
    for ax, name in zip(axes[:, 0], names):
        log = logs[name]
        _shade(ax, phases, label=(name == names[0]))
        err = np.linalg.norm(log.states[:, :, :2] - log.refs[:, :, :2], axis=2)
        for i in range(err.shape[1]):
            ax.plot(log.t, err[:, i], color=_color(i), lw=0.9, label=f"agent {i+1}")
        ax.axhline(eps, color="r", ls="--", lw=1.0, label=f"$\\epsilon$ = {eps} m")
        ax.set_ylabel(r"$\|p_i - p_{ci}\|$ [m]")
        ax.set_title(name)
        ax.set_yscale("log")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, ncol=3, loc="upper right")
    axes[-1, 0].set_xlabel("t [s]")
    fig.suptitle("Formation tracking error")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_solve_time(log: SimLog, path: str, dt: float) -> None:
    ok = ~np.isnan(log.solve_ms)
    if not np.any(ok):
        return
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 3.6))
    axes[0].hist(log.solve_ms[ok], bins=50, color="#1f77b4")
    axes[0].axvline(dt * 1e3, color="r", ls="--",
                    label=f"control period = {dt*1e3:.0f} ms")
    axes[0].set_xlabel("solve time [ms]")
    axes[0].set_ylabel("count")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[1].plot(log.t[ok], log.solve_ms[ok], lw=0.7)
    axes[1].axhline(dt * 1e3, color="r", ls="--")
    axes[1].set_xlabel("t [s]")
    axes[1].set_ylabel("solve time [ms]")
    axes[1].grid(alpha=0.3)
    fig.suptitle("MPC solve time (OSQP)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
