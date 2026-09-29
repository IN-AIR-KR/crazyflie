"""그래프 — 제어기 비교. main.py 가 --no-plot 이 아닐 때만 불러온다."""
from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

import metrics as mt  # noqa: E402

AGENT_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf", "#8c564b", "#e377c2"]
CTRL_COLORS = {"qi": "#7f7f7f", "dist": "#1f77b4", "tiny-dist": "#1f77b4",
               "central": "#d62728", "tiny-central": "#d62728",
               "dist-static": "#9467bd", "osqp": "#2ca02c", "mpc": "#2ca02c"}


def _c(name: str) -> str:
    return CTRL_COLORS.get(name, "#333333")


def _shade(ax, sc) -> None:
    for ph in sc.phases():
        if ph.name in ("Obstacle",):
            ax.axvspan(ph.t_start, ph.t_end, color="orange", alpha=0.08)


def plot_overview(logs, extras, sc, path: str) -> None:
    """제어기마다: 커버리지 격자 + 궤적 + 장애물 경로."""
    names = list(logs)
    fig, axes = plt.subplots(1, len(names), figsize=(4.6 * len(names), 5.6), squeeze=False)
    for ax, name in zip(axes[0], names):
        log, ex = logs[name], extras[name]
        if "mask" in ex:
            g = ex["grid"]
            ext = (g.xs[0], g.xs[-1], g.ys[0], g.ys[-1])
            # 계획 대비 못 덮은 칸(빨강)과 덮은 칸(연회색)
            img = np.zeros(ex["mask"].shape + (4,))
            img[ex["mask"]] = (0.75, 0.85, 0.95, 1.0)
            lost = ex["mask_plan"] & ~ex["mask"]
            img[lost] = (0.95, 0.35, 0.35, 1.0)
            ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
            x0, x1, y0, y1 = sc.region
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=1.0, ls="--", color="k"))
        for i in range(log.states.shape[1]):
            ax.plot(log.states[:, i, 0], log.states[:, i, 1], lw=1.0, color=AGENT_COLORS[i % 8])
            ax.plot(log.states[0, i, 0], log.states[0, i, 1], "o", ms=3, color=AGENT_COLORS[i % 8])
        for k in range(log.obs_pos.shape[1]):
            tr = log.obs_pos[:, k]
            ok = ~np.isnan(tr[:, 0])
            if not np.any(ok):
                continue
            ax.plot(tr[ok, 0], tr[ok, 1], ":", color="k", lw=1.0)
            # 기체와 가장 가까웠던 순간의 장애물 위치
            d = np.linalg.norm(log.states[:, :, :2] - tr[:, None], axis=2).min(axis=1)
            d = np.where(ok, d, np.inf)
            j = int(np.argmin(d))
            ax.add_patch(Circle(tr[j], log.obs_rad[k], color="k", alpha=0.35))
        res = ex.get("summary", {})
        title = name
        if "coverage" in res:
            title += f"\ncoverage {100*res['coverage']:.1f}% (plan {100*res['coverage_plan']:.1f}%)"
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        # 장애물 경로가 멀리까지 뻗으므로 기체가 다닌 범위로 자른다
        P = log.states[:, :, :2].reshape(-1, 2)
        lo, hi = P.min(axis=0) - 0.8, P.max(axis=0) + 0.8
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
    axes[0][0].set_ylabel("y [m]")
    fig.suptitle("trajectories / coverage (red = planned but missed)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_safety(logs, sc, r_safe: float, clearance: float, path: str) -> None:
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for name, log in logs.items():
        dp = mt.pair_distances(log.states)
        a1.plot(log.t, dp.min(axis=1), color=_c(name), lw=1.2, label=name)
        ds = mt.obstacle_surface_distance(log).min(axis=(1, 2))
        a2.plot(log.t, np.minimum(ds, 3.0), color=_c(name), lw=1.2, label=name)
    a1.axhline(r_safe, color="k", ls="--", lw=1, label=f"r_safe={r_safe}")
    a2.axhline(clearance, color="k", ls="--", lw=1, label=f"clearance={clearance:.2f}")
    a2.axhline(0.0, color="r", ls=":", lw=1)
    a1.set_ylabel("min inter-agent distance [m]")
    a2.set_ylabel("min distance to obstacle surface [m]\n(clipped at 3)")
    a2.set_xlabel("t [s]")
    a1.legend(fontsize=8, ncol=5)
    a2.legend(fontsize=8, ncol=5)
    a1.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_tracking(logs, extras, eps: float, path: str) -> None:
    has_cov = any("cov_t" in e for e in extras.values())
    fig, axes = plt.subplots(2 if has_cov else 1, 1, figsize=(10, 6.5 if has_cov else 3.5),
                             sharex=True, squeeze=False)
    a1 = axes[0][0]
    for name, log in logs.items():
        err = np.linalg.norm(log.states[:, :, :2] - log.refs[:, :, :2], axis=2).max(axis=1)
        a1.plot(log.t, err, color=_c(name), lw=1.2, label=name)
    a1.axhline(eps, color="k", ls="--", lw=1, label=f"recovery eps={eps}")
    a1.set_ylabel("max formation error [m]")
    a1.set_yscale("symlog", linthresh=0.05)
    a1.legend(fontsize=8, ncol=5)
    if has_cov:
        a2 = axes[1][0]
        for name, ex in extras.items():
            if "cov_t" in ex:
                a2.plot(ex["cov_t"], 100 * ex["cov_ratio"], color=_c(name), lw=1.2, label=name)
        a2.set_ylabel("coverage [%]")
        a2.legend(fontsize=8, ncol=5)
    axes[-1][0].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_solver(logs, path: str) -> None:
    mpcs = {n: l for n, l in logs.items() if getattr(l, "iter_hist", None) is not None}
    if not mpcs:
        return
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 3.8))
    for name, log in mpcs.items():
        a1.plot(log.t, log.iterations, color=_c(name), lw=0.8, label=name)
        a2.hist(log.iter_hist, bins=np.arange(0, np.max(log.iter_hist) + 6, 5), histtype="step",
                color=_c(name), lw=1.5, label=name, density=True)
    a1.set_xlabel("t [s]")
    a1.set_ylabel("ADMM iterations (max over agents)")
    a2.set_xlabel("iterations per solve (per agent)")
    a2.set_ylabel("density")
    a1.legend(fontsize=8)
    a2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_snapshots(log, sc, path: str, n_snap: int = 4) -> None:
    """장애물과 가장 가까웠던 순간 전후의 예측 궤적 — split(갈라짐)과 merge(다시 모임)."""
    if getattr(log, "plans", None) is None or log.obs_pos.shape[1] == 0:
        return
    ds = mt.obstacle_surface_distance(log)            # (T, M, K)
    events = []
    for k in range(ds.shape[2]):
        dk = ds[:, :, k].min(axis=1)
        if np.isfinite(dk).any():
            events.append((k, int(np.argmin(dk))))
    if not events:
        return
    dt = log.t[1] - log.t[0]
    offs = np.array([-2.0, -0.8, 0.0, 1.5])[:n_snap]
    fig, axes = plt.subplots(len(events), len(offs), figsize=(3.3 * len(offs), 3.3 * len(events)),
                             squeeze=False)
    for r, (k, j0) in enumerate(events):
        for c, off in enumerate(offs):
            ax = axes[r][c]
            j = int(np.clip(j0 + off / dt, 0, len(log.t) - 1))
            P = log.states[j, :, :2]
            plan = log.plans[j]
            for i in range(P.shape[0]):
                col = AGENT_COLORS[i % 8]
                ax.plot(*P[i], "o", color=col, ms=5)
                ax.plot(log.refs[j, i, 0], log.refs[j, i, 1], "x", color=col, ms=5)
                if plan is not None:
                    ax.plot(plan[i, :, 0], plan[i, :, 1], "-", color=col, lw=1.0, alpha=0.8)
            for kk in range(log.obs_pos.shape[1]):
                o = log.obs_pos[j, kk]
                if np.isnan(o[0]):
                    continue
                ax.add_patch(Circle(o, log.obs_rad[kk], color="k", alpha=0.35))
                o2 = log.obs_pos[min(j + 20, len(log.t) - 1), kk]
                ax.annotate("", xy=o2, xytext=o, arrowprops=dict(arrowstyle="->", color="k", lw=0.8))
            cen = log.refs[j, :, :2].mean(axis=0)
            ax.set_xlim(cen[0] - 1.6, cen[0] + 1.6)
            ax.set_ylim(cen[1] - 1.6, cen[1] + 1.6)
            ax.set_aspect("equal")
            ax.set_title(f"obs {k}  t={log.t[j]:.1f}s ({off:+.1f})", fontsize=9)
            ax.tick_params(labelsize=7)
    fig.suptitle(f"{log.name}: o = agent, x = formation reference, line = MPC plan", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def save_all(logs, extras, sc, prof, outdir: str) -> None:
    tiny = prof["tiny"]
    plot_overview(logs, extras, sc, os.path.join(outdir, "overview.png"))
    plot_safety(logs, sc, tiny.r_safe, tiny.r_agent + tiny.margin, os.path.join(outdir, "safety.png"))
    plot_tracking(logs, extras, prof["eps_recovery"], os.path.join(outdir, "tracking.png"))
    plot_solver(logs, os.path.join(outdir, "solver.png"))
    for name, log in logs.items():
        if getattr(log, "plans", None) and log.plans[0] is not None:
            plot_snapshots(log, sc, os.path.join(outdir, f"snapshots_{name}.png"))
