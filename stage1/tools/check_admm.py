"""tinyadmm.py 가 명시한 QP 를 정확히 푸는지 OSQP 로 검증한다.

    python3 tools/check_admm.py

같은 문제(비용, 종단 P_f, 입력·속도 팔각형, 시변 반공간)를 OSQP 로 정확하게 풀고
ADMM 해와 비교한다. 비교 실험에서 MPC 가 "자기 문제를 제대로 푼다"는 전제를
확인하는 용도다. 경우마다 u0 차이와 제약 위반량을 출력하고, 기준을 넘으면 FAIL.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import osqp
import scipy.sparse as sp

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from model import double_integrator  # noqa: E402
from tinyadmm import AdmmSettings, TinyADMM, _OCT_A, C8  # noqa: E402

DT, N = 0.05, 30
Q = np.array([10.0, 10.0, 1.0, 1.0])
R = np.array([0.1, 0.1])
U_MAX, V_MAX = 2.0, 1.0
RHO = float(os.environ.get("RHO", "20.0"))


def exact_qp(A, B, Pf, x0, xref, Ha, hb, u_max, v_max):
    """OSQP 로 정확한 해. 결정변수 [x_0..x_{N-1}, u_0..u_{N-2}]."""
    nx, nu = B.shape
    nX, nU = N * nx, (N - 1) * nu
    P = sp.block_diag([sp.diags(Q)] * (N - 1) + [sp.csc_matrix(Pf)] + [sp.diags(R)] * (N - 1), format="csc")
    qv = np.zeros(nX + nU)
    for k in range(N - 1):
        qv[k * nx:(k + 1) * nx] = -Q * xref[k]
    qv[(N - 1) * nx:nX] = -Pf @ xref[N - 1]
    rows, lo, up = [], [], []

    def row(entries):
        r = np.zeros(nX + nU)
        for idx, val in entries:
            r[idx] = val
        return r

    for c in range(nx):                                   # x0 고정
        rows.append(row([(c, 1.0)])); lo.append(x0[c]); up.append(x0[c])
    for k in range(N - 1):                                # 동역학
        for c in range(nx):
            e = [((k + 1) * nx + c, 1.0)]
            e += [(k * nx + j, -A[c, j]) for j in range(nx) if A[c, j] != 0]
            e += [(nX + k * nu + j, -B[c, j]) for j in range(nu) if B[c, j] != 0]
            rows.append(row(e)); lo.append(0.0); up.append(0.0)
    for k in range(N - 1):                                # 입력 팔각형
        for a in _OCT_A:
            rows.append(row([(nX + k * nu, a[0]), (nX + k * nu + 1, a[1])]))
            lo.append(-np.inf); up.append(u_max * C8)
    for k in range(N):                                    # 속도 팔각형 (k=0 제외)
        if k == 0:
            continue
        for a in _OCT_A:
            rows.append(row([(k * nx + 2, a[0]), (k * nx + 3, a[1])]))
            lo.append(-np.inf); up.append(v_max * C8)
    if Ha is not None:                                    # 시변 반공간 (k≥1)
        for k in range(1, N):
            for c in range(Ha.shape[1]):
                rows.append(row([(k * nx + j, Ha[k, c, j]) for j in range(nx)]))
                lo.append(-np.inf); up.append(hb[k, c])
    Aq = sp.csc_matrix(np.array(rows))
    prob = osqp.OSQP()
    prob.setup(P=P, q=qv, A=Aq, l=np.array(lo), u=np.array(up), eps_abs=1e-9, eps_rel=1e-9,
               max_iter=200000, polish=True, verbose=False)
    res = prob.solve()
    return res.x[nX:nX + nu], res.x[:nX].reshape(N, nx)


def main() -> int:
    A, B = double_integrator(DT)
    fails = 0
    cases = []
    xref = np.zeros((N, 4)); xref[:, 0] = 1.0
    cases.append(("무제약에 가까움 (한계 크게)", np.zeros(4), xref, None, None, 100.0, 100.0))
    cases.append(("입력·속도 팔각형 활성", np.zeros(4), xref, None, None, U_MAX, V_MAX))
    # 시변 반공간: 움직이는 벽  x ≤ 0.3 + 0.2·t
    t = np.arange(N) * DT
    Ha = np.zeros((N, 1, 4)); Ha[:, 0, 0] = 1.0
    hb = (0.3 + 0.2 * t)[:, None]
    cases.append(("시변 반공간 (움직이는 벽)", np.zeros(4), xref, Ha, hb, U_MAX, V_MAX))
    # 두 개의 반공간 + 대각 참조
    xref2 = np.zeros((N, 4)); xref2[:, 0] = 1.0; xref2[:, 1] = 1.0
    Ha2 = np.zeros((N, 2, 4)); Ha2[:, 0, 0] = 1.0; Ha2[:, 1, 0:2] = [-0.6, 0.8]
    hb2 = np.stack([0.5 + 0.1 * t, 0.2 + 0.0 * t], axis=1)
    cases.append(("반공간 2개 동시 활성", np.array([0, 0, 0.3, 0]), xref2, Ha2, hb2, U_MAX, V_MAX))

    for name, x0, xr, Ha_, hb_, um, vm in cases:
        st = AdmmSettings(rho=RHO, max_iter=5000, abs_pri_tol=1e-6, abs_dua_tol=1e-6,
                          dykstra_sweeps=10, shift_warm_start=False)
        slv = TinyADMM(A, B, Q, R, N, 1, st, um, vm, [(2, 3)], [(0, 1)])
        Hb = None if Ha_ is None else Ha_[None]
        hbb = None if hb_ is None else hb_[None]
        act = None if Ha_ is None else np.ones(hb_.shape, bool)[None]
        sol = slv.solve(x0[None], xr[None], Hb, hbb, act)
        u_ex, x_ex = exact_qp(A, B, slv.Pf, x0, xr, Ha_, hb_, um, vm)
        du = float(np.max(np.abs(sol["u"][0, 0] - u_ex)))
        dx = float(np.max(np.abs(sol["x"][0] - x_ex)))
        viol = 0.0
        if Ha_ is not None:
            viol = float(np.max(np.einsum("kcn,kn->kc", Ha_[1:], sol["x"][0, 1:]) - hb_[1:]))
        ok = du < 1e-2 and dx < 1e-2 and viol < 1e-3
        fails += int(not ok)
        print(f"[{'OK  ' if ok else 'FAIL'}] {name:28s} iters={int(sol['iters'][0]):5d} "
              f"u0 admm={np.round(sol['u'][0,0],4)} exact={np.round(u_ex,4)} "
              f"max|Δx|={dx:.2e} viol={viol:.1e}")
    return fails


if __name__ == "__main__":
    sys.exit(main())
