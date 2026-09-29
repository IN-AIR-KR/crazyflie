"""공식 tinympc 파이썬 패키지가 명시한 QP 의 최적해를 내는지 확인한다.

    pip install tinympc     # 0.0.8 에서 확인
    python3 tools/check_tinympc.py

Stage 1 에서 공식 패키지 대신 tinyadmm.py 를 쓰게 된 근거를 재현한다.
같은 문제(이중적분기, N=30, Q=diag(10,10,1,1), R=0.1I, 참조 px=1)를
  (a) 정확한 LQR (무한 호라이즌 이득 = 긴 호라이즌 QP 의 첫 입력)
  (b) 공식 tinympc (제약 없음 / 비활성 선형 제약 1줄 / 박스 ±∞ 명시)
  (c) tinyadmm.py
로 풀고 u0 를 비교한다. 반복 수는 5000 으로 강제해 '덜 풀려서'가 아님을 보인다.

2026-09-28 결과 (tinympc 0.0.8):
    LQR 기준 u0 ≈ 8.72,  tinyadmm 8.75 (종단 비용 P_f 차이만큼)
    tinympc  무제약 1.735, 비활성 선형 제약 1줄 추가 시 1.925
    → 고정점이 문제의 최적해가 아니고, 걸리지도 않는 제약이 해를 바꾼다.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import scipy.linalg as sl

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from model import double_integrator  # noqa: E402
from tinyadmm import AdmmSettings, TinyADMM  # noqa: E402

DT, N, RHO = 0.05, 30, 5.0
Q = np.array([10.0, 10.0, 1.0, 1.0])
R = np.array([0.1, 0.1])


def main() -> None:
    A, B = double_integrator(DT)
    xref = np.zeros((N, 4)); xref[:, 0] = 1.0
    P = sl.solve_discrete_are(A, B, np.diag(Q), np.diag(R))
    K = np.linalg.solve(np.diag(R) + B.T @ P @ B, B.T @ P @ A)
    print(f"LQR (정확)                         u0 = {(-K @ (np.zeros(4) - xref[0]))[0]:.3f}")

    st = AdmmSettings(rho=RHO, max_iter=5000, abs_pri_tol=1e-7, abs_dua_tol=1e-7,
                      shift_warm_start=False)
    mine = TinyADMM(A, B, Q, R, N, 1, st, 1e3, 1e3, [(2, 3)], [(0, 1)])
    print(f"tinyadmm.py                        u0 = {mine.solve(np.zeros((1, 4)), xref[None])['u'][0, 0, 0]:.3f}")

    try:
        import tinympc
    except ImportError:
        print("tinympc 미설치 — pip install tinympc 후 다시 실행")
        return
    from importlib.metadata import version
    ver = version("tinympc")
    for label, lin, box in [("무제약", False, False),
                            ("비활성 선형 제약 1줄 (px ≤ 100)", True, False),
                            ("박스 ±1e17 명시", False, True)]:
        m = tinympc.TinyMPC()
        m.setup(A, B, np.diag(Q), np.diag(R), N, rho=RHO, max_iter=5000,
                abs_pri_tol=0.0, abs_dua_tol=0.0)
        if box:
            big = 1e17
            m.set_bound_constraints(np.full(4, -big), np.full(4, big), np.full(2, -big), np.full(2, big))
        if lin:
            m.set_linear_constraints(np.array([[1.0, 0, 0, 0]]), np.array([100.0]),
                                     np.zeros((0, 2)), np.zeros(0))
        m.set_x0(np.zeros(4))
        m.set_x_ref(xref.T.copy())
        u0 = m.solve()["controls"][0]
        print(f"tinympc {ver} [{label:28s}] u0 = {u0:.3f}")


if __name__ == "__main__":
    main()
