"""TinyMPC 알고리즘의 numpy 구현 (배치 처리 + 시변 반공간).

알고리즘은 TinyMPC (Nguyen et al., ICRA 2024) 와 같다.
  * 오프라인: (Q+ρI, R+ρI) 로 무한 호라이즌 Riccati 를 풀어 Kinf, Pinf,
    Quu⁻¹, (A−BK)ᵀ 를 캐시한다. 온라인에서는 행렬 분해를 하지 않는다.
  * 온라인 ADMM 한 반복 = 전방 롤아웃 → 슬랙 투영 → 쌍대 갱신
    → 선형 비용 갱신 → 역방향 패스(벡터만). MCU 에서 수백 Hz 가 나오는 이유다.

공식 tinympc 파이썬 패키지(0.0.8) 대신 직접 구현한 이유는 stage1/README.md 의
"공식 패키지 검증" 절과 tools/check_tinympc.py 에 재현 스크립트로 남겼다.
요약: (1) 파이썬 API 에 시변 선형 제약이 없고, (2) 무제약 문제에서도 고정점이
명시한 QP 의 최적해와 달랐다(u0 1.735 vs 8.722). 비교 실험의 전제가 "MPC 가 자기
문제를 제대로 푼다"이므로, 결과를 OSQP 로 검증할 수 있는 구현을 쓴다.

이 구현이 푸는 문제 (배치 b 마다 독립):
    min  Σ_{k=0}^{N-2} ½‖x_k − x̄_k‖²_Q + ½‖u_k‖²_R  +  ½‖x_{N-1} − x̄_{N-1}‖²_{P_f}
    s.t. x_{k+1} = A x_k + B u_k,  x_0 고정
         H_k x_k ≤ h_k   (k = 1..N-1, 시변 반공간 — 회피 제약)
         ‖v_k‖ ≤ v_max, ‖u_k‖ ≤ u_max   (정팔각형, 배치 전체 공통)
    P_f = Pinf − ρI.  TinyMPC 는 종단 선형항을 −Pinf·x̄ 로 두는데, 그러면 고정점의
    종단 비용이 (Pinf − ρI) 가 되면서 목표점이 어긋난다. 여기서는 선형항을
    −(Pinf − ρI)·x̄ 로 맞춰 "고정점 = 위 QP 의 최적해"가 정확히 성립하게 했다.
    k=0 에는 상태 제약을 걸지 않는다(x0 는 측정값이라 바꿀 수 없다).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

C8 = np.cos(np.pi / 8.0)
_OCT_A = np.stack([np.cos(np.arange(8) * np.pi / 4.0), np.sin(np.arange(8) * np.pi / 4.0)], axis=1)


def project_octagon(w: np.ndarray, radius: float) -> np.ndarray:
    """정팔각형(꼭짓점이 반지름 radius 원 위)으로의 정확한 유클리드 투영. w: (..., 2).

    가장 많이 넘친 면에 투영한 뒤, 이웃 면을 넘으면 두 면이 만나는 꼭짓점으로 보낸다.
    """
    c = radius * C8
    viol = w @ _OCT_A.T - c                          # (..., 8)
    k = np.argmax(viol, axis=-1)
    vk = np.take_along_axis(viol, k[..., None], axis=-1)[..., 0]
    out = w - np.maximum(vk, 0.0)[..., None] * _OCT_A[k]
    # 이웃 면 검사 → 꼭짓점
    for dk in (1, -1):
        kn = (k + dk) % 8
        vn = np.einsum("...c,...c->...", out, _OCT_A[kn]) - c
        bad = (vk > 0.0) & (vn > 1e-12)
        if np.any(bad):
            ang = (k + 0.5 * dk) * (np.pi / 4.0)
            vert = radius * np.stack([np.cos(ang), np.sin(ang)], axis=-1)
            out = np.where(bad[..., None], vert, out)
    return out


def dykstra_halfspaces(z: np.ndarray, a: np.ndarray, b: np.ndarray, active: np.ndarray,
                       sweeps: int) -> np.ndarray:
    """반공간 교집합 {a_c'z ≤ b_c} 으로의 투영 (Dykstra). 배치 축은 앞쪽 전부.

    z: (..., n), a: (..., C, n), b/active: (..., C).
    반공간이 1~2개면 1~2 번 스윕으로 정확해지고, 그 이상은 근사다.
    ADMM 안에서 매 반복 호출되고 warm start 가 걸려 있어 근사 오차가 누적되지 않는다.
    """
    if a.shape[-2] == 0:
        return z
    nrm2 = np.maximum(np.einsum("...cn,...cn->...c", a, a), 1e-12)
    incr = np.zeros(a.shape)
    x = z
    for _ in range(sweeps):
        changed = False
        for c in range(a.shape[-2]):
            y = x + incr[..., c, :]
            viol = np.einsum("...n,...n->...", y, a[..., c, :]) - b[..., c]
            viol = np.where(active[..., c], np.maximum(viol, 0.0), 0.0)
            if np.any(viol > 0.0):
                changed = True
            x_new = y - (viol / nrm2[..., c])[..., None] * a[..., c, :]
            incr[..., c, :] = y - x_new
            x = x_new
        if not changed:
            break
    return x


@dataclass
class AdmmSettings:
    rho: float = 5.0
    max_iter: int = 100
    abs_pri_tol: float = 1e-3
    abs_dua_tol: float = 1e-3
    check_every: int = 5
    dykstra_sweeps: int = 3
    shift_warm_start: bool = True


class TinyADMM:
    """배치 b 개의 같은 모양 문제를 한꺼번에 푼다.

    분산형은 기체마다 A,B,Q,R 가 같으므로 b = M 으로 한 번에 돌리고(반복 수는 기체별로
    따로 센다), 중앙집중형은 b = 1 에 상태를 쌓아서 쓴다.
    pos_idx: 회피 반공간이 걸리는 위치 성분 인덱스, vel_idx: (기체별) 속도 성분 쌍.
    """

    def __init__(self, A: np.ndarray, B: np.ndarray, q_diag, r_diag, N: int, batch: int,
                 st: AdmmSettings, u_max: float, v_max: float,
                 vel_pairs: list[tuple[int, int]], u_pairs: list[tuple[int, int]]) -> None:
        self.A, self.B = np.asarray(A, float), np.asarray(B, float)
        self.nx, self.nu = self.B.shape
        self.N, self.nb, self.st = int(N), int(batch), st
        self.Q = np.asarray(q_diag, float)
        self.R = np.asarray(r_diag, float)
        self.u_max, self.v_max = float(u_max), float(v_max)
        self.vel_pairs = [list(p) for p in vel_pairs]
        self.u_pairs = [list(p) for p in u_pairs]
        self._cache()
        self.reset()

    # ------------------------------------------------------------ 오프라인 캐시
    def _cache(self) -> None:
        rho = self.st.rho
        a, b = self.A, self.B
        q1 = np.diag(self.Q) + rho * np.eye(self.nx)
        r1 = np.diag(self.R) + rho * np.eye(self.nu)
        p = q1.copy()
        k = np.zeros((self.nu, self.nx))
        for _ in range(10000):
            k_new = np.linalg.solve(r1 + b.T @ p @ b, b.T @ p @ a)
            p = q1 + a.T @ p @ (a - b @ k_new)
            if np.max(np.abs(k_new - k)) < 1e-12:
                k = k_new
                break
            k = k_new
        self.Kinf, self.Pinf = k, p
        self.Quu_inv = np.linalg.inv(r1 + b.T @ p @ b)
        self.AmBKt = (a - b @ k).T
        self.Pf = p - rho * np.eye(self.nx)            # 고정점에서의 종단 비용

    def reset(self) -> None:
        nb, n, nx, nu = self.nb, self.N, self.nx, self.nu
        self.x = np.zeros((nb, n, nx))
        self.u = np.zeros((nb, n - 1, nu))
        self.v = np.zeros((nb, n, nx))
        self.z = np.zeros((nb, n - 1, nu))
        self.g = np.zeros((nb, n, nx))
        self.y = np.zeros((nb, n - 1, nu))
        self.d = np.zeros((nb, n - 1, nu))
        self.p = np.zeros((nb, n, nx))
        self._warm = False

    def _shift(self) -> None:
        for name in ("x", "v", "g", "p"):
            arr = getattr(self, name)
            arr[:, :-1] = arr[:, 1:].copy()
        for name in ("u", "z", "y", "d"):
            arr = getattr(self, name)
            arr[:, :-1] = arr[:, 1:].copy()

    # ------------------------------------------------------------ 투영
    def _project_state(self, w: np.ndarray, Ha, hb, act) -> np.ndarray:
        out = w.copy()
        for (ix, iy) in self.vel_pairs:           # 속도 팔각형 (좌표가 겹치지 않아 분리 투영)
            out[..., [ix, iy]] = project_octagon(w[..., [ix, iy]], self.v_max)
        if Ha is not None:                         # 회피 반공간 (k ≥ 1)
            out[:, 1:] = dykstra_halfspaces(out[:, 1:], Ha[:, 1:], hb[:, 1:], act[:, 1:],
                                            self.st.dykstra_sweeps)
        out[:, 0] = w[:, 0]                        # x0 는 제약하지 않는다
        return out

    def _project_input(self, w: np.ndarray) -> np.ndarray:
        out = w.copy()
        for (ix, iy) in self.u_pairs:
            out[..., [ix, iy]] = project_octagon(w[..., [ix, iy]], self.u_max)
        return out

    # ------------------------------------------------------------ 풀기
    def solve(self, x0: np.ndarray, xref: np.ndarray,
              Ha: np.ndarray | None = None, hb: np.ndarray | None = None,
              act: np.ndarray | None = None) -> dict:
        """x0: (nb, nx), xref: (nb, N, nx), Ha: (nb, N, C, nx), hb/act: (nb, N, C)."""
        st, rho = self.st, self.st.rho
        if self._warm and st.shift_warm_start:
            self._shift()
        self._warm = True
        Q, A, B = self.Q, self.A, self.B
        Kt = self.Kinf.T
        done = np.zeros(self.nb, dtype=bool)
        iters = np.full(self.nb, st.max_iter)
        pri = dua = np.inf

        for it in range(st.max_iter):
            # 1) 전방 롤아웃: u = −K x − d
            self.x[:, 0] = x0
            for k in range(self.N - 1):
                self.u[:, k] = -self.x[:, k] @ Kt - self.d[:, k]
                self.x[:, k + 1] = self.x[:, k] @ A.T + self.u[:, k] @ B.T
            # 2) 슬랙 투영
            v_prev, z_prev = self.v, self.z
            self.z = self._project_input(self.u + self.y)
            self.v = self._project_state(self.x + self.g, Ha, hb, act)
            # 3) 쌍대 갱신
            self.y = self.y + self.u - self.z
            self.g = self.g + self.x - self.v
            # 4) 선형 비용
            r = -rho * (self.z - self.y)
            q = -(xref * Q) - rho * (self.v - self.g)
            p_last = -(xref[:, -1] @ self.Pf.T) - rho * (self.v[:, -1] - self.g[:, -1])
            # 5) 종료 판정 (배치별)
            if (it + 1) % st.check_every == 0:
                pri_b = np.maximum(np.abs(self.x - self.v).max(axis=(1, 2)),
                                   np.abs(self.u - self.z).max(axis=(1, 2)))
                dua_b = rho * np.maximum(np.abs(self.v - v_prev).max(axis=(1, 2)),
                                         np.abs(self.z - z_prev).max(axis=(1, 2)))
                newly = (~done) & (pri_b < st.abs_pri_tol) & (dua_b < st.abs_dua_tol)
                iters[newly] = it + 1
                done |= newly
                pri, dua = float(pri_b.max()), float(dua_b.max())
                if np.all(done):
                    break
            # 6) 역방향 패스 (벡터만)
            self.p[:, -1] = p_last
            for k in range(self.N - 2, -1, -1):
                self.d[:, k] = (self.p[:, k + 1] @ B + r[:, k]) @ self.Quu_inv.T
                self.p[:, k] = q[:, k] + self.p[:, k + 1] @ self.AmBKt.T - r[:, k] @ self.Kinf

        return {"x": self.x.copy(), "u": self.u.copy(), "z": self.z.copy(),
                "iters": iters, "solved": done.copy(), "pri_res": pri, "dua_res": dua}
