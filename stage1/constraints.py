"""회피 제약의 기하 — Qi 식 (5)·(7)~(9)의 휴리스틱 항이 옮겨 가는 자리.

모든 회피 제약을 하나의 형태로 통일한다.

    n'·p(t_k) ≥ b0 + b1·t_k          (n: 단위 법선, t_k = k·dt: 예측 시각)

예측 knot 마다 우변이 달라지는 시변 반공간이다. 등속으로 움직이는 장애물·이웃의
위치가 t 에 대한 1차식이라 이 형태 하나로 정적/동적을 모두 표현한다.
tinyadmm.py 가 knot 별 반공간을 그대로 받는다.

필요 여유 h(t) = h0 + s·t 를 "램프"로 둔 이유:
    현재 이미 여유가 부족한데 k=1 에서 바로 목표 여유 H 를 요구하면 한 주기 안에
    벌릴 수 없어 문제가 실현 불가능해지고, ADMM 은 실현 불가능한 문제에서 쌍대변수가
    누적되어 발산한다(공식 패키지로 시험할 때 실제로 관측됨). 그래서 h0 는 현재
    여유를 넘지 않게 자르고, 목표 여유 H 까지는 기울기 s 로 끌어올려 "언제까지
    벌려야 하는지"를 제약이 직접 말하게 한다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

EPS = 1e-9


@dataclass(frozen=True)
class HalfSpace:
    """기체 하나에 대한 n'p(t) ≥ b0 + b1·t."""
    n: np.ndarray       # (2,)
    b0: float
    b1: float
    kind: str           # "pair" | "obs" — 진단용
    urgent: bool = False  # 현재 이미 목표 여유 H 를 못 지키는 중이면 True
    b0_safe: float = 0.0  # 제동 여유로 낮추기 전의 b0 — 안전 필터(barrier)가 쓴다


@dataclass(frozen=True)
class PairHalfSpace:
    """중앙집중형 전용: n'(p_i(t) - p_j(t)) ≥ b0 + b1·t."""
    i: int
    j: int
    n: np.ndarray
    b0: float
    b1: float
    urgent: bool = False
    b0_safe: float = 0.0


def _unit(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    nrm = float(np.linalg.norm(v))
    if nrm < EPS:
        return fallback / float(np.linalg.norm(fallback))
    return v / nrm


def _ramp(current: float, target: float, t_reach: float, s_max: float) -> tuple[float, float, bool]:
    """h(t) = h0 + s·t. h0 = min(target, current), 부족분은 t_reach 안에 채운다."""
    h0 = min(target, current)
    if h0 >= target:
        return target, 0.0, False
    s = (target - h0) / max(t_reach, EPS)
    return h0, min(s, s_max), True


def _admissible(b0: float, b1: float, value0: float, rate0: float,
                a_brake: float, rate_max: float) -> tuple[float, float]:
    """제약 g(t) = value(t) − (b0 + b1·t) ≥ 0 이 '물리적으로 지킬 수 있게' 고친다.

    1) 요구 속도 상한: b1 은 제약된 양이 늘어나야 하는 속도다. 기체 최대 속도보다
       빠르면 어떤 입력으로도 못 지키므로 rate_max 로 자른다 (이때는 보장을 잃는다
       — 장애물이 기체보다 빠르게 덮치는 경우. 그 사실은 urgent 로 드러난다).
    2) 제동 여유: 지금 value 가 b1 보다 느리게 늘고 있으면(g'(0) < 0) 가속도 a 로
       따라잡는 동안 g 가 (g'(0))²/(2a) 만큼 더 줄어든다. b0 를 그만큼 낮춰 두지
       않으면 k=1 부터 실현 불가능해지고, 실현 불가능한 문제에서 ADMM 쌍대변수는
       warm start 를 타고 주기마다 누적되어 발산한다(실제로 관측됨).
       제어 장벽 함수(CBF)의 '제동거리' 조건과 같은 모양이다.
    """
    b1 = min(b1, rate_max)
    deficit_rate = b1 - rate0
    if deficit_rate > 0.0:
        b0 = min(b0, value0 - deficit_rate ** 2 / (2.0 * a_brake))
    return b0, b1


# ============================================================ 기체 간 (BVC)
def pair_halfspace_distributed(
    p_i: np.ndarray, v_i: np.ndarray, p_j: np.ndarray, v_j: np.ndarray,
    r_safe: float, t_recover: float, s_max: float, fallback_dir: np.ndarray,
    a_brake: float, rate_max: float, moving: bool = True,
) -> HalfSpace:
    """이동하는 buffered Voronoi cell — 분산형에서 기체 i 가 이웃 j 에 대해 거는 제약.

    두 기체의 중점 m(t) = m0 + v̄·t 에서 각자 r_safe/2 씩 물러난다.
        i:  n'(p_i - m(t)) ≥ h(t)
        j: -n'(p_j - m(t)) ≥ h(t)     (j 도 같은 식을 반대 부호로 세운다)
    두 식을 더하면 n'(p_i - p_j) ≥ 2h(t) 이므로, 둘 다 지키면 모든 예측 시점에서
    거리 ≥ r_safe 가 보장된다. 서로 교환하는 정보는 현재 위치·속도뿐으로,
    Qi 식 (5)가 쓰던 정보(상대 위치·상대 속도)와 똑같다.

    중점을 평균 속도로 이동시키는 이유: 편대 전체가 같은 방향으로 움직일 때
    정적 BVC 는 셀이 제자리에 묶여 있어 전진 자체를 막는다.

    상호 일관성: n, m0, v̄, s 는 두 기체 상태의 대칭 함수라 i 와 j 가 같은 값을
    얻는다. 중점 속도 성분 n'v̄ 도 대칭으로 잘라 둘 다 rate_max 안에 들게 한다.
    기체별로 다른 것은 _admissible 의 제동 여유뿐이고, 그만큼만 보장이 줄어든다.
    """
    n = _unit(p_i - p_j, fallback_dir)
    m0 = 0.5 * (p_i + p_j)
    half_gap = float(n @ (p_i - m0))                 # = 현재 거리 / 2
    h0, s, urgent = _ramp(half_gap, 0.5 * r_safe, t_recover, s_max)
    w = float(n @ (0.5 * (v_i + v_j))) if moving else 0.0
    lim = max(rate_max - s, 0.0)
    w = min(max(w, -lim), lim)
    b0_safe = float(n @ m0) + h0
    b0, b1 = _admissible(b0_safe, w + s, float(n @ p_i), float(n @ v_i), a_brake, rate_max)
    return HalfSpace(n=n, b0=b0, b1=b1, kind="pair", urgent=urgent, b0_safe=b0_safe)


def pair_halfspace_central(
    i: int, j: int, p_i: np.ndarray, v_i: np.ndarray, p_j: np.ndarray, v_j: np.ndarray,
    r_safe: float, t_recover: float, s_max: float, fallback_dir: np.ndarray,
    a_brake: float, rate_max: float,
) -> PairHalfSpace:
    """중앙집중형: 두 궤적을 한 문제에서 같이 정하므로 중점 가정이 필요 없다.
    상대 운동이라 제동 가속도와 속도 상한은 둘 다 두 배로 본다."""
    n = _unit(p_i - p_j, fallback_dir)
    dist = float(n @ (p_i - p_j))
    h0, s, urgent = _ramp(dist, r_safe, t_recover, 2.0 * s_max)
    b0, b1 = _admissible(h0, s, dist, float(n @ (v_i - v_j)), 2.0 * a_brake, 2.0 * rate_max)
    return PairHalfSpace(i=i, j=j, n=n, b0=b0, b1=b1, urgent=urgent, b0_safe=h0)


# ============================================================ 장애물 (split 의 대체)
def obstacle_halfspace(
    p: np.ndarray, v: np.ndarray, o: np.ndarray, v_o: np.ndarray,
    clearance: float, t_recover: float, s_max: float, side_hint: np.ndarray,
    a_brake: float, rate_max: float,
) -> HalfSpace:
    """동적 장애물에 대한 반공간. 법선은 최근접점(CPA) 기준의 '빗겨가는 방향'이다.

    상대 운동 r(t) = r0 + ṙ·t (r0 = p - o, ṙ = v - v_o) 가 가장 가까워지는 시각
    t_cpa 에서의 빗나감 벡터 m = r0 + ṙ·t_cpa 는 ṙ 에 수직이다. 법선을 m 방향으로
    잡으면 제약은 "상대 진행 방향의 옆으로 비켜라"가 된다. 기체마다 자기가 이미
    있는 쪽으로 비키므로 편대가 장애물 양옆으로 갈라지고(split), 장애물이
    멀어지면 제약이 풀려 추종 비용이 다시 모은다(merge). Qi 식 (8)~(9)의
    sign(·) 기반 측방 조향이 하던 일을 제약이 대신한다.

    정면 충돌(m≈0)이면 방향이 정의되지 않는다. 이때는 side_hint(편대 중심에서
    본 자기 오프셋 등)로 쪽을 정해 매 주기 방향이 뒤집히는 진동을 막는다.
    """
    r0 = p - o
    rdot = v - v_o
    closing = float(r0 @ rdot) < 0.0 and float(rdot @ rdot) > EPS
    if closing:
        t_cpa = -float(r0 @ rdot) / float(rdot @ rdot)
        miss = r0 + rdot * t_cpa
        perp = np.array([-rdot[1], rdot[0]])
        if float(perp @ side_hint) < 0.0:
            perp = -perp
        n = _unit(miss, perp)
        t_reach = t_cpa
    else:
        n = _unit(r0, side_hint)
        t_reach = t_recover
    current = float(n @ r0)
    h0, s, urgent = _ramp(current, clearance, t_reach, s_max)
    # n'(p(t) - o0 - v_o·t) ≥ h0 + s·t
    b1_raw = float(n @ v_o) + s
    b0_safe = float(n @ o) + h0
    b0, b1 = _admissible(b0_safe, b1_raw, float(n @ p), float(n @ v), a_brake, rate_max)
    return HalfSpace(n=n, b0=b0, b1=b1, kind="obs", urgent=urgent or b1 < b1_raw,
                     b0_safe=b0_safe)


# ============================================================ 안전 필터용 barrier
def barrier_sigma_min(g0: float, r0: float, dt: float, a_b: float, sigma_cap: float) -> float:
    """제동거리 barrier 를 다음 스텝에 지키는 데 필요한 최소 법선 가속도 σ = n'u.

    반공간 g(t) = n'p − (b0 + b1·t) ≥ 0, 그 변화율 r = n'v − b1 에 대해
        B(g, r) = g − max(0, −r)² / (2·a_b)      ("지금 a_b 로 제동하면 멈출 수 있다")
    를 불변으로 두고 싶다. 다음 스텝 값은 σ 에 대해
        g1 = g0 + r0·dt + ½dt²·σ,   r1 = r0 + dt·σ
    이고 B(g1, r1) 은 σ 에 대해 단조 증가다. 그래서 "B(g1, r1) ≥ 0" 은 σ ≥ σ_min 이라는
    u 공간의 반공간 하나가 된다(이산 시간 CBF 와 같은 구조, 선형이라 QP 가 필요 없다).
    σ_min 이 σ_cap(낼 수 있는 최대 가속)을 넘으면 이미 늦은 상태 — 최대 제동을 요구한다.
    """
    def barrier(sig: float) -> float:
        g1 = g0 + r0 * dt + 0.5 * dt * dt * sig
        r1 = r0 + dt * sig
        return g1 - max(0.0, -r1) ** 2 / (2.0 * a_b)

    lo, hi = -sigma_cap, sigma_cap
    if barrier(lo) >= 0.0:
        return lo                     # 어떤 입력이든 안전 — 제약 없음과 같다
    if barrier(hi) < 0.0:
        return hi                     # 늦었다: 최대 제동
    for _ in range(40):               # 단조 함수의 이분법 (MCU 에서는 닫힌 식으로 바꿀 수 있다)
        mid = 0.5 * (lo + hi)
        if barrier(mid) >= 0.0:
            hi = mid
        else:
            lo = mid
    return hi
