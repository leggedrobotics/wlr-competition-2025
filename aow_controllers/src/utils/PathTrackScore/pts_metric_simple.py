
import numpy as np
import math
from dataclasses import dataclass

@dataclass
class PTSParamsSimple:
    d0: float = 0.05
    w_d: float = 1.0
    tau: float = 20.0
    kappa: float = 2.0
    alpha: float = 1.0
    beta: float = 1.0

def compute_pts_simple(path_xy, t, x, y, params: PTSParamsSimple = PTSParamsSimple()):
    path = np.asarray(path_xy, dtype=float)
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    assert path.ndim == 2 and path.shape[1] == 2 and len(path) >= 2, "path_xy must be (N,2) with N>=2"
    assert np.all(np.diff(t) > 0), "t must be strictly increasing"

    seg_vecs = path[1:] - path[:-1]
    seg_lens = np.linalg.norm(seg_vecs, axis=1)
    assert np.all(seg_lens > 0), "Path segments must be non-zero length"
    tangents = seg_vecs / seg_lens[:, None]
    cumlen = np.concatenate([[0.0], np.cumsum(seg_lens)])
    L = float(cumlen[-1])

    points = np.stack([x, y], axis=1)
    s_proj = np.zeros(len(t))
    d_signed = np.zeros(len(t))

    for i, p in enumerate(points):
        ap = p - path[:-1]
        v = seg_vecs
        v2 = (seg_lens**2)
        u = np.clip((ap * v).sum(axis=1) / v2, 0.0, 1.0)
        q = path[:-1] + (v * u[:, None])
        diff = p - q
        d2 = (diff**2).sum(axis=1)
        j = int(np.argmin(d2))
        u_j = u[j]
        seg_base_s = cumlen[j]
        s_proj[i] = seg_base_s + u_j * seg_lens[j]
        tan = tangents[j]
        diff_j = p - q[j]
        cross = tan[0]*diff_j[1] - tan[1]*diff_j[0]
        d_signed[i] = math.copysign(np.linalg.norm(diff_j), cross)

    s_mono = np.maximum.accumulate(s_proj)
    s_final = min(L, float(s_mono[-1]))
    finished = bool(s_mono[-1] >= L)

    if finished:
        idx = int(np.searchsorted(s_mono, L, side='left'))
        if idx == 0:
            t_finish = t[0]
        elif s_mono[idx] == L or idx >= len(t):
            t_finish = t[min(idx, len(t)-1)]
        else:
            s0, s1 = s_mono[idx-1], s_mono[idx]
            t0, t1 = t[idx-1], t[idx]
            ratio = (L - s0) / max(1e-12, (s1 - s0))
            t_finish = t0 + ratio * (t1 - t0)
        T = max(1e-9, float(t_finish - t[0]))
    else:
        T = max(1e-9, float(t[-1] - t[0]))

    Rd2 = float(np.mean((d_signed / params.d0)**2))
    A = math.exp(-params.w_d * Rd2)

    T_term = 1.0 / (1.0 + (T / params.tau))

    F_term = math.exp(-params.kappa * (1.0 - s_final / L))

    pts = 100.0 * (A**params.alpha) * (T_term**params.beta) * F_term

    return {
        "pts": float(pts),
        "A": float(A),
        "T_term": float(T_term),
        "F_term": float(F_term),
        "finished": finished,
        "s_final": float(s_final),
        "T": float(T),
        "Rd2": float(Rd2),
        "L": float(L),
    }
