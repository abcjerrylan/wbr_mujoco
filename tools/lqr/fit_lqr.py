#!/usr/bin/env python3
"""
Fit dual-leg LQR polynomial gains for wbr_mujoco (Python port of wbr_2026 User/Matlab/fit.m).

Dependencies: Python 3 + numpy only for --scale mode.
Full refit also needs scipy (usually already installed with numpy).

Examples:
    # Print C++ arrays
    python3 tools/lqr/fit_lqr.py

    # Write controller/include/control/lqr_coeffs.hpp
    python3 tools/lqr/fit_lqr.py --write

    # Scale existing header without refitting (numpy only)
    python3 tools/lqr/fit_lqr.py --scale 0.62

    # Regenerate amatrix.py / bmatrix.py after updating vendor/*.m
    python3 tools/lqr/gen_matrices.py
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

from dynamics import matrices_at_lengths
from leg_data import LEG_DATA, L_VALS, R_L, R_W

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DEFAULT_HEADER = REPO / "controller" / "include" / "control" / "lqr_coeffs.hpp"

# Edit Q/R here. State order:
# Q: [s, ds, yaw, dyaw, alpha_l, dalpha_l, alpha_r, dalpha_r, pitch, dpitch]
# R: [T_wl, T_wr, T_hip_l, T_hip_r]
LQR_WEIGHTS: dict[str, dict[str, list[float]]] = {
    "low": {
        "Q": [80, 10, 300, 45, 4500, 200, 4500, 200, 32000, 500],
        "R": [10, 10, 1, 1],
    },
    "high": {
        "Q": [80, 10, 300, 45, 4500, 200, 4500, 200, 32000, 500],
        "R": [10, 10, 1, 1],
    },
    "spin": {
        "Q": [80, 10, 300, 45, 4500, 200, 4500, 200, 32000, 500],
        "R": [10, 10, 1, 1],
    },
}

ARRAY_MAP = {
    "low": "k_lqr_low",
    "high": "k_lqr_high",
    "spin": "k_lqr_spin",
}


def solve_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    try:
        from scipy.linalg import solve_continuous_are
    except ImportError as exc:
        raise RuntimeError("scipy is required for refit; use --scale or: pip install scipy") from exc
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def fit_gains(q_diag: list[float], r_diag: list[float]) -> np.ndarray:
    Q = np.diag(q_diag)
    R = np.diag(r_diag)
    num_samples = len(L_VALS) * len(L_VALS)
    k_samples = np.zeros((4, 10, num_samples))
    idx = 0
    for l_len in L_VALS:
        for r_len in L_VALS:
            A, B = matrices_at_lengths(l_len, r_len, LEG_DATA, R_W, R_L)
            k_samples[:, :, idx] = -solve_lqr(A, B, Q, R)
            idx += 1

    L_grid, R_grid = np.meshgrid(L_VALS, L_VALS)
    L_vec = L_grid.T.ravel()
    R_vec = R_grid.T.ravel()
    X = np.column_stack([L_vec ** 2, L_vec * R_vec, R_vec ** 2, L_vec, R_vec, np.ones(num_samples)])

    coeffs = np.zeros((4, 10, 6))
    for i in range(4):
        for j in range(10):
            c = np.linalg.lstsq(X, k_samples[i, j, :], rcond=None)[0]
            coeffs[i, j, :] = [c[5], c[3], c[4], c[0], c[1], c[2]]
    return coeffs


def eval_k(coeffs: np.ndarray, ll: float, lr: float) -> np.ndarray:
    k = np.zeros((4, 10))
    for i in range(4):
        for j in range(10):
            a1, a2, a3, a4, a5, a6 = coeffs[i, j]
            k[i, j] = a1 + a2 * ll + a3 * lr + a4 * ll * ll + a5 * ll * lr + a6 * lr * lr
    return k


def parse_header(path: Path) -> dict[str, np.ndarray]:
    text = path.read_text()
    out: dict[str, np.ndarray] = {}
    for key, name in ARRAY_MAP.items():
        m = re.search(rf"constexpr float {name}\[40\]\[6\] =\s*\{{(.*?)\}};", text, re.S)
        if not m:
            raise ValueError(f"{name} not found in {path}")
        rows = []
        for line in m.group(1).strip().splitlines():
            line = line.strip().rstrip(",")
            if not line.startswith("{"):
                continue
            rows.append([float(x) for x in line.strip("{}").split(",")])
        if len(rows) != 40:
            raise ValueError(f"{name}: expected 40 rows, got {len(rows)}")
        out[key] = np.array(rows).reshape(4, 10, 6)
    return out


def format_array(name: str, coeffs: np.ndarray, q_diag: list[float], r_diag: list[float]) -> str:
    lines = [
        f"constexpr float {name}[40][6] =",
        "{",
        f"    /*Q = [{', '.join(f'{x:.2f}' for x in q_diag)}] R = [{', '.join(f'{x:.2f}' for x in r_diag)}]*/",
        "    /* a1 + a2*L_len + a3*R_len + a4*L_len^2 + a5*L_len*R_len + a6*R_len^2 */",
    ]
    for i in range(4):
        for j in range(10):
            c = coeffs[i, j]
            lines.append(
                f"    {{ {c[0]:9.6f} , {c[1]:9.6f}, {c[2]:9.6f},"
                f" {c[3]:9.6f}, {c[4]:9.6f}, {c[5]:9.6f}}},"
            )
    lines.append("};")
    return "\n".join(lines)


def write_header(path: Path, coeffs: dict[str, np.ndarray]) -> None:
    body = [
        "#pragma once",
        "namespace control {",
        "enum class lqr_mode { low, high, spin };",
        "",
    ]
    for key in ("low", "high", "spin"):
        body.append(format_array(ARRAY_MAP[key], coeffs[key], LQR_WEIGHTS[key]["Q"], LQR_WEIGHTS[key]["R"]))
        body.append("")
    body.append("} // namespace control")
    body.append("")
    path.write_text("\n".join(body))


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit LQR polynomial gains for wbr_mujoco")
    parser.add_argument("--scale", type=float, default=None, help="Scale coeffs from --from-header")
    parser.add_argument("--from-header", type=Path, default=DEFAULT_HEADER)
    parser.add_argument("--write", action="store_true", help="Update controller/include/control/lqr_coeffs.hpp")
    parser.add_argument("--eval-len", type=float, default=0.16, help="Leg length for ||K|| diagnostic")
    args = parser.parse_args()

    if args.scale is not None:
        parsed = parse_header(args.from_header)
        coeffs = {k: parsed[k] * args.scale for k in parsed}
    else:
        coeffs = {}
        for key in ("low", "high", "spin"):
            cfg = LQR_WEIGHTS[key]
            coeffs[key] = fit_gains(cfg["Q"], cfg["R"])

    for key in ("low", "high", "spin"):
        cfg = LQR_WEIGHTS[key]
        print(format_array(ARRAY_MAP[key], coeffs[key], cfg["Q"], cfg["R"]))
        print()

    if args.eval_len > 0:
        ll = lr = args.eval_len
        k = eval_k(coeffs["low"], ll, lr)
        print(f"// @ L=R={ll:.2f}: ||K_low||_F={np.linalg.norm(k):.2f}", file=sys.stderr)

    if args.write:
        write_header(DEFAULT_HEADER, coeffs)
        print(f"wrote {DEFAULT_HEADER}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
