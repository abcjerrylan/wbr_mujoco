"""Build 10x10 A and 10x4 B from wbr_2026 fit.m (compute_A / compute_B)."""

from __future__ import annotations

import numpy as np

from amatrix import amatrix
from bmatrix import bmatrix


def compute_a(j_a: np.ndarray, r_w: float, r_l: float, l_l: float, l_r: float) -> np.ndarray:
    a = np.zeros((10, 10), dtype=float)
    for p in range(5, 10, 2):
        col = (p - 3) // 2 - 1
        a[1, p - 1] = r_w * (j_a[0, col] + j_a[1, col]) / 2.0
        a[3, p - 1] = (
            (r_w * (-j_a[0, col] + j_a[1, col])) / (2.0 * r_l)
            - (l_l * j_a[2, col]) / (2.0 * r_l)
            + (l_r * j_a[3, col]) / (2.0 * r_l)
        )
        for q in range(6, 11, 2):
            a[q - 1, p - 1] = j_a[q // 2 - 1, col]

    for r in range(10):
        if r % 2 == 1:
            a[r, 0:4] = 0.0
            a[r, 5] = 0.0
            a[r, 7] = 0.0
            a[r, 9] = 0.0
        else:
            a[r, :] = 0.0
            a[r, r + 1] = 1.0
    return a


def compute_b(j_b: np.ndarray, r_w: float, r_l: float, l_l: float, l_r: float) -> np.ndarray:
    b = np.zeros((10, 4), dtype=float)
    for h in range(4):
        b[1, h] = r_w * (j_b[0, h] + j_b[1, h]) / 2.0
        b[3, h] = (
            (r_w * (-j_b[0, h] + j_b[1, h])) / (2.0 * r_l)
            - (l_l * j_b[2, h]) / (2.0 * r_l)
            + (l_r * j_b[3, h]) / (2.0 * r_l)
        )
        for f in range(6, 11, 2):
            b[f - 1, h] = j_b[f // 2 - 1, h]

    for e in range(0, 9, 2):
        b[e, :] = 0.0
    return b


def matrices_at_lengths(l_length: float, r_length: float, leg_table: np.ndarray, r_w: float, r_l: float):
    l_row = int(round((l_length - 0.10) / 0.01))
    r_row = int(round((r_length - 0.10) / 0.01))
    l_wl, l_bl, i_ll = leg_table[l_row, 1], leg_table[l_row, 2], leg_table[l_row, 3]
    l_wr, l_br, i_lr = leg_table[r_row, 1], leg_table[r_row, 2], leg_table[r_row, 3]

    j_a = amatrix(l_length, r_length, l_wl, l_bl, i_ll, l_wr, l_br, i_lr)
    j_b = bmatrix(l_length, r_length, l_wl, l_bl, i_ll, l_wr, l_br, i_lr)
    a = compute_a(j_a, r_w, r_l, l_length, r_length)
    b = compute_b(j_b, r_w, r_l, l_length, r_length)
    return a, b
