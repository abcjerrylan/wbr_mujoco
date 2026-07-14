#!/usr/bin/env python3
"""One-shot: convert vendor/Amatrix.m and Bmatrix.m to amatrix.py / bmatrix.py."""

from __future__ import annotations

import re
from pathlib import Path


def matlab_line_to_py(line: str) -> str:
    line = re.sub(r"\.(\^|\*|/)", r"\1", line)
    line = line.replace("^", "**")
    return line.replace(";", "")


def convert_m_to_py(m_path: Path, py_path: Path, func_name: str, ret: str) -> None:
    text = m_path.read_text()
    body = re.sub(r"^function.*?\n", "", text, count=1)
    body = re.sub(r"\nend\s*$", "", body.strip())
    lines_out = ["import numpy as np", "", f"def {func_name}(l_l, l_r, l_wl, l_bl, I_ll, l_wr, l_br, I_lr):"]
    for line in body.splitlines():
        line = line.strip()
        if not line or line.startswith("%"):
            continue
        line = matlab_line_to_py(line)
        line = re.sub(
            rf"{ret} = reshape\(\[([^\]]+)\],(\d+),(\d+)\)",
            r"return np.concatenate([\1]).reshape((\2, \3), order='F')",
            line,
        )
        lines_out.append("    " + line)
    py_path.write_text("\n".join(lines_out) + "\n")


def main() -> int:
    here = Path(__file__).resolve().parent
    vendor = here / "vendor"
    convert_m_to_py(vendor / "Amatrix.m", here / "amatrix.py", "amatrix", "J_A")
    convert_m_to_py(vendor / "Bmatrix.m", here / "bmatrix.py", "bmatrix", "J_B")
    print(f"generated {here / 'amatrix.py'} and {here / 'bmatrix.py'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
