# LQR gain fitting (wbr_2026 `User/Matlab/fit.m` → Python)

Port of [CosmosMount/wbr_2026](https://github.com/CosmosMount/wbr_2026) MATLAB scripts. No MATLAB required.

## Quick start

```bash
# Fit and update header
python3 tools/lqr/fit_lqr.py --write

# Print C++ only
python3 tools/lqr/fit_lqr.py

# Scale existing coeffs without refitting (numpy only)
python3 tools/lqr/fit_lqr.py --scale 0.62 --write
```

## Dependencies

| Mode | Needs |
|------|--------|
| `--scale` | Python 3 + numpy |
| full refit | Python 3 + numpy + scipy |

## Tuning

Edit `LQR_WEIGHTS` in `fit_lqr.py` (`low` / `high` / `spin` each have `Q` and `R`):

- **Softer**: lower `Q` on alpha/pitch, higher `R` on wheel/hip torques
- **Stiffer**: raise `Q` or lower `R`

Output polynomial matches `lqr_solver` in `controller/include/control/balance.hpp`:

`a1 + a2*L + a3*R + a4*L² + a5*L*R + a6*R²`

## Files

| File | Role |
|------|------|
| `fit_lqr.py` | Main CLI: LQR + polynomial fit → `lqr_coeffs.hpp` |
| `dynamics.py` | `compute_A` / `compute_B` from fit.m |
| `leg_data.py` | Leg inertia table |
| `amatrix.py`, `bmatrix.py` | Auto-generated from `vendor/*.m` |
| `gen_matrices.py` | Regenerate `amatrix.py` / `bmatrix.py` |
| `vendor/Amatrix.m`, `Bmatrix.m` | Copied from wbr_2026 |
