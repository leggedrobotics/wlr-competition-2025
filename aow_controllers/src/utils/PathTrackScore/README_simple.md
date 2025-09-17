# PTS (Simplified): XY-only + Time

This variant removes yaw and reference-speed. It rewards **staying near the path** and **finishing sooner**.

## Score
\[
\text{PTS} = 100 \cdot A^{\alpha} \cdot \underbrace{\frac{1}{1+T/\tau}}\_{\text{time reward}} \cdot \exp\!\left(-\kappa\left(1-\frac{s_\text{final}}{L}\right)\right)
\]

- **Accuracy** (XY only): \(A=\exp(-w_d\,\overline{(d/d_0)^2})\).
- **Time reward**: smaller total time \(T\) gives a larger term; \(\tau\) is a *time constant* (choose it near a typical completion time for your course). This is not a target speed.
- **Completeness**: unfinished runs are penalized smoothly by the covered fraction \(s_\text{final}/L\).

## API

```python
from pts_metric_simple import compute_pts_simple, PTSParamsSimple
```

### Parameters
- `d0` (m): lateral RMS scale (e.g., 0.02–0.05).
- `w_d`: weight on XY accuracy (default 1.0).
- `tau` (s): time constant for the time reward; pick around the median finish time you expect.
- `kappa`: penalty strength for DNFs (default 2.0).
- `alpha`, `beta`: top-level tradeoff between accuracy and time.

### Tips
- If you want time to matter **more**, increase `beta` or **decrease** `tau`.
- If you want accuracy to dominate, increase `alpha`, `w_d` or **decrease** `d0`.
