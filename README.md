# LTS‑MATLAB – Lap‑Time Simulator

## What it does
Calculates a feasible speed profile and lap time for a vehicle on a given track, using:
- Quasi‑static vehicle dynamics (bicycle or 4‑wheel)
- Tyre models: Linear, Nonlinear Saturating, Pacejka Magic Formula
- Load transfer (longitudinal + lateral) and aero
- GGV surface generation
- 3‑pass lap simulation (cornering limit + forward acceleration + backward braking)

## How to run
```matlab
result = run_laptime_sim();                % uses default circular track
result = run_laptime_sim('my_track.xlsx'); % reads Excel file
result = run_laptime_sim(xy);              % uses Nx2 numeric array
