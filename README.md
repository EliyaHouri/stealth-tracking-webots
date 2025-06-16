# Stealth Tracking in Polygonal Environments

This project implements the methods from the thesis _Stealth Tracking in 
Polygonal Environments_ in Python and Webots.

## Structure
- `world/` – Webots world file.
- `controllers/` – target and observer Webots controllers.
- `src/` – core Python modules:
  - `world_model.py` – obstacle loader and free-space checks.
  - `discretization.py` – grid and graph construction.
  - `distributions.py` – common random distributions.
  - `dp_solver.py` – dynamic programming solver.
  - `utils.py` – geometry helpers (visibility, detection).
- `tests/` – unit tests for key modules.

## New pipeline
1. `python src/world_generator.py` — generates `obstacles.json` with fixed seed.
2. `python src/dp_solver.py --seed 42 --num_obs 5` — builds graph, computes Dijkstra path, samples suspicious flags, and solves DP, outputting `policy.pkl`.
3. Launch Webots:
   ```bash
   webots world/epuck2_tracking.wbt
