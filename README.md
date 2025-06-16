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

## Setup
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
