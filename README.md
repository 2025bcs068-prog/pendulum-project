# Inverted Pendulum on a Cart: Swing-up and Balance
Differential Equations group project (Project 7, Inverted Pendulum Challenge)

A virtual version of the "pole on a cart" laboratory rig: the pendulum starts hanging down, is swung up by an energy controller, caught, and balanced by a PD controller. Euler and RK4 are written by us (no ready-made ODE solver, no physics engine).

## Setup
```
python -m venv venv
venv\Scripts\activate          (Mac/Linux: source venv/bin/activate)
pip install -r requirements.txt
```
If pygame fails to install: `pip install pygame-ce`.

## Run
```
python compare.py     # makes all plots in figures/ and results_table.md
python simulate.py    # live animated simulation (keys are listed at the top of simulate.py)
```

## Files
| File | Purpose |
|---|---|
| pendulum.py | ODE model, our Euler and RK4, energy, PD and swing-up controllers |
| compare.py | numerical comparison, experiments, validation, all figures |
| simulate.py | Pygame live simulation (all motion comes from the numerical states) |
| GUIDE.md | study guide: theory, how to read each graph, demo plan, viva questions |
| REPORT_TEMPLATE.md | 1-3 page report structure linked to the marking scheme |

## Marking scheme -> where the evidence is
| Component (marks) | Evidence in this project |
|---|---|
| Formulation, assumptions, ODEs (25) | fig0, GUIDE sections 2-4, `f()` in pendulum.py, section 16 limits |
| Independent Euler and RK4 (25) | `euler_step`, `rk4_step` in pendulum.py (hand written) |
| Numerical comparison and experiments (20) | fig1, fig2, fig3, results_table.md; experiments fig5 (gain), fig6 (start angle), fig9 (damping), fig10 (length), step size fig8 |
| Interactive simulation and events (15) | simulate.py: push, manual mode, fall detection, catch/lost-balance events, track limit, solver blow-up |
| Validation and limitations (10) | fig4 (small-angle formula), fig3 (energy), fig8 (numerical vs physical), GUIDE section 16 |
| Report, code quality, demo (5) | REPORT_TEMPLATE.md, comments in code, GUIDE section 11 |
