# Inverted Pendulum on a Cart: Swing-up and Balance
Differential Equations group project ( Inverted Pendulum Challenge)

A virtual version of the "pole on a cart" laboratory rig, with a mass on the tip: the pendulum starts hanging down, is swung up by an energy controller, caught, and balanced by a PD controller. Euler and RK4 are written by us (no ready-made ODE solver, no physics engine).

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



fig3 (energy), fig8 (numerical vs physical), GUIDE section 16 |
| Report, code quality, demo (5) | REPORT_TEMPLATE.md, comments in code, GUIDE section 11 |
