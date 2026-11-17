# Inverted Pendulum: Swing-up and Balancing

**Differential Equations group2 project ( Inverted Pendulum Challenge)**
Mbarara University of Science and Technology

A virtual "broom on your hand". A pendulum starts hanging down on a movable cart. The program swings it up, catches it at the top, balances it, and recovers when pushed. Every frame on screen comes from numbers computed by our own **Euler** and **Runge-Kutta 4 (RK4)** solvers. No ready-made ODE solver or physics engine is used.

## The idea in plain words
- A **differential equation** is a rule for change. Ours says: *how fast the pendulum's turning speeds up = gravity's pull − friction − the effect of moving the hand*.
- Near the top, gravity pushes the pendulum further away, so a small lean grows about 23 times per second. The upright position is **unstable**.
- A **PD controller** moves the hand toward the lean (P) and brakes the motion (D) to keep it up.
- An **energy controller** swings the pendulum up first, like pushing a child on a swing, then hands over to the PD controller near the top.

## The ODE
```
theta'' = (g/L) sin(theta) - b theta' - (a/L) cos(theta)
```
State `s = [x, v, theta, omega]`; `theta = 0` is upright, `theta = pi` is hanging. The input `a` is the hand's acceleration (limited to ±25 m/s²). Parameters: g = 9.81, L = 1.0 m, b = 0.05 1/s.

## Setup
```bash
python -m venv venv
venv\Scripts\activate            # Mac/Linux: source venv/bin/activate
pip install -r requirements.txt  # if pygame fails: pip install pygame-ce
```

## Run
```bash
python compare.py    # builds every graph in figures/ and results_table.md
python simulate.py   # live animated simulation
```

### Keys in the live simulation
| Key | Action | Key | Action |
|---|---|---|---|
| R | restart hanging down | E | switch Euler / RK4 |
| B | start near upright | D | change step size |
| M | manual mode (LEFT / RIGHT) | [ ] | change pendulum length |
| A | automatic swing-up and balance | K | change damping |
| O | controller off | H | show or hide labels |
| SPACE | push (disturbance) | ESC | quit |

Events handled: fall detected, caught (swing-up to balance), balance lost (back to swing-up), track end warning, solver blow-up.

## Results at a glance
| Method | Step dt (s) | Steps | Angle error at t = 10 s (rad) |
|---|---|---|---|
| Euler | 0.1 | 100 | 39.7 |
| Euler | 0.01 | 1000 | 0.965 |
| Euler | 0.001 | 10000 | 0.066 |
| RK4 | 0.1 | 100 | 0.00076 |
| RK4 | 0.01 | 1000 | 0.00000014 |
| RK4 | 0.001 | 10000 | 0.000000000014 |

- Measured convergence: Euler about order 1.3, RK4 about order 3.9 (theory: 1 and 4).
- Swing-up then catch takes about 3.5 s. A push at t = 12 s is rejected.
- Balancing stands from 5° to 45° starts, fails from 60°. Swing-up works for lengths up to 1.0 m and fails from 1.3 m. Medium damping or more (b ≥ 0.3) defeats the limited swing-up push.
- Euler with dt = 0.1 never catches the pendulum; RK4 with the same step does. That failure is numerical, not physical.

## Files
| File | Purpose |
|---|---|
| `pendulum.py` | the ODE model, our Euler and RK4, energy, PD and swing-up controllers |
| `compare.py` | numerical comparison, experiments, validation, all figures |
| `simulate.py` | Pygame live simulation |
| `figures/` | generated graphs (fig0 to fig12) |
| `results_table.md` | Euler vs RK4 table |
| `GUIDE.md` | study guide and viva questions |
| `REPORT_TEMPLATE.md` | report structure |
| `requirements.txt` | numpy, matplotlib, pygame |



## Limitations
The pivot is moved directly by an acceleration (no motor model). The pole is a point mass on a massless rod. There is no sensor noise or delay. Swing-up and catch rules were tuned for L = 1 m.

## Authors
BIYIMBWA ELIJAH SIMWOGERERE            2025/BCS/068/PS 
NOREEN MUSASIZI                        2025/BCS/201/PS
SHADRACH OKORI SMITH                   2025/BCS/205/PS
AGENGO ANNE MARIE CASMIRO              2025/BCS/009/PS
AINOMUGISHA RONALD                     2025/BCS/021/PS
JUMAH KYEGOMBE                         2024/BCS/093/PS

