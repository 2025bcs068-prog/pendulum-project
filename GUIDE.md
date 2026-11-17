# Inverted Pendulum: Swing-up and Balance
### Differential Equations project guide (learn it, then present it)

## 0. How to run (5 minutes)
1. Open Command Prompt in this folder.
2. `python -m venv venv` then `venv\Scripts\activate`
3. `pip install -r requirements.txt`   (if pygame fails: `pip install pygame-ce`)
4. `python compare.py`   makes all graphs in the folder `figures`
5. `python simulate.py`  opens the live demo

Files:
- `pendulum.py`  the equations, our own Euler and RK4, and the controllers (the heart of the project)
- `compare.py`   all graphs for the report
- `simulate.py`  the live Pygame animation

---

## 1. What is a differential equation (DE)?
An ordinary equation gives a number. A DE gives a **rule for change**.
Example: "the speed of a falling stone is how fast its position changes": x' = v.
Solving a DE means finding the whole path (position over time) that follows the rule.
Many DEs cannot be solved with a formula, so we **step through time with a computer**. That is numerical solving.

## 2. Our physical story
A broom (rod with a weight at the end, length L) is held by a hand (the pivot) that can move left and right.
- Angle theta is measured from **upright**. theta = 0 is up, theta = pi (180 degrees) is hanging down.
- The hand has acceleration a. This is our control: a is the "push" we choose.

## 3. Building the equation
Torque balance about the pivot gives:

    theta'' = (g/L) sin(theta) - b theta' - (a/L) cos(theta)

each term:
- (g/L) sin(theta): gravity. Near the top it **pulls the pendulum away** (instability).
- b theta': damping (air and friction). It removes energy.
- (a/L) cos(theta): the effect of moving the hand.

The pivot itself follows x' = v and v' = a.

## 4. Turn it into first-order equations (needed for Euler and RK4)
Solvers only know first-order form. Let omega = theta'. State s = [x, v, theta, omega]:

    x'     = v
    v'     = a
    theta' = omega
    omega' = (g/L) sin(theta) - b omega - (a/L) cos(theta)

In code this is the function `f(s, a)` in `pendulum.py`.

## 5. Euler's method (simplest)
Idea: at the current point, find the slope, and walk one small step dt along it.

    s_next = s + dt * f(s)

- Easy, but each step has a small error that adds up.
- Global error is proportional to dt (order 1). Halve dt, the error halves.

## 6. Runge-Kutta 4 (RK4)
Idea: measure the slope four times inside one step, then take a weighted average.

    k1 = f(s)
    k2 = f(s + dt/2 * k1)
    k3 = f(s + dt/2 * k2)
    k4 = f(s + dt * k3)
    s_next = s + dt/6 * (k1 + 2 k2 + 2 k3 + k4)

- Four times the work per step, but global error is proportional to dt^4 (order 4). Halve dt, the error drops by 16.
- Our own run measured: Euler about 1.3 and RK4 about 3.9 (figure 2). This proves the theory.
- Note: in our code the control a is held constant during one step (this is normal for digital control).

## 7. Why the pendulum falls and how control stops it (linearization and stability)
Near the top, theta is small: sin(theta) ~ theta and cos(theta) ~ 1. Then

    theta'' = (g/L) theta - a/L

With no control (a = 0), the characteristic equation is lambda^2 = g/L, so lambda = +/- sqrt(g/L) = +/- 3.13.
One root is **positive**, so the upright position is **unstable**. This is the mathematical reason it falls.

Now choose PD control: a = Kp theta + Kd theta'. Then

    theta'' + (Kd/L) theta' + ((Kp - g)/L) theta = 0

This is stable when **Kp > g** and Kd > 0. With Kp = 40, Kd = 10, L = 1:
lambda^2 + 10 lambda + 30.19 = 0, so lambda = -5 +/- 2.28 i. Both have negative real part, so the pendulum stands up.
- P (Kp): push toward the side it leans, in proportion to the angle.
- D (Kd): brake the motion, so it does not overshoot.
- Experiment 1 (figure 5) shows what happens if Kp is too small.
- In the code we also add small terms Kx x + Kv v to keep the hand near the middle of the track. (The full 4-variable system was checked with eigenvalues, all negative real parts.)

## 8. Swing-up (the "several tries" in your broom story)
Energy (per m L^2): E = 1/2 omega^2 + (g/L)(cos(theta) - 1).
- Hanging at rest: E = -2g/L = -19.6.  Upright at rest: E = 0.
From the equation: dE/dt = -(a/L) omega cos(theta) - b omega^2.
So we choose the sign of a to make dE/dt positive:

    a = K * E * sign(omega cos(theta))     (E is negative, so this pushes the right way)

Each swing adds energy, like pushing a child on a swing. When theta is within 0.35 rad (20 degrees) of the top and slow enough, the program **switches to PD balance**. If it falls past 1.2 rad, it goes back to swing-up. That is a small "hybrid" controller with two modes.

## 9. Validation 
1. Small-angle formula: for a small swing around hanging with no damping, theta = phi0 cos(sqrt(g/L) t). Our RK4 agrees within 0.04 degrees (figure 4).
2. Energy conservation: with b = 0 and a = 0, energy must stay constant. RK4 keeps it, Euler drifts (figure 3).
3. Reference solution: RK4 with a tiny step (dt = 0.0005) is the "almost exact" answer for the error graphs (figures 1 and 2).
4. Numerical failure versus physical failure: figure 8. Same physics, same controller, only the solver changed. Euler with dt = 0.1 never catches the pendulum, RK4 with the same dt does. So that failure is **numerical**, not physical.

## 10. Graphs
| File | What it shows |
|---|---|
| fig1 | Euler and RK4 at three step sizes against the reference |
| fig2 | error versus step size (order of the methods) |
| fig3 | energy drift (validation) |
| fig4 | small-angle formula versus RK4 |
| fig5 | Experiment 1: different Kp |
| fig6 | Experiment 2: different starting angles |
| fig7 | full scenario: hanging, swing-up, balance, push, recover |
| fig8 | numerical failure: same system, different solver |
| fig0 | model diagram (put it first in the report) |
| fig9 | Experiment 3: damping |
| fig10 | Experiment 4: robustness to pendulum length |
| fig11 | phase portrait: why the top is unstable and how swing-up reaches it |
| fig12 | controller logic (two modes, two switching rules) |
| results_table.md | Euler vs RK4 error, steps and cost at three step sizes |

## 11. Live demo 
Keys: R hanging start | B balance start | M manual (you balance it) | A auto | O controller off | SPACE push | E Euler/RK4 | D step size | [ ] pendulum length | K damping | H labels | ESC quit

1. Press M and try to balance it by hand (hold LEFT and RIGHT). It falls. Say: "The upright position is unstable, as the eigenvalue +3.13 predicts." The program shows "FALL DETECTED" (event handling).
2. Press R: hanging start. Watch the swing-up stage, the catch, and the balance. The EVENTS list on the screen shows "CAUGHT".
3. Press SPACE: disturbance rejection. It recovers.
4. Press ] three times: longer pendulum (robustness to parameter variation, like the weight in the video). Press R again and explain why it can no longer be caught (figure 10).
5. Press K: damping. Explain figure 9.
6. Press E and D: Euler with a big step fails, RK4 does not. Explain with figure 2 and figure 8.
7. Close with figure 11 (phase portrait) and the results table.


## 12. New: how to read the extra graphs 
**Phase portrait (fig11).** The horizontal axis is the angle, the vertical axis is the angular velocity. Each curve is one possible motion.
- Near the top (red dot) the curves are pushed away along one direction and attracted along another. This shape is called a **saddle**. It is the picture of an unstable equilibrium: one positive eigenvalue and one negative.
- Near the bottom (green dots) the curves spiral inwards. The pendulum comes to rest. This is a **stable** equilibrium (damping makes it a spiral).
- With control, the swing-up curve spirals outward, gaining energy, until it reaches the goal and stays there.

**Damping (fig9).** Left: the free swing dies out faster when b is bigger, because damping removes energy. Right: our swing-up pushes with at most 8 m/s^2. For b = 0 and 0.05 it catches the pendulum in about 3.5 s. For b = 0.3 and bigger it never does: damping removes energy faster than our limited push adds it. The balance part is not affected (it is a feedback law, not an energy law). Possible fix (future work): allow a bigger push, or adapt the gain K.

**Robustness to length (fig10).** The gains were tuned for L = 1 m.
- Balance only: it works from L = 0.5 up to 1.6 m (a long pendulum is just slower to settle). At L = 2.0 m it fails, because the cart wanders far outside the track and the acceleration limit is reached.
- Swing-up plus balance: it works up to 1.0 m and fails to catch at 1.3 m and longer.
- Meaning: a longer pendulum is slower, so the same gains and the same catch rules no longer fit. This is the same lesson as the video, where extra weight on the pole was a test of robustness. (In our simple model the mass cancels, so we change the length instead. State this assumption in the report.)

**Numerical versus physical failure.**
- Physical instability: the equation itself pushes the pendulum away from the top (positive eigenvalue). RK4 with a tiny step shows it too. Example: controller OFF.
- Numerical error: the equation is fine but the solver is not accurate enough. Example: Euler with dt = 0.1 never catches the pendulum, while RK4 with the same dt does (figure 8). Rule: if the failure disappears when you make dt smaller or switch to RK4, it was numerical.

## 13. Honest limits of the model (write these in the report)
- The pivot is moved directly by an acceleration a. A real cart has a motor, voltage, friction and a limited rail. We model only the acceleration limit (+/- 25 m/s^2) and the rail ends (warning only).
- The pole is a point mass on a massless rod. Real poles have their own inertia.
- No sensor noise and no time delay.
- Swing-up and catch rules were tuned for L = 1 m (shown in figure 10).
