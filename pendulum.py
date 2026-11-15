"""
pendulum.py  -  the model, the two solvers (Euler, RK4) and the controllers.

State vector s = [x, v, th, om]
    x   : position of the pivot (cart) on the track      [m]
    v   : velocity of the pivot                          [m/s]
    th  : pendulum angle measured from UPRIGHT           [rad]  (th = 0 is up, th = pi is hanging down)
    om  : angular velocity                               [rad/s]

The control input is a = acceleration of the pivot [m/s^2]
(this is the "hand moving the broom").
"""
import numpy as np

# ---------------- physical parameters ----------------
G = 9.81        # gravity                [m/s^2]
L = 1.0         # pendulum length        [m]
B = 0.05        # damping (air/friction) [1/s]
A_MAX = 25.0    # the hand cannot accelerate forever: |a| <= A_MAX [m/s^2]
X_MAX = 2.0     # track half-length (used only for the picture / warning)

# ---------------- controller gains ----------------
KP, KD = 40.0, 10.0     # balance: angle and angular-velocity feedback
KX, KV = 3.0, 4.0       # balance: keep the pivot near the centre of the track
K_SWING = 1.0           # swing-up: energy pumping strength
A_SWING = 8.0           # swing-up: largest push while pumping [m/s^2] (gentle = several swings)
CATCH_ANGLE = 0.35      # [rad] switch swing-up -> balance when closer than this to upright
DROP_ANGLE = 1.2        # [rad] switch balance -> swing-up when the pendulum falls further than this


def wrap(angle):
    """Wrap an angle to the range [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


# =====================================================
# 1. THE DIFFERENTIAL EQUATIONS   s' = f(s, a)
# =====================================================
def f(s, a):
    x, v, th, om = s
    return np.array([
        v,                                              # x'  = v
        a,                                              # v'  = a
        om,                                             # th' = om
        (G / L) * np.sin(th) - B * om - (a / L) * np.cos(th),   # om' = ...
    ])


# =====================================================
# 2. THE SOLVERS (written by us, no ready-made ODE solver)
# =====================================================
def euler_step(s, a, dt):
    """Euler: one slope at the start of the step."""
    return s + dt * f(s, a)


def rk4_step(s, a, dt):
    """Runge-Kutta 4: four slopes, weighted average (1, 2, 2, 1)/6."""
    k1 = f(s, a)
    k2 = f(s + 0.5 * dt * k1, a)
    k3 = f(s + 0.5 * dt * k2, a)
    k4 = f(s + dt * k3, a)
    return s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


STEPPERS = {"euler": euler_step, "rk4": rk4_step}


# =====================================================
# 3. ENERGY  (used by swing-up and by the validation)
# =====================================================
def energy(s):
    """Energy per (m L^2), measured so that 'upright at rest' = 0 and 'hanging at rest' = -2 g/L."""
    th, om = s[2], s[3]
    return 0.5 * om ** 2 + (G / L) * (np.cos(th) - 1.0)


# =====================================================
# 4. CONTROLLERS
# =====================================================
def balance_control(s):
    """PD (+ position) feedback: push the pivot toward the side the pendulum leans to."""
    x, v, th, om = s
    th = wrap(th)
    return KP * th + KD * om + KX * x + KV * v


def swing_control(s):
    """Energy pumping: add energy until the pendulum has the energy of the upright position."""
    x, v, th, om = s
    e = energy(s)                          # negative = not enough energy yet; 0 = can just reach the top
    # From dE/dt = -(a/L) om cos(th) - B om^2 :  choose a with the sign that makes dE/dt > 0
    push = np.clip(K_SWING * e, -A_SWING, 0.0) * np.sign(om * np.cos(th))
    centre = -3.0 * x - 2.0 * v            # gently keep the pivot on the track
    return push + centre


class Controller:
    """Chooses the mode (SWING or BALANCE) and returns the pivot acceleration a."""

    def __init__(self, mode="swing"):
        self.mode = mode

    def __call__(self, s):
        th = wrap(s[2])
        if self.mode == "swing" and abs(th) < CATCH_ANGLE and abs(s[3]) < 3.5:
            self.mode = "balance"
        elif self.mode == "balance" and abs(th) > DROP_ANGLE:
            self.mode = "swing"
        a = balance_control(s) if self.mode == "balance" else swing_control(s)
        return float(np.clip(a, -A_MAX, A_MAX))


# =====================================================
# 5. HELPER: run a whole simulation
# =====================================================
def simulate(s0, method, dt, t_end, controller=None, force=None):
    """
    Integrate from s0 to t_end.
    controller : callable s -> a   (None means a = 0, no control)
    force      : optional callable t -> extra pivot acceleration (a 'push' / disturbance)
    Returns times, states (N x 4), modes (list of str).
    """
    step = STEPPERS[method]
    n = int(round(t_end / dt))
    t = np.zeros(n + 1)
    S = np.zeros((n + 1, 4))
    modes = []
    s = np.array(s0, float)
    S[0] = s
    for i in range(n):
        a = controller(s) if controller else 0.0
        if force:
            a += force(t[i])
        modes.append(getattr(controller, "mode", "free"))
        s = step(s, a, dt)
        t[i + 1] = t[i] + dt
        S[i + 1] = s
        if not np.all(np.isfinite(s)) or abs(s[2]) > 1e3:      # solver blew up
            S[i + 1:] = np.nan
            break
    modes.append(modes[-1] if modes else "free")
    return t, S, modes
