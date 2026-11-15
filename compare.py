"""
compare.py  -  makes all the graphs for the report (saved in the folder 'figures').
Run:  python compare.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")                      # draw to files, no window needed
import matplotlib.pyplot as plt
from pendulum import *

os.makedirs("figures", exist_ok=True)
plt.rcParams.update({"figure.dpi": 130, "font.size": 10, "axes.grid": True})


# ------------------------------------------------------------
# FIGURE 1  Euler vs RK4 on the free pendulum (3 step sizes)
# ------------------------------------------------------------
# Start: pendulum lifted 60 degrees from hanging, released from rest (no control).
S0 = [0, 0, np.pi - np.radians(60), 0]
T_END = 10.0
t_ref, S_ref, _ = simulate(S0, "rk4", 0.0005, T_END)           # reference: RK4 with a tiny step

fig, axs = plt.subplots(1, 2, figsize=(11, 4))
axs[0].plot(t_ref, np.degrees(wrap(S_ref[:, 2] - np.pi)), "k", lw=2, label="reference (RK4, dt=0.0005)")
for dt, col in [(0.1, "tab:red"), (0.01, "tab:orange"), (0.001, "tab:green")]:
    t, S, _ = simulate(S0, "euler", dt, T_END)
    axs[0].plot(t, np.degrees(wrap(S[:, 2] - np.pi)), color=col, label=f"Euler dt={dt}")
axs[0].set_title("Euler at three step sizes")
axs[1].plot(t_ref, np.degrees(wrap(S_ref[:, 2] - np.pi)), "k", lw=2, label="reference (RK4, dt=0.0005)")
for dt, col in [(0.1, "tab:red"), (0.01, "tab:orange"), (0.001, "tab:green")]:
    t, S, _ = simulate(S0, "rk4", dt, T_END)
    axs[1].plot(t, np.degrees(wrap(S[:, 2] - np.pi)), "--", color=col, label=f"RK4 dt={dt}")
axs[1].set_title("RK4 at three step sizes")
for ax in axs:
    ax.set_xlabel("time [s]"); ax.set_ylabel("angle from hanging position [deg]"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/fig1_euler_vs_rk4_trajectories.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 2  Error vs step size (log-log)  ->  order of the methods
# ------------------------------------------------------------
dts = [0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002]
err = {"euler": [], "rk4": []}
for m in err:
    for dt in dts:
        t, S, _ = simulate(S0, m, dt, T_END)
        k = int(round(T_END / dt))
        kr = int(round(T_END / 0.0005))
        err[m].append(abs(S[k, 2] - S_ref[kr, 2]))              # error in angle at t = 10 s
fig, ax = plt.subplots(figsize=(5.5, 4))
ax.loglog(dts, err["euler"], "o-", label="Euler")
ax.loglog(dts, err["rk4"], "s-", label="RK4")
ax.loglog(dts, [err["euler"][-1] * (d / dts[-1]) for d in dts], ":", color="gray", label="slope 1 (order 1)")
ax.loglog(dts, [err["rk4"][-1] * (d / dts[-1]) ** 4 for d in dts], ":", color="black", label="slope 4 (order 4)")
ax.set_xlabel("step size dt [s]"); ax.set_ylabel("angle error at t = 10 s [rad]")
ax.set_title("Error vs step size"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/fig2_error_vs_step_size.png"); plt.close()
slope = lambda e: np.polyfit(np.log(dts[2:]), np.log(e[2:]), 1)[0]
print(f"measured order: Euler ~ {slope(err['euler']):.2f}, RK4 ~ {slope(err['rk4']):.2f}")

# ------------------------------------------------------------
# FIGURE 3  Energy drift (validation).  With B = 0 energy must stay constant.
# ------------------------------------------------------------
import pendulum as P
b_old = P.B; P.B = 0.0
fig, ax = plt.subplots(figsize=(6, 4))
for m, dt, col in [("euler", 0.01, "tab:red"), ("rk4", 0.01, "tab:green"), ("rk4", 0.1, "tab:blue")]:
    t, S, _ = simulate(S0, m, dt, 30.0)
    E = np.array([energy(s) for s in S])
    ax.plot(t, E - E[0], color=col, label=f"{m.upper()} dt={dt}")
P.B = b_old
ax.set_xlabel("time [s]"); ax.set_ylabel("energy change (should be 0)"); ax.set_title("Energy check with no damping")
ax.legend(fontsize=8); plt.tight_layout(); plt.savefig("figures/fig3_energy_drift.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 4  Validation: small-angle formula vs numerical solution
# ------------------------------------------------------------
phi0 = np.radians(5)                                           # small swing around hanging
b_old = P.B; P.B = 0.0
t, S, _ = simulate([0, 0, np.pi - phi0, 0], "rk4", 0.005, 6.0)
P.B = b_old
exact = phi0 * np.cos(np.sqrt(G / L) * t)                      # phi'' = -(g/L) phi  ->  phi = phi0 cos(w t)
fig, ax = plt.subplots(figsize=(6, 4))
ax.plot(t, np.degrees(np.pi - S[:, 2]), label="RK4 (full nonlinear)")
ax.plot(t, np.degrees(exact), "--", label="formula  phi0 cos(sqrt(g/L) t)")
ax.set_xlabel("time [s]"); ax.set_ylabel("angle from hanging [deg]"); ax.set_title("Validation: small-angle formula")
ax.legend(fontsize=8); plt.tight_layout(); plt.savefig("figures/fig4_validation_small_angle.png"); plt.close()
print("max difference to formula [deg]:", np.degrees(np.abs((np.pi - S[:, 2]) - exact)).max())

# ------------------------------------------------------------
# FIGURE 5  Experiment 1: different Kp (gain) for balancing from 15 degrees
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.5, 4))
for kp in [5, 12, 25, 40, 80]:
    P.KP = kp
    t, S, _ = simulate([0, 0, np.radians(15), 0], "rk4", 0.005, 6.0, Controller("balance"))
    ax.plot(t, np.degrees(wrap(S[:, 2])), label=f"Kp = {kp}")
P.KP = 40.0
ax.axhline(0, color="k", lw=0.8); ax.set_ylim(-60, 60)
ax.set_xlabel("time [s]"); ax.set_ylabel("angle from upright [deg]")
ax.set_title("Experiment 1: effect of Kp (needs Kp > g to stand)"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/fig5_exp1_gain.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 6  Experiment 2: different starting angles (balance mode only)
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.5, 4))
for deg in [5, 15, 30, 45, 60]:
    t, S, _ = simulate([0, 0, np.radians(deg), 0], "rk4", 0.005, 6.0, Controller("balance"))
    ax.plot(t, np.degrees(wrap(S[:, 2])), label=f"start {deg} deg")
ax.axhline(0, color="k", lw=0.8); ax.set_ylim(-70, 70)
ax.set_xlabel("time [s]"); ax.set_ylabel("angle from upright [deg]")
ax.set_title("Experiment 2: how far can the controller catch it?"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/fig6_exp2_start_angle.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 7  Full scenario: hanging -> swing-up -> balance -> push -> recover
# ------------------------------------------------------------
push = lambda t: 30.0 if 12.0 < t < 12.1 else 0.0
c = Controller("swing")
t, S, modes = simulate([0, 0, np.pi - 0.05, 0], "rk4", 0.005, 18.0, c, force=push)
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
ang = np.degrees(wrap(S[:, 2])); ang[1:][np.abs(np.diff(ang)) > 180] = np.nan   # no vertical lines at the +-180 jump
axs[0].plot(t, ang, "tab:blue"); axs[0].set_ylabel("angle [deg]")
axs[0].axhline(0, color="k", lw=0.8)
mode_num = np.array([1 if m == "balance" else 0 for m in modes])
axs[0].fill_between(t, -180, 180, where=mode_num == 1, color="tab:green", alpha=0.12, label="balance mode")
axs[0].set_title("Full scenario: hanging -> swing-up -> balance -> push -> recover"); axs[0].legend(fontsize=8, loc="upper right")
axs[1].plot(t, S[:, 0], "tab:orange"); axs[1].set_ylabel("pivot position x [m]")
axs[2].plot(t, [energy(s) for s in S], "tab:purple"); axs[2].set_ylabel("energy (0 = upright)"); axs[2].set_xlabel("time [s]")
plt.tight_layout(); plt.savefig("figures/fig7_full_scenario.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 8  Numerical failure: the same swing-up and balance with different solver settings
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7.5, 4))
for m, dt, col in [("rk4", 0.005, "tab:green"), ("rk4", 0.1, "tab:orange"), ("euler", 0.1, "tab:red")]:
    t, S, _ = simulate([0, 0, np.pi - 0.05, 0], m, dt, 15.0, Controller("swing"))
    ang = np.degrees(wrap(S[:, 2])); ang[1:][np.abs(np.diff(ang)) > 180] = np.nan
    ax.plot(t, ang, color=col, label=f"{m.upper()} dt={dt}")
ax.axhline(0, color="k", lw=0.8)
ax.set_xlabel("time [s]"); ax.set_ylabel("angle from upright [deg]")
ax.set_title("Same controller and same physics: only the solver changes"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/fig8_numerical_vs_physical_failure.png"); plt.close()


# ------------------------------------------------------------
# FIGURE 9  Experiment 3: damping b  (handbook: "test ... damping")
# ------------------------------------------------------------
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle, Arc, Circle
b_old = P.B
fig, axs = plt.subplots(1, 2, figsize=(11, 4))
for b, col in [(0.0, "tab:blue"), (0.05, "tab:green"), (0.3, "tab:orange"), (1.0, "tab:red")]:
    P.B = b
    t, S, _ = simulate([0, 0, np.pi - np.radians(60), 0], "rk4", 0.005, 20.0)
    axs[0].plot(t, np.degrees(np.pi - S[:, 2]), color=col, label=f"b = {b}")
axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("angle from hanging [deg]")
axs[0].set_title("Free swing: damping removes energy"); axs[0].legend(fontsize=8)
bs = [0.0, 0.05, 0.3, 1.0, 2.0]
catch = []
for b in bs:
    P.B = b
    t, S, md = simulate([0, 0, np.pi - 0.05, 0], "rk4", 0.005, 40.0, Controller("swing"))
    tb = [t[i] for i in range(len(md)) if md[i] == "balance"]
    catch.append(tb[0] if tb else np.nan)
P.B = b_old
bars = axs[1].bar([str(b) for b in bs], [0 if np.isnan(c) else c for c in catch], color="tab:green")
for i, c in enumerate(catch):
    axs[1].text(i, 0.5 if np.isnan(c) else c + 0.3, "no catch" if np.isnan(c) else f"{c:.1f} s", ha="center", fontsize=9,
                color="red" if np.isnan(c) else "black")
axs[1].set_xlabel("damping b [1/s]"); axs[1].set_ylabel("time until the pendulum is caught [s]")
axs[1].set_ylim(0, 4.5); axs[1].set_title("Swing-up with limited push (A_SWING = 8 m/s^2)", fontsize=9); axs[1].grid(False)
plt.tight_layout(); plt.savefig("figures/fig9_exp3_damping.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 10  Experiment 4: robustness to a parameter change (pendulum length L)
# ------------------------------------------------------------
Ls = [0.5, 0.7, 1.0, 1.3, 1.6, 2.0]
res_bal, res_full = [], []
for Lv in Ls:
    P.L = Lv
    t, S, _ = simulate([0, 0, np.radians(10), 0], "rk4", 0.005, 40.0, Controller("balance"))
    res_bal.append(abs(wrap(S[-1, 2])))                # error after 40 s (a long pendulum settles slowly)
    t, S, md = simulate([0, 0, np.pi - 0.05, 0], "rk4", 0.005, 30.0, Controller("swing"))
    res_full.append(abs(wrap(S[-1, 2])) if md[-1] == "balance" else abs(wrap(S[-1, 2])))
P.L = 1.0
fig, ax = plt.subplots(figsize=(6.5, 4))
x = np.arange(len(Ls)); w = 0.38
ok = lambda v: "tab:green" if v < 0.02 else "tab:red"
ax.bar(x - w / 2, np.degrees(res_bal), w, color=[ok(v) for v in res_bal], label="balance only (start 10 deg, after 40 s)")
ax.bar(x + w / 2, np.degrees(res_full), w, color=[ok(v) for v in res_full], alpha=0.55, hatch="//", label="swing-up + balance")
ax.set_xticks(x); ax.set_xticklabels([str(v) for v in Ls])
ax.set_xlabel("pendulum length L [m]  (gains were tuned for L = 1.0)"); ax.set_ylabel("final angle error [deg]")
ax.set_title("Robustness: green = stands up, red = fails"); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig("figures/fig10_exp4_robustness_length.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 11  Phase portrait (angle vs angular velocity)
# ------------------------------------------------------------
fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
for th0 in np.linspace(-3.0, 3.0, 9):
    for w0 in [-6, -3, 0, 3, 6]:
        t, S, _ = simulate([0, 0, th0, w0], "rk4", 0.01, 6.0)
        ang = wrap(S[:, 2]); ang = ang.copy(); ang[1:][np.abs(np.diff(ang)) > 3] = np.nan
        axs[0].plot(ang, S[:, 3], color="tab:blue", lw=0.6, alpha=0.7)
axs[0].plot([0], [0], "ro", label="upright: unstable (saddle)")
axs[0].plot([np.pi, -np.pi], [0, 0], "go", label="hanging: stable (spiral)")
axs[0].set_xlim(-np.pi, np.pi); axs[0].set_ylim(-8, 8)
axs[0].set_xlabel("angle from upright [rad]"); axs[0].set_ylabel("angular velocity [rad/s]")
axs[0].set_title("No control: trajectories move away from the top"); axs[0].legend(fontsize=8, loc="upper right")
t, S, md = simulate([0, 0, np.pi - 0.05, 0], "rk4", 0.005, 12.0, Controller("swing"))
ang = wrap(S[:, 2]).copy(); ang[1:][np.abs(np.diff(ang)) > 3] = np.nan
axs[1].plot(ang, S[:, 3], color="tab:purple")
axs[1].plot([ang[0]], [S[0, 3]], "go", label="start (hanging)"); axs[1].plot([0], [0], "r*", ms=14, label="goal (upright)")
axs[1].set_xlim(-np.pi, np.pi); axs[1].set_ylim(-8, 8)
axs[1].set_xlabel("angle from upright [rad]"); axs[1].set_ylabel("angular velocity [rad/s]")
axs[1].set_title("With control: swing-up spirals into the goal"); axs[1].legend(fontsize=8, loc="upper right")
plt.tight_layout(); plt.savefig("figures/fig11_phase_portrait.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 0  Model diagram (put this first in the report)
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6.2, 5.2))
ax.set_aspect("equal"); ax.axis("off"); ax.grid(False)
th = 0.55; Lp = 1.0
ax.plot([-1.6, 1.6], [0, 0], color="gray", lw=5, solid_capstyle="round")
ax.add_patch(Rectangle((-0.25, -0.1), 0.5, 0.2, color="tab:blue"))
ax.plot([0, 0], [0, 1.25], "k--", lw=1)
bx, by = Lp * np.sin(th), Lp * np.cos(th)
ax.plot([0, bx], [0, by], color="navy", lw=5)
ax.add_patch(Circle((bx, by), 0.07, color="tab:orange"))
ax.add_patch(Arc((0, 0), 0.7, 0.7, theta1=90 - np.degrees(th), theta2=90, color="red", lw=2))
ax.text(0.12, 0.42, "theta", color="red", fontsize=12)
ax.text(0.5 * bx + 0.05, 0.5 * by, "L", fontsize=12)
ax.annotate("", xy=(0.95, -0.28), xytext=(0.3, -0.28), arrowprops=dict(arrowstyle="->", color="red", lw=2))
ax.text(0.3, -0.45, "a  (pivot acceleration = our control)", color="red", fontsize=9)
ax.annotate("", xy=(bx, by - 0.4), xytext=(bx, by - 0.07), arrowprops=dict(arrowstyle="->", color="black", lw=2))
ax.text(bx + 0.08, by - 0.3, "m g", fontsize=11)
ax.text(-1.55, 1.2, "theta = 0: upright\ntheta = pi: hanging", fontsize=9, va="top")
ax.text(-1.55, -0.85, "theta'' = (g/L) sin(theta) - b theta' - (a/L) cos(theta)", fontsize=10)
ax.set_xlim(-1.7, 1.7); ax.set_ylim(-1.0, 1.35); ax.set_title("Physical model: pendulum on a moving pivot (cart)")
plt.tight_layout(); plt.savefig("figures/fig0_model_diagram.png"); plt.close()

# ------------------------------------------------------------
# FIGURE 12  Controller logic (state machine)
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(9, 3.6)); ax.axis("off"); ax.grid(False)
def box(x, y, w, h, txt, col):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02", fc=col, ec="black"))
    ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=9)
box(0.02, 0.35, 0.28, 0.35, "SWING-UP mode\na = K E sign(w cos(th))\n(adds energy)", "#fde3c4")
box(0.62, 0.35, 0.28, 0.35, "BALANCE mode\na = Kp th + Kd w + Kx x + Kv v\n(PD feedback)", "#c9efd2")
ax.annotate("", xy=(0.62, 0.58), xytext=(0.30, 0.58), arrowprops=dict(arrowstyle="->", lw=2))
ax.text(0.46, 0.64, "|th| < 0.35 rad\nand |w| < 3.5 rad/s", ha="center", fontsize=8)
ax.annotate("", xy=(0.30, 0.45), xytext=(0.62, 0.45), arrowprops=dict(arrowstyle="->", lw=2))
ax.text(0.46, 0.30, "|th| > 1.2 rad  (fell)", ha="center", fontsize=8)
ax.text(0.5, 0.92, "Hybrid controller: two modes and two switching rules", ha="center", fontsize=11)
ax.text(0.5, 0.08, "Both modes output the pivot acceleration a, limited to +/- 25 m/s^2 (the hand has limits)", ha="center", fontsize=9)
ax.set_xlim(0, 0.95); ax.set_ylim(0, 1)
plt.tight_layout(); plt.savefig("figures/fig12_controller_logic.png"); plt.close()

# ------------------------------------------------------------
# TABLE  Euler vs RK4 at three step sizes (copy into the report)
# ------------------------------------------------------------
import time
rows = ["| Method | dt [s] | steps | f-evaluations | angle error at t = 10 s [rad] | run time [s] |", "|---|---|---|---|---|---|"]
for m, evals in [("euler", 1), ("rk4", 4)]:
    for dt in [0.1, 0.01, 0.001]:
        t0 = time.time(); t, S, _ = simulate(S0, m, dt, T_END); el = time.time() - t0
        k = int(round(T_END / dt)); kr = int(round(T_END / 0.0005))
        rows.append(f"| {m.upper()} | {dt} | {k} | {k * evals} | {abs(S[k, 2] - S_ref[kr, 2]):.2e} | {el:.2f} |")
open("results_table.md", "w").write("\n".join(rows) + "\n")
print("\n".join(rows))

print("All figures saved in the folder 'figures'.")
