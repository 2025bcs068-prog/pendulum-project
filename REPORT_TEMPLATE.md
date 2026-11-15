# Report template (1 to 3 pages, follow the handbook)
Write this in your own words. Every group member must be able to explain every sentence.

**Title:** Swing-up and balancing of an inverted pendulum on a cart: Euler versus RK4
**Group members and registration numbers:**

## 1. Problem and objectives (3 to 4 lines)
Keep a pendulum upright, starting from hanging down, using a controlled pivot acceleration. Compare Euler and RK4.

## 2. Model (about half a page)
- Figure: fig0 (model diagram)
- State variables, parameters with units (g, L, b, A_MAX), initial conditions
- Assumptions (point mass, massless rod, pivot acceleration as input, limits)
- Equation: theta'' = (g/L) sin(theta) - b theta' - (a/L) cos(theta); first-order system s' = f(s, a) with s = [x, v, theta, omega]
- Stability: linearization, eigenvalues +/- sqrt(g/L), why PD with Kp > g works

## 3. Methods (about quarter page)
- Euler and RK4 formulas, order 1 and 4
- Controllers: PD balance, energy swing-up, switching rules (fig12)
- Events: fall, catch, track limit, solver failure

## 4. Results (about one page: figures with 2 or 3 sentences each)
| Figure | Say what it proves |
|---|---|
| fig1 and results_table.md | three step sizes against the reference |
| fig2 | measured orders (about 1.3 and 3.9) |
| fig5 and fig6 | gain and starting-angle experiments, physical meaning |
| fig9 and fig10 | damping and length experiments |
| fig7 and fig11 | full scenario and phase portrait |

## 5. Validation and limitations (about quarter page)
- Small-angle formula (fig4), energy check (fig3), reference solution
- Numerical versus physical failure (fig8)
- Limits listed in GUIDE section 16

## 6. Conclusion (3 lines) and future work
Real motor model, sensor noise, adaptive gains for different lengths.

## 7. How to run
Copy the Setup and Run sections of README.md.
