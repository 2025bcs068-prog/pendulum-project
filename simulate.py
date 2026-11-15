"""
simulate.py  -  LIVE DEMO: virtual version of the "pole on a cart" laboratory rig
(swing-up, balance, disturbance rejection, robustness to parameter change).

Run:   python simulate.py

KEYS
  R          restart: pendulum hanging down, controller swings it up and balances it (AUTO)
  B          start 15 deg from upright and balance at once (AUTO)
  M          MANUAL mode: YOU balance it. Hold LEFT / RIGHT to accelerate the cart.
  A          AUTO mode (controller on)          O   controller OFF (free pendulum)
  SPACE      push the cart to the right (disturbance)       LEFT / RIGHT  also push in AUTO/OFF mode
  E          switch solver  RK4 <-> Euler       D   switch step size 0.005 s <-> 0.1 s
  [  ]       shorter / longer pendulum (robustness to parameter variation)
  K          change damping b  (0, 0.05, 0.3, 1.0)
  H          show / hide part labels            ESC quit

Every number and every movement on the screen comes from OUR solver in pendulum.py.
No physics engine and no ready-made ODE solver is used.
"""
import collections
import numpy as np
import pygame
import pendulum as P

W, H = 1100, 700
SCALE = 170.0                     # pixels per metre
PIVOT_Y = 430
CX = W // 2
HISTORY_SECONDS = 8.0
MANUAL_A = 12.0                   # cart acceleration while a key is held in MANUAL mode [m/s^2]
DAMPING_CHOICES = [0.0, 0.05, 0.3, 1.0]

WHITE, BLACK, GREY, DGREY = (255, 255, 255), (20, 20, 20), (205, 205, 205), (110, 110, 110)
BLUE, NAVY, RED, GREEN, ORANGE, YELLOW = (40, 90, 200), (25, 60, 150), (200, 40, 40), (30, 150, 70), (230, 130, 20), (245, 200, 30)


class Demo:
    def __init__(self):
        self.method = "rk4"
        self.dt = 0.005
        self.ui = "auto"                       # "auto", "manual" or "off"
        self.show_labels = True
        self.restart_hanging()

    # ---------------- starting situations ----------------
    def reset_common(self):
        self.t = 0.0
        self.acc = 0.0
        self.hist = collections.deque()
        self.events = []
        self.a_now = 0.0
        self.push_until = 0.0
        self.push_dir = 0.0
        self.last_push_t = -99.0
        self.param_changed_t = -99.0
        self.exploded = False
        self.was_up = False
        self.fallen = False
        self.out_of_track = False
        self.last_mode = None

    def log(self, text):
        self.events.append((self.t, text))
        self.events = self.events[-5:]

    def restart_hanging(self):
        self.s = np.array([0.0, 0.0, np.pi - 0.05, 0.0])
        self.ctrl = P.Controller("swing")
        self.ui = "auto"
        self.reset_common()
        self.log("start: hanging down, AUTO mode")

    def start_balance(self):
        self.s = np.array([0.0, 0.0, np.radians(15), 0.0])
        self.ctrl = P.Controller("balance")
        self.ui = "auto"
        self.reset_common()
        self.log("start: 15 deg from upright, AUTO mode")

    def start_manual(self):
        self.s = np.array([0.0, 0.0, np.radians(3), 0.0])
        self.ctrl = P.Controller("balance")
        self.ui = "manual"
        self.reset_common()
        self.log("MANUAL: hold LEFT / RIGHT to keep it up")

    def set_ui(self, mode):
        self.ui = mode
        self.fallen = False
        self.log({"auto": "controller ON (AUTO)", "off": "controller OFF", "manual": "MANUAL mode"}[mode])

    # ---------------- one physics step with OUR solver ----------------
    def step(self, manual_dir):
        if self.ui == "auto":
            a = self.ctrl(self.s)
        elif self.ui == "manual":
            a = float(np.clip(manual_dir * MANUAL_A, -P.A_MAX, P.A_MAX))
        else:
            a = 0.0
        if self.t < self.push_until:
            a += self.push_dir * 30.0
        self.a_now = a
        self.s = P.STEPPERS[self.method](self.s, a, self.dt)
        self.t += self.dt

        # ---- events (the handbook asks us to handle events sensibly) ----
        if not np.all(np.isfinite(self.s)) or abs(self.s[2]) > 1e4:
            if not self.exploded:
                self.log("SOLVER BLEW UP (numerical error)")
            self.exploded = True
            self.s = np.array([0.0, 0.0, np.pi, 0.0])
        th = P.wrap(self.s[2])
        if abs(th) < 0.35:
            self.was_up = True
        if self.ui in ("manual", "off") and self.was_up and abs(th) > 1.2 and not self.fallen:
            self.fallen = True
            self.log(f"FALL DETECTED at t = {self.t:.2f} s")
        if self.ui == "auto" and self.ctrl.mode != self.last_mode:
            if self.last_mode is not None:
                self.log("CAUGHT: swing-up -> BALANCE" if self.ctrl.mode == "balance" else "LOST BALANCE: back to SWING-UP")
            self.last_mode = self.ctrl.mode
        out = abs(self.s[0]) > P.X_MAX
        if out and not self.out_of_track:
            self.log("TRACK LIMIT exceeded")
        self.out_of_track = out

        self.hist.append((self.t, float(np.degrees(th))))
        while self.hist and self.hist[0][0] < self.t - HISTORY_SECONDS:
            self.hist.popleft()

    def push(self, direction):
        self.push_dir = direction
        self.push_until = self.t + 0.1
        self.last_push_t = self.t
        self.log("PUSH (disturbance)")

    def advance(self, real_dt, manual_dir=0.0):
        self.acc += min(real_dt, 0.05)
        while self.acc >= self.dt:
            self.step(manual_dir)
            self.acc -= self.dt

    def change_length(self, delta):
        P.L = float(np.clip(round(P.L + delta, 2), 0.5, 2.0))
        self.param_changed_t = self.t
        self.log(f"parameter change: L = {P.L} m")

    def cycle_damping(self):
        i = DAMPING_CHOICES.index(P.B) if P.B in DAMPING_CHOICES else 0
        P.B = DAMPING_CHOICES[(i + 1) % len(DAMPING_CHOICES)]
        self.param_changed_t = self.t
        self.log(f"parameter change: damping b = {P.B}")

    def banner(self):
        """Text like the lower-third captions in the laboratory video."""
        if self.exploded:
            return "Numerical failure: the solver blew up", RED
        if self.fallen:
            return "FALL DETECTED", RED
        if self.t - self.param_changed_t < 8:
            return f"Robustness to parameter variations:  L = {P.L} m,  b = {P.B}", ORANGE
        if self.t - self.last_push_t < 3:
            return "Disturbance rejection during balancing", ORANGE
        if self.ui == "manual":
            return "Manual control: hold LEFT / RIGHT", BLUE
        if self.ui == "off":
            return "Controller OFF: free pendulum", DGREY
        if self.ctrl.mode == "swing":
            return "Swing-up stage", BLUE
        return "Balancing in the upright position", GREEN


# =====================================================================
#  DRAWING
# =====================================================================
def label(screen, small, text, target, offset):
    tx, ty = target
    ex, ey = tx + offset[0], ty + offset[1]
    pygame.draw.line(screen, DGREY, (tx, ty), (ex, ey), 1)
    surf = small.render(text, True, BLACK)
    rect = surf.get_rect(center=(ex, ey - 9)) if offset[1] < 0 else surf.get_rect(center=(ex, ey + 9))
    pygame.draw.rect(screen, (255, 255, 235), rect.inflate(8, 4))
    pygame.draw.rect(screen, DGREY, rect.inflate(8, 4), 1)
    screen.blit(surf, rect)


def draw(screen, fonts, d):
    font, big, small, huge = fonts
    screen.fill(WHITE)
    x, v, th, om = d.s
    L = P.L

    # ---- rail (two rods), end stops and the DC motor ----
    half = P.X_MAX * SCALE + 45
    for off in (-9, 9):
        pygame.draw.line(screen, DGREY, (CX - half, PIVOT_Y + off), (CX + half, PIVOT_Y + off), 5)
    for sx in (-1, 1):
        pygame.draw.rect(screen, (150, 150, 150), (CX + sx * half - 8, PIVOT_Y - 30, 16, 60), border_radius=3)
        pygame.draw.line(screen, RED, (CX + sx * P.X_MAX * SCALE, PIVOT_Y - 24), (CX + sx * P.X_MAX * SCALE, PIVOT_Y + 24), 3)
    pygame.draw.rect(screen, (60, 60, 70), (CX + half + 6, PIVOT_Y - 26, 58, 52), border_radius=6)
    pygame.draw.circle(screen, (150, 150, 160), (int(CX + half + 35), PIVOT_Y), 13)

    # ---- cart, pole, tip weight ----
    px = CX + x * SCALE
    bx = px + SCALE * L * np.sin(th)
    by = PIVOT_Y - SCALE * L * np.cos(th)
    pygame.draw.rect(screen, (235, 235, 240), (px - 34, PIVOT_Y - 16, 68, 32), border_radius=5)
    pygame.draw.rect(screen, DGREY, (px - 34, PIVOT_Y - 16, 68, 32), 2, border_radius=5)
    pygame.draw.circle(screen, (90, 90, 100), (int(px - 20), int(PIVOT_Y - 20)), 8)      # encoder
    pygame.draw.line(screen, NAVY, (px, PIVOT_Y), (bx, by), 8)
    tip = (GREEN if d.ui == "auto" and d.ctrl.mode == "balance" else ORANGE if d.ui == "auto" else RED if d.ui == "off" else BLUE)
    pygame.draw.circle(screen, YELLOW, (int(bx), int(by)), 15)
    pygame.draw.circle(screen, tip, (int(bx), int(by)), 15, 4)
    pygame.draw.circle(screen, BLACK, (int(px), PIVOT_Y), 5)
    if d.push_until > d.t:
        pygame.draw.line(screen, RED, (px, PIVOT_Y + 46), (px + 80 * d.push_dir, PIVOT_Y + 46), 7)
    if d.ui == "manual":                                                                  # arrows for the keys held
        keys = pygame.key.get_pressed()
        dirn = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)
        if dirn:
            pygame.draw.line(screen, BLUE, (px, PIVOT_Y + 46), (px + 80 * dirn, PIVOT_Y + 46), 7)

    if d.show_labels:
        label(screen, small, "cart assembly", (px + 30, PIVOT_Y - 16), (110, -70))
        label(screen, small, "pendulum", ((px + bx) / 2, (PIVOT_Y + by) / 2), (-110, -10))
        label(screen, small, "DC motor driving the cart", (CX + half + 35, PIVOT_Y - 26), (0, -60))
        label(screen, small, "linear guide (rail)", (CX - half + 60, PIVOT_Y - 9), (0, -75))

    # ---- status text ----
    mode_txt = {"auto": d.ctrl.mode.upper() if True else "", "manual": "MANUAL", "off": "CONTROLLER OFF"}[d.ui]
    lines = [
        f"time  {d.t:6.2f} s",
        f"mode  {mode_txt}",
        f"solver  {d.method.upper()}    dt = {d.dt} s",
        f"L = {P.L} m    damping b = {P.B}",
        f"acceleration a = {d.a_now:6.1f} m/s^2",
        f"energy (0 = upright) {P.energy(d.s):6.2f}",
    ]
    for i, txt in enumerate(lines):
        screen.blit(font.render(txt, True, BLACK), (18, 14 + 25 * i))
    # event log (the "event handling" part of the marks)
    screen.blit(small.render("EVENTS", True, DGREY), (18, 175))
    for i, (tt, txt) in enumerate(reversed(d.events)):
        col = RED if ("FALL" in txt or "BLEW" in txt or "TRACK" in txt or "LOST" in txt) else (GREEN if "CAUGHT" in txt else BLACK)
        screen.blit(small.render(f"{tt:6.2f}s  {txt}", True, col), (18, 193 + 18 * i))

    # ---- monitor like the one in the video ----
    mx, my, mw, mh = W - 250, 14, 236, 150
    pygame.draw.rect(screen, (25, 25, 35), (mx, my, mw, mh), border_radius=8)
    pygame.draw.rect(screen, (120, 120, 130), (mx, my, mw, mh), 3, border_radius=8)
    screen.blit(small.render("SCOPE  (values from our solver)", True, (180, 180, 190)), (mx + 10, my + 8))
    screen.blit(small.render("angle [deg]", True, (150, 220, 150)), (mx + 10, my + 32))
    screen.blit(huge.render(f"{np.degrees(P.wrap(th)):7.1f}", True, (90, 255, 120)), (mx + 70, my + 28))
    screen.blit(small.render("cart x [m]", True, (240, 150, 150)), (mx + 10, my + 90))
    screen.blit(huge.render(f"{x:7.2f}", True, (255, 110, 110)), (mx + 70, my + 86))

    # ---- lower-third caption ----
    text, colour = d.banner()
    surf = big.render(text, True, WHITE)
    bar = pygame.Rect(18, 478, surf.get_width() + 36, 40)
    pygame.draw.rect(screen, colour, bar, border_radius=4)
    screen.blit(surf, (bar.x + 18, bar.y + 4))

    # ---- live graph ----
    gx, gy, gw, gh = 18, 532, W - 36, 138
    pygame.draw.rect(screen, (245, 245, 245), (gx, gy, gw, gh))
    pygame.draw.rect(screen, GREY, (gx, gy, gw, gh), 1)
    pygame.draw.line(screen, GREY, (gx, gy + gh // 2), (gx + gw, gy + gh // 2), 1)
    screen.blit(small.render("angle from upright (-180 to +180 deg), last 8 s", True, BLACK), (gx + 6, gy + 4))
    pts = [(gx + gw * (tt - (d.t - HISTORY_SECONDS)) / HISTORY_SECONDS, gy + gh / 2 - (aa / 180.0) * (gh / 2 - 6)) for tt, aa in d.hist]
    seg = pts[:1]
    for p, q in zip(pts[:-1], pts[1:]):
        if abs(p[1] - q[1]) > gh * 0.6:
            if len(seg) > 1:
                pygame.draw.lines(screen, BLUE, False, seg, 2)
            seg = []
        seg.append(q)
    if len(seg) > 1:
        pygame.draw.lines(screen, BLUE, False, seg, 2)

    help_txt = "R hanging start | B balance start | M manual | A auto | O off | SPACE push | E Euler/RK4 | D step size | [ ] length | K damping | H labels | ESC"
    screen.blit(small.render(help_txt, True, (90, 90, 90)), (18, H - 22))


def make_fonts():
    return (pygame.font.SysFont("consolas,dejavusansmono,monospace", 19),
            pygame.font.SysFont("arial,dejavusans", 24, bold=True),
            pygame.font.SysFont("arial,dejavusans", 14),
            pygame.font.SysFont("consolas,dejavusansmono,monospace", 34, bold=True))


def main():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Inverted pendulum on a cart: swing-up and balance (Euler vs RK4)")
    fonts = make_fonts()
    clock = pygame.time.Clock()
    d = Demo()
    running = True
    while running:
        real_dt = clock.tick(60) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                k = e.key
                if k == pygame.K_ESCAPE:
                    running = False
                elif k == pygame.K_SPACE:
                    d.push(+1.0)
                elif k in (pygame.K_LEFT, pygame.K_RIGHT) and d.ui != "manual":
                    d.push(+1.0 if k == pygame.K_RIGHT else -1.0)
                elif k == pygame.K_r:
                    d.restart_hanging()
                elif k == pygame.K_b:
                    d.start_balance()
                elif k == pygame.K_m:
                    d.start_manual()
                elif k == pygame.K_a:
                    d.set_ui("auto")
                elif k == pygame.K_o:
                    d.set_ui("off")
                elif k == pygame.K_e:
                    d.method = "euler" if d.method == "rk4" else "rk4"
                elif k == pygame.K_d:
                    d.dt = 0.1 if d.dt < 0.05 else 0.005
                elif k == pygame.K_LEFTBRACKET:
                    d.change_length(-0.1)
                elif k == pygame.K_RIGHTBRACKET:
                    d.change_length(+0.1)
                elif k == pygame.K_k:
                    d.cycle_damping()
                elif k == pygame.K_h:
                    d.show_labels = not d.show_labels
        keys = pygame.key.get_pressed()
        manual_dir = (1.0 if keys[pygame.K_RIGHT] else 0.0) - (1.0 if keys[pygame.K_LEFT] else 0.0)
        d.advance(real_dt, manual_dir)
        draw(screen, fonts, d)
        pygame.display.flip()
    P.L, P.B = 1.0, 0.05                     # leave the module as we found it
    pygame.quit()


if __name__ == "__main__":
    main()
