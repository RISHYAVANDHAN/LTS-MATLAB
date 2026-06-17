# LTS-MATLAB
Lap-time sim using MATLAB
## Theory, Mathematics, and Current Implementation

A quasi‑steady‑state (QSS) lap‑time simulator following the recipe of Mike Law's
LinkedIn post on vehicle dynamics. This document specifies the full mathematics of
what is currently implemented, file by file, and ends with the known gaps
relative to a fully connected vehicle/tyre‑derived solver.

---

## 1. Pipeline overview

The simulator computes a feasible speed profile $v(s)$ along a track arc length
parameter $s$, then integrates $\mathrm{d}t = \mathrm{d}s/v$ to get a lap time.
The actual pipeline driven by `run_laptime_sim.m` is:

```
init_params  ──►  track_loader  ──►  ggv_envelope  ──►  compute_speed_limits
                                                              │
                          ┌─────────────────────────┐         │
                          │                         │         ▼
                          │              lap_forward_pass  v_lim(s)
                          │                         │
                          │              lap_backward_pass
                          │                         │
                          ▼                         ▼
                          └────────►  lap_merge  ◄──┘
                                          │
                                          ▼
                                  compute_lap_time   ──►  T_lap
```

Two side scripts, `simulate_straight.m` and `simulate_corner.m`, drive the
time‑domain bicycle model `bicycle_rhs.m` (with `tire_forces.m`) directly via
`ode45`. They exist as **unit tests for the vehicle model** and do **not**
contribute to the lap‑time chain in the current build.

The seven blocks below correspond to the seven sections of code, plus the two
side tests.

---

## 2. Parameter set (`init_params.m`)

A single struct `p` holds everything. The relevant groups:

**Mass and inertia.** $m = 300\ \mathrm{kg}$, $I_z = 120\ \mathrm{kg\,m^2}$.

**Geometry.** Front and rear semi‑wheelbases $a = 0.80\ \mathrm{m}$,
$b = 0.70\ \mathrm{m}$, so the wheelbase is $L = a+b = 1.50\ \mathrm{m}$.
CG height $h = 0.25\ \mathrm{m}$. Gravity $g = 9.81\ \mathrm{m/s^2}$.

**Wheel.** Effective rolling radius $R_w = 0.228\ \mathrm{m}$ and per‑wheel
spin inertia $J_w = 1.20\ \mathrm{kg\,m^2}$.

**Aero (currently zeroed).** Drag area $C_d A = 0$, downforce area $C_l A = 0$,
$\rho = 1.225\ \mathrm{kg/m^3}$.

**Tyre.** Linear stiffnesses $C_{\alpha f}=C_{\alpha r}=60\,000\ \mathrm{N/rad}$
(lateral), $C_{xf}=C_{xr}=8\,000\ \mathrm{N}$ (longitudinal). Peak friction
$\mu_{xf}=\mu_{yf}=\mu_{xr}=\mu_{yr}=1.80$. Saturation shape parameters
$B_x=10,\ B_y=8,\ S_x=1,\ S_y=1$. Velocity guard $\varepsilon_v = 10^{-3}$.

**Actuator limits.** $\delta_{\max}=18^\circ$, $\dot\delta_{\max}=300^\circ/\mathrm{s}$,
drive torque $T_{\text{drive,max}}=350\ \mathrm{N\,m}$, brake torque
$T_{\text{brake,max}}=500\ \mathrm{N\,m}$.

**Static axle loads.** With weight transfer disabled,

$$
F_{zf,0} = \frac{m g\, b}{L},\qquad
F_{zr,0} = \frac{m g\, a}{L}.
$$

For the default parameters $F_{zf,0}\approx 1373\ \mathrm{N}$ and
$F_{zr,0}\approx 1570\ \mathrm{N}$.

**Track / numerical.** Spatial step $\Delta s = 0.25\ \mathrm{m}$, default
half‑width $1.5\ \mathrm{m}$, default time step $\mathrm{d}t = 0.01\ \mathrm{s}$.

---

## 3. Track geometry (`track_loader.m`)

The input is an $N\times 2$ array of $(x,y)$ centreline points. The function
delivers an arc‑length parameterised, smooth, optionally closed track with
heading and curvature.

**(a) Raw arc length.** Define cumulative segment lengths

$$
\Delta s_k = \sqrt{(x_{k+1}-x_k)^2 + (y_{k+1}-y_k)^2},\qquad
s_k = \sum_{i=1}^{k-1}\Delta s_i,\qquad L=s_{N}.
$$

**(b) Spline reparameterisation.** A new uniform arc‑length grid
$s\in[0,L]$ with $N_s = \mathtt{SmoothFactor}\cdot(N-1)$ samples is created
and the path is interpolated with shape‑preserving cubic Hermite (PCHIP):

$$
x_s(s) = \mathrm{PCHIP}(s_{\text{raw}},\,x;\,s),\quad
y_s(s) = \mathrm{PCHIP}(s_{\text{raw}},\,y;\,s).
$$

**(c) Heading and curvature.** Using MATLAB's centred-difference `gradient`, the derivatives of the smoothed coordinates with respect to arc length are first computed. The heading angle and curvature are then given by

$$
\psi(s) =
\mathrm{atan2}\!\left(
\frac{dy_s}{ds},
\frac{dx_s}{ds}
\right),
\qquad
\kappa(s) =
\frac{d\psi}{ds}.
$$

Because the path is parameterised by arc length, $|\mathrm{d}\boldsymbol r/\mathrm{d}s|\equiv 1$,
so the standard curvature formula reduces to this single derivative.

**(d) Segment midpoints.** Useful for solver checks:

$$
s_{\text{mid},k} = \tfrac12(s_k+s_{k+1}),\qquad
\kappa_{\text{mid},k} = \tfrac12(\kappa_k+\kappa_{k+1}).
$$

**(e) Boundaries.** The unit normal to the path (in the direction of "left") is
$\mathbf{n} = (-\sin\psi,\,\cos\psi)$. With a constant half‑width
$w_{1/2}=1.5\ \mathrm{m}$ (a placeholder, **not** taken from `p.trackWidth`):

$$
\mathbf r_{\text{left}}  = \mathbf r + w_{1/2}\,\mathbf n,\qquad
\mathbf r_{\text{right}} = \mathbf r - w_{1/2}\,\mathbf n.
$$

If the input does not close, the loader appends the first point so that
$\mathbf r_0=\mathbf r_N$.

---

## 4. Tyre model (`tire_forces.m`)

A practical nonlinear saturating model. **Inputs** per axle:
normal load $F_z$, slip angle $\alpha$, longitudinal slip ratio $\kappa$,
parameter struct $p$, axle tag.

**(a) Linear seed.** For the requested axle (with axle‑specific stiffnesses
$C_x,\,C_y$ and peak frictions $\mu_x,\,\mu_y$):

$$
F_{x,\text{lin}} = C_x\,\kappa,\qquad
F_{y,\text{lin}} = -C_y\,\alpha.
$$

The sign convention is the standard one: positive slip angle produces a
negative side force (the tyre opposes the lateral drift).

**(b) Smooth saturation.** Each axis is squashed onto the friction cap with
$\tanh$:

$$
F_{x,\text{pure}} = \mu_x F_z\;\tanh\!\left(\frac{S_x\,F_{x,\text{lin}}}{\mu_x F_z + \varepsilon}\right),
\qquad
F_{y,\text{pure}} = \mu_y F_z\;\tanh\!\left(\frac{S_y\,F_{y,\text{lin}}}{\mu_y F_z + \varepsilon}\right),
$$

where $\varepsilon = 10^{-9}$ guards against $F_z\to 0$.
At low slip this reproduces the linear law; at large slip it saturates at
$\pm\mu F_z$. Sharpness is tunable via $S_x,\,S_y$.

**(c) Friction‑ellipse combined slip.** Normalised demands

$$
\hat F_x = \frac{F_{x,\text{pure}}}{\mu_x F_z},\qquad
\hat F_y = \frac{F_{y,\text{pure}}}{\mu_y F_z},\qquad
\lambda = \sqrt{\hat F_x^2 + \hat F_y^2}.
$$

If $\lambda \leq 1$ the demand is inside the ellipse and forces are accepted as
is; otherwise both are scaled down to land on the ellipse:

$$
(F_x,F_y) =
\begin{cases}
(F_{x,\text{pure}},\,F_{y,\text{pure}}) & \lambda \leq 1\\[2pt]
\dfrac{1}{\lambda}(F_{x,\text{pure}},\,F_{y,\text{pure}}) & \lambda > 1
\end{cases}
$$

**(d) Final clamp.** As a safety net the output is hard‑capped to $\pm\mu F_z$
on each axis.

This is **not** a Magic‑Formula tyre, but it has the three essential traits:
linear seed, smooth saturation, and combined‑slip coupling.

---

## 5. Bicycle vehicle model (`bicycle_rhs.m`)

An 8‑state time‑domain model for the time‑domain unit tests and (in future) for
the trim optimiser. **Currently not called by the lap pipeline.**

### 5.1 State and inputs

$$
\mathbf x = \begin{bmatrix} u\\ v\\ r\\ \omega_f\\ \omega_r\\ X\\ Y\\ \psi\end{bmatrix},
\qquad
\mathbf u_{\text{in}} = \begin{bmatrix} \delta\\ T_f\\ T_r\end{bmatrix}.
$$

Symbols: $u$ longitudinal body velocity, $v$ lateral body velocity, $r$ yaw
rate, $\omega_f,\omega_r$ front/rear wheel spin rates, $(X,Y,\psi)$ global pose,
$\delta$ front steer angle, $T_f,T_r$ front/rear wheel torques.

### 5.2 Axle velocities

Treating the CG as a single rigid body in plane motion,

$$
u_f = u,\;\; v_f = v + a r,\qquad u_r = u,\;\; v_r = v - b r.
$$

### 5.3 Slip angles

The front slip is measured relative to the *steered* wheel; the rear relative
to the body:

$$
\alpha_f = \operatorname{atan2}\!\left(v_f,\;\max(|u_f|,\varepsilon_v)\right) - \delta,\qquad
\alpha_r = \operatorname{atan2}\!\left(v_r,\;\max(|u_r|,\varepsilon_v)\right).
$$

### 5.4 Longitudinal slip ratios

With wheel spin speed $\omega_\cdot$ and effective radius $R_w$,

$$
\kappa_f = \frac{R_w \omega_f - u_f}{\max(|u_f|,\varepsilon_v)},\qquad
\kappa_r = \frac{R_w \omega_r - u_r}{\max(|u_r|,\varepsilon_v)}.
$$

### 5.5 Normal loads

Static only in the current version:

$$
F_{zf} = F_{zf,0},\qquad F_{zr} = F_{zr,0}.
$$

There is a placeholder for later longitudinal weight transfer
$\Delta F_z = m a_x h/L$.

### 5.6 Tyre force evaluation

$$
(F_{xf},F_{yf}) = \texttt{tire\_forces}(F_{zf},\alpha_f,\kappa_f,p,\text{`front'}),\quad
(F_{xr},F_{yr}) = \texttt{tire\_forces}(F_{zr},\alpha_r,\kappa_r,p,\text{`rear'}).
$$

### 5.7 Front forces rotated into body frame

Because $F_{xf},F_{yf}$ are expressed in the wheel frame at steer angle $\delta$,
they are rotated:

$$
F_{xf}^{\text{body}} = F_{xf}\cos\delta - F_{yf}\sin\delta,\qquad
F_{yf}^{\text{body}} = F_{yf}\cos\delta + F_{xf}\sin\delta.
$$

The rear axle is not steered so $F_{xr}^{\text{body}}=F_{xr}$,
$F_{yr}^{\text{body}}=F_{yr}$.

### 5.8 Rigid‑body equations of motion

In the rotating body frame (so $\mathbf v$ has Coriolis‑style cross terms):

$$
\dot u = \frac{F_{xf}^{\text{body}} + F_{xr}^{\text{body}}}{m} + v r,
\qquad
\dot v = \frac{F_{yf}^{\text{body}} + F_{yr}^{\text{body}}}{m} - u r,
$$

$$
\dot r = \frac{a\,F_{yf}^{\text{body}} - b\,F_{yr}^{\text{body}}}{I_z}.
$$

### 5.9 Wheel spin dynamics

For each wheel a torque balance about the spin axis:

$$
\dot\omega_f = \frac{T_f - R_w F_{xf}}{J_w},\qquad
\dot\omega_r = \frac{T_r - R_w F_{xr}}{J_w}.
$$

Positive $T_\cdot$ accelerates the wheel; the tyre's longitudinal reaction
provides braking torque on the wheel through $R_w F_x$.

### 5.10 Global pose kinematics

$$
\dot X = u\cos\psi - v\sin\psi,\quad
\dot Y = u\sin\psi + v\cos\psi,\quad
\dot\psi = r.
$$

The packed derivative $\dot{\mathbf x}$ is what `ode45` consumes inside
`simulate_straight.m` and `simulate_corner.m`.

---

## 6. GGV envelope (`ggv_envelope.m`) — current implementation

This block is the **largest divergence from a "proper" QSS sim** and is the
subject of the gap analysis. The current code does **not** solve a trim problem
over the bicycle model. Instead, for each speed in a grid
$v\in[0,40]\ \mathrm{m/s}$ (250 points by default), it computes:

**(a) Speed‑dependent aero terms.**

$$
F_{z,\text{aero}}(v) = \tfrac12\rho\,C_l A\,v^2,\qquad
F_{d,\text{aero}}(v) = \tfrac12\rho\,C_d A\,v^2.
$$

With the default parameters both are zero, but they are kept symbolic.

**(b) Axle loads with aero downforce split 50/50.**

$$
F_{zf}(v) = F_{zf,0} + \tfrac12 F_{z,\text{aero}}(v),\qquad
F_{zr}(v) = F_{zr,0} + \tfrac12 F_{z,\text{aero}}(v).
$$

**(c) Force capacities — friction circle / box, not optimiser.**

$$
\begin{aligned}
F_{x,\text{drive}}^{\max}(v) &= \max\!\big(\mu_{xf}F_{zf} + \mu_{xr}F_{zr} - F_{d,\text{aero}},\,0\big),\\[2pt]
F_{x,\text{brake}}^{\max}(v) &= \max\!\big(\mu_{xf}F_{zf} + \mu_{xr}F_{zr} + F_{d,\text{aero}},\,0\big),\\[2pt]
F_{y}^{\max}(v) &= \max\!\big(\mu_{yf}F_{zf} + \mu_{yr}F_{zr},\,0\big).
\end{aligned}
$$

The signs in the longitudinal capacities follow from drag opposing motion: drag
**subtracts** from net drive force but **assists** braking.

**(d) Accelerations.** Divide by mass:

$$
a_x^+(v) = \frac{F_{x,\text{drive}}^{\max}(v)}{m},\quad
a_x^-(v) = \frac{F_{x,\text{brake}}^{\max}(v)}{m},\quad
a_y^{\max}(v) = \frac{F_y^{\max}(v)}{m}.
$$

The output struct `ggv` stores $v$, $a_x^\pm$, $a_y^{\max}$, and the underlying
force capacities and axle loads as per‑speed diagnostics.

This formulation is a **friction box scaled by static + aero‑adjusted loads**.
It is independent of slip angles, slip ratios, the bicycle model, and the
nonlinear tyre force law. Mike's "shortcut if you don't mind taking shortcuts"
clause is what is being used here.

---

## 7. Curvature speed limits (`compute_speed_limits.m`)

For steady cornering at speed $v$ on a path of curvature $\kappa$, the lateral
acceleration demand is $a_y = v^2|\kappa|$. Imposing $a_y \leq a_y^{\max}$,

$$
v_{\text{lim}}(s) = \min\!\left(\sqrt{\frac{a_y^{\text{ref}}}{|\kappa(s)|}},\; v_{\text{cap}}\right),
\qquad |\kappa|\geq 10^{-10}.
$$

On straights ($|\kappa|<10^{-10}$) the cap is just $v_{\text{cap}}=\max(\mathtt{ggv.v})$.

**Choice of $a_y^{\text{ref}}$.** The implementation uses

$$
a_y^{\text{ref}} = \min_{v\in\mathtt{ggv.v}}\;a_y^{\max}(v).
$$

This is conservative — it picks the *worst* lateral grip over the whole speed
range. While aero is off the GGV is flat in speed, so this is harmless; once
$C_l A > 0$ this becomes incorrect and the relation $v_{\text{lim}}^2|\kappa| = a_y^{\max}(v_{\text{lim}})$
becomes implicit and needs a fixed‑point solve per node.

---

## 8. Forward acceleration pass (`lap_forward_pass.m`)

Starting at the first node with $v(0)=\min(v_{\text{lim}}(0),\,0.5)\ \mathrm{m/s}$,
march along the track and apply the maximum available longitudinal
acceleration at each step:

$$
a_{\max}(v) = \max\!\big(\mathrm{interp1}(\mathtt{ggv.v},\,a_x^+,\,v_i),\,0\big),
$$

$$
v_{i+1}^{\text{trial}} = \sqrt{\max\!\big(v_i^2 + 2\,a_{\max}(v_i)\,\Delta s_i,\,0\big)},
$$

$$
v_{i+1} = \min\!\big(v_{i+1}^{\text{trial}},\,v_{\text{lim}}(i+1)\big).
$$

The first equation interpolates the GGV's positive‑$x$ envelope at the current
speed. The second is the standard "constant acceleration over a spatial step"
kinematic relation, equivalent to applying $v\,\mathrm{d}v = a_x\,\mathrm{d}s$.
The third clips to the curvature limit at the next node.

Cumulative segment time uses the mean speed of the segment:

$$
t_{i+1} = t_i + \frac{\Delta s_i}{\max\!\big(\tfrac12(v_i+v_{i+1}),\,10^{-3}\big)}.
$$

---

## 9. Backward braking pass (`lap_backward_pass.m`)

Identical structure but marched from the end of the track backwards. The
braking acceleration is interpolated from $a_x^-$ and is treated as a positive
magnitude in the kinematic update:

$$
a_{\text{brake}}(v) = \max\!\big(\mathrm{interp1}(\mathtt{ggv.v},\,a_x^-,\,v_i),\,0\big),
$$

$$
v_{i-1} = \min\!\Big(\sqrt{v_i^2 + 2\,a_{\text{brake}}(v_i)\,\Delta s_{i-1}},\;v_{\text{lim}}(i-1)\Big).
$$

The sign on $a_x$ reported as $-a_{\text{brake}}$ is stored so the merged
profile reads with the convention "positive acceleration, negative deceleration".

**Initialisation.** Currently $v_N = \min(v_{\text{lim}}(N),\, 0.5\,\max v_{\text{lim}})$.
This is heuristic — a proper closed‑track sim should iterate so
$v_0 \approx v_N$.

---

## 10. Merge and active bound (`lap_merge.m`)

The forward pass enforces acceleration feasibility, the backward pass enforces
braking feasibility, so the feasible speed is the pointwise minimum:

$$
v(s_i) = \min\!\big(v^{\text{fwd}}(s_i),\;v^{\text{bwd}}(s_i)\big).
$$

For diagnostics the code tags each node by which bound is active:

```
source(i) = "forward"   if v_fwd(i) < v_bwd(i)
source(i) = "backward"  if v_bwd(i) < v_fwd(i)
source(i) = "both"      if equal (typically apex of a corner)
```

A finite‑difference longitudinal acceleration is recovered from $v(s)$ via the
energy relation $\mathrm{d}(v^2)/\mathrm{d}s = 2 a_x$:

$$
a_x(s_i) \approx \frac{v_{i+1}^2 - v_i^2}{2\,\Delta s_i}.
$$

Cumulative time is recomputed on the merged profile by the same trapezoidal
rule as in the forward/backward passes.

---

## 11. Lap time (`compute_lap_time.m`)

$$
T_{\text{lap}} = \sum_{i=1}^{N-1}\frac{\Delta s_i}{\max\!\big(\tfrac12(v_i+v_{i+1}),\,10^{-3}\big)}.
$$

This is the trapezoidal approximation of $T = \int_0^L \mathrm{d}s/v(s)$. The
$10^{-3}$ floor prevents the integrand from blowing up at zero speed (which can
happen at the very first node).

---

## 12. Vehicle‑model unit tests

### 12.1 `simulate_straight.m`

Initial state $\mathbf x_0 = (0.5,\,0,\,0,\,0.5/R_w,\,0.5/R_w,\,0,\,0,\,0)$,
constant input $(\delta,T_f,T_r) = (0,\,0,\,120)\ \mathrm{N\,m}$ rear drive,
integrated with `ode45` over $t\in[0,8]\ \mathrm{s}$ at tolerances
$\text{RelTol}=10^{-8}$, $\text{AbsTol}=10^{-10}$. Reports final $u$, $v$, $r$,
$Y$ and plots $u(t)$, $|\mathbf V|(t)$, $v(t)$, $r(t)$, $(X,Y)$. Pass criteria:
$v\to 0$, $r\to 0$, $Y\to 0$, $u$ grows monotonically.

### 12.2 `simulate_corner.m`

Same model, but $u_0 = 5\ \mathrm{m/s}$ and a constant steer
$\delta = 25^\circ$ with $T_r = 80\ \mathrm{N\,m}$. Pass criteria: $r\neq 0$,
$v\neq 0$, curved path in $(X,Y)$. These are not lap‑time tests — they confirm
sign conventions and stability of `bicycle_rhs.m` in isolation.

---

## 13. Solver conventions, numerical guards, and units

* Arc length $s$ in metres, time in seconds, velocity in m/s, accelerations in
  m/s², angles in radians, forces in newtons, torques in N·m. Steer angle limit
  is stored in radians after `deg2rad` conversion.
* Velocity guard $\varepsilon_v = 10^{-3}\ \mathrm{m/s}$ in slip calculations
  prevents division by zero at standstill.
* Force guard $\varepsilon = 10^{-9}$ in the tyre saturation argument avoids
  $0/0$ when $F_z\to 0$.
* Speed floor $10^{-3}\ \mathrm{m/s}$ in all $\Delta s / v_{\text{avg}}$
  integrations prevents lap time from diverging at the first node.
* All squared‑speed updates use $\max(\cdot,0)$ before $\sqrt{\cdot}$ to keep
  the solver robust against tiny negative round‑off when $v_i\to 0$.
* Angle unwrapping is applied to $\psi$ from `atan2` so curvature
  $\kappa = \mathrm{d}\psi/\mathrm{d}s$ is well‑defined across $\pm\pi$ crossings.


## 14. Symbols quick reference

| Symbol | Meaning | Units |
|---|---|---|
| $s$ | arc length | m |
| $\kappa$ | path curvature | 1/m |
| $\psi$ | heading angle | rad |
| $v$ | speed along path | m/s |
| $u,v$ | body‑frame long/lat velocity | m/s |
| $r$ | yaw rate | rad/s |
| $\omega_f,\omega_r$ | wheel spin | rad/s |
| $\delta$ | steer angle | rad |
| $\alpha$ | slip angle | rad |
| $\kappa_{\text{tyre}}$ | longitudinal slip ratio | – |
| $F_z, F_x, F_y$ | tyre normal / long / lat force | N |
| $T_f, T_r$ | wheel torques | N·m |
| $a, b$ | CG to front / rear axle | m |
| $L$ | wheelbase | m |
| $m, I_z$ | mass / yaw inertia | kg, kg·m² |
| $R_w, J_w$ | wheel radius / spin inertia | m, kg·m² |
| $\mu_x, \mu_y$ | peak friction long / lat | – |
| $C_x, C_\alpha$ | linear long / lat tyre stiffness | N, N/rad |
| $\rho, C_dA, C_lA$ | air density, drag / lift area | kg/m³, m² |
| $g$ | gravity | m/s² |

---

*Document scope: theory and mathematics of the current `LTS-MATLAB` build,
written to sit alongside the source files in the repository. The mathematics
in §6 is intentionally honest about the shortcut: it describes what the code
**actually computes**, not what a full QSS‑with‑trim GGV would produce.*
