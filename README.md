# GGV Lap Simulation

**COMPLETE FORMULATION — TIRE · VEHICLE · OPTIMISER · CIRCUIT · LAP TIME**

## Tire Models

**STEP 1 — SLIP STATE + VERTICAL LOAD → Fx, Fy, Mz**

The tire is the only element that generates forces at the road contact patch. All three models below take the same inputs and return forces in the tire/wheel frame. The vehicle model then rotates these into the body frame.

| Input | Symbol | Description |
|---|---|---|
| Slip angle | α | Angle between wheel heading and velocity vector at contact patch [rad] |
| Slip ratio | κ | (ω·R_e − v_x) / v_x — normalised difference between wheel speed and hub speed |
| Vertical load | F_z | Normal force at contact patch [N], computed from weight + load transfer + aero |

### Linear Tire Model

Valid only near zero slip. Cornering stiffness C_α and longitudinal stiffness C_κ are constants independent of load. Simplest possible model — useful for analytical solutions and initial bicycle model work.

**LATERAL FORCE**

$$
F_y = -C_\alpha \cdot \alpha
$$

**LONGITUDINAL FORCE**

$$
F_x = -C_\kappa \cdot \kappa
$$

**SELF-ALIGNING MOMENT — LINEAR APPROXIMATION**

$$
M_z = t_p \cdot C_\alpha \cdot \alpha
$$

where t_p is pneumatic trail [m], typically 0.01–0.05 m. Often neglected in linear models.

| Parameter | Unit | Typical value |
|---|---|---|
| C_α | N/rad | 50,000–100,000 per tire |
| C_κ | N/– | 80,000–150,000 per tire |
| t_p | m | 0.01–0.05 |

### Nonlinear Saturating Tire Model

Retains linear stiffness near zero slip but saturates at the friction limit μ·F_z. The tanh function provides a smooth transition. Load dependence is included by scaling stiffness with F_z.

**LATERAL FORCE**

$$
F_y = -\mu_y F_z \tanh\!\left(\frac{C_{\alpha 0}}{\mu_y F_{z0}} \cdot \frac{F_z}{F_{z0}} \cdot \alpha\right)
$$

**LONGITUDINAL FORCE**

$$
F_x = -\mu_x F_z \tanh\!\left(\frac{C_{\kappa 0}}{\mu_x F_{z0}} \cdot \frac{F_z}{F_{z0}} \cdot \kappa\right)
$$

**COMBINED SLIP — FRICTION ELLIPSE SCALING**

$$
F_x^{comb} = F_x \cdot \frac{1}{\sqrt{1 + (F_y / \mu_y F_z)^2}}, \qquad F_y^{comb} = F_y \cdot \frac{1}{\sqrt{1 + (F_x / \mu_x F_z)^2}}
$$

| Parameter | Unit | Description |
|---|---|---|
| μ_x, μ_y | — | Peak friction coefficients longitudinal and lateral |
| C_α0, C_κ0 | N/rad, N/— | Stiffness at reference load F_z0 |
| F_z0 | N | Reference normal load (nominal static load per tire) |

### Pacejka Magic Formula

Empirical curve fit covering the full slip range including peak and falloff. Parameters B, C, D, E are fitted to measured tire data. D is the peak force and is load-dependent.

**CORE MAGIC FORMULA — BOTH AXES USE THIS FORM**

$$
F = D \sin\!\left[C \arctan\!\left(Bx - E\left(Bx - \arctan Bx\right)\right)\right]
$$

$$
\text{where } x = \alpha \text{ (lateral) or } x = \kappa \text{ (longitudinal)}
$$

**LATERAL FORCE — PURE SLIP**

$$
F_y(\alpha, F_z) = D_y \sin\!\left[C_y \arctan\!\left(B_y\alpha - E_y(B_y\alpha - \arctan B_y\alpha)\right)\right]
$$

$$
D_y = \mu_y F_z, \qquad B_y = \frac{C_{\alpha}}{C_y D_y}
$$

**LONGITUDINAL FORCE — PURE SLIP**

$$
F_x(\kappa, F_z) = D_x \sin\!\left[C_x \arctan\!\left(B_x\kappa - E_x(B_x\kappa - \arctan B_x\kappa)\right)\right]
$$

$$
D_x = \mu_x F_z, \qquad B_x = \frac{C_{\kappa}}{C_x D_x}
$$

**SELF-ALIGNING MOMENT — OWN MAGIC FORMULA FIT**

$$
M_z(\alpha, F_z) = D_{Mz} \sin\!\left[C_{Mz} \arctan\!\left(B_{Mz}\alpha - E_{Mz}(B_{Mz}\alpha - \arctan B_{Mz}\alpha)\right)\right]
$$

$$
D_{Mz} = -t_0 \cdot C_\alpha \qquad \text{(t}_0\text{ = pneumatic trail at zero slip)}
$$

**COMBINED SLIP — FRICTION ELLIPSE ON PACEJKA**

$$
\sigma = \sqrt{\left(\frac{\kappa}{1+\kappa}\right)^2 + \left(\frac{\tan\alpha}{1+\kappa}\right)^2}
$$

$$
F_x^{comb} = \frac{\kappa/(1+\kappa)}{\sigma} F_x^{pure}(\sigma), \qquad F_y^{comb} = \frac{\tan\alpha/(1+\kappa)}{\sigma} F_y^{pure}(\sigma)
$$

| Parameter | Role |
|---|---|
| B | Stiffness factor — controls initial slope |
| C | Shape factor — controls peak width |
| D | Peak value = μ·F_z |
| E | Curvature factor — controls falloff after peak |

### Tire Model Compatibility

| Tire model | Point mass | Bicycle | Four wheel | Solver needed |
|---|---|---|---|---|
| Linear | ✗ | ✓ | ✓ | Analytic / fixed point |
| Nonlinear | circle | ✓ | ✓ | fsolve |
| Pacejka | circle | ✓ | ✓ | fsolve / NLP |

## Vehicle Models

**STEP 2 — STATES + DRIVER INPUTS → ACCELERATIONS**

### Point Mass

No geometry, no rotation, no tire slip angles. The vehicle is a particle of mass m subject to a net friction force. The friction limit defines the GG envelope directly.

**EQUATIONS OF MOTION**

$$
m\,a_x = F_x, \qquad m\,a_y = F_y
$$

$$
\text{No yaw equation — no rotational DOF}
$$

**FRICTION CIRCLE CONSTRAINT**

$$
a_x^2 + a_y^2 \leq (\mu g)^2
$$

$$
\text{With aero: } \quad a_x^2 + a_y^2 \leq \left(\mu\,g + \frac{\mu\,\rho\,C_L\,A_{ref}\,v^2}{2m}\right)^2
$$

States: speed v. Driver inputs: direction of net force vector. No steer angle, no sideslip, no yaw rate.

### Quarter Car

One corner of the vehicle. Two masses, two springs, vertical motion only. Cannot generate lateral or longitudinal forces — it lives upstream of the GGV pipeline, not inside it. Use it to compute how F_z varies dynamically under road inputs, then feed that varying F_z into the tire model.

**SPRUNG MASS**

$$
m_s\,\ddot{z}_s = -k_s(z_s - z_u) - c_s(\dot{z}_s - \dot{z}_u)
$$

**UNSPRUNG MASS**

$$
m_u\,\ddot{z}_u = k_s(z_s - z_u) + c_s(\dot{z}_s - \dot{z}_u) - k_t(z_u - z_r)
$$

**DYNAMIC NORMAL LOAD (feeds into tire model)**

$$
F_z(t) = k_t\,(z_u(t) - z_r(t))
$$

> Quarter car position in pipeline: road profile → quarter car ODE → F_z(t) → tire model → GGV. It is an upstream pre-processor, not part of the trim optimiser loop.

### Bicycle Model (Half Car)

Two wheels on the centreline. Front wheel steerable. Captures yaw dynamics, sideslip, understeer and oversteer. No track width — left and right wheels on each axle are collapsed to one. This is the standard model for GGV generation with a trim optimiser.

#### Slip Angles

**FRONT AND REAR SLIP ANGLES — EXACT FORM**

$$
\alpha_f = \delta - \arctan\!\left(\frac{v + a\,r}{u}\right)
$$

$$
\alpha_r = -\arctan\!\left(\frac{v - b\,r}{u}\right)
$$

#### Slip Ratios

**LONGITUDINAL SLIP**

$$
\kappa_f = \frac{\omega_f R_e - u}{\max(|u|, \varepsilon)}, \qquad \kappa_r = \frac{\omega_r R_e - u}{\max(|u|, \varepsilon)}
$$

#### Force Rotation into Body Frame

**FRONT AXLE — ROTATE BY STEER ANGLE δ**

$$
F_{x,f}^{body} = F_{xf}\cos\delta - F_{yf}\sin\delta
$$

$$
F_{y,f}^{body} = F_{yf}\cos\delta + F_{xf}\sin\delta
$$

#### Equations of Motion — Bicycle

**LONGITUDINAL**

$$
m(\dot{u} - v\,r) = F_{xf}\cos\delta - F_{yf}\sin\delta + F_{xr}
$$

**LATERAL**

$$
m(\dot{v} + u\,r) = F_{yf}\cos\delta + F_{xf}\sin\delta + F_{yr}
$$

**YAW**

$$
I_z\,\dot{r} = a\!\left(F_{yf}\cos\delta + F_{xf}\sin\delta\right) - b\,F_{yr}
$$

> The bicycle model yaw equation has no y_i·F_xi terms because all wheels are on the centreline (y_i = 0). Self-aligning moment M_z from the tire can be added: I_z·ṙ = a(F_yf cosδ + F_xf sinδ) − b·F_yr + M_zf + M_zr

### Four Wheel Model

Four wheels at track width offsets ±t_f/2 (front) and ±t_r/2 (rear). Left and right loads differ due to lateral load transfer. Left and right longitudinal forces can differ (torque vectoring, differential braking). Yaw moment includes the y_i·F_xi term from each wheel's lateral position.

#### Individual Wheel Yaw Contribution (from Jazar eq 10.86)

**YAW MOMENT FROM WHEEL i — FULL FORM**

$$
M_{z,i} = M_{z_{w,i}} + x_i F_{y,i} - y_i F_{x,i}
$$

$$
\text{where } x_i = \pm a \text{ or } \pm b, \quad y_i = \pm t_f/2 \text{ or } \pm t_r/2
$$

#### Four Wheel Equations of Motion

**LONGITUDINAL**

$$
m\,a_x = (F_{xfl} + F_{xfr})\cos\delta - (F_{yfl} + F_{yfr})\sin\delta + F_{xrl} + F_{xrr}
$$

**LATERAL**

$$
m\,a_y = (F_{yfl} + F_{yfr})\cos\delta + (F_{xfl} + F_{xfr})\sin\delta + F_{yrl} + F_{yrr}
$$

**YAW — FULL FOUR WHEEL WITH TRACK WIDTH**

$$
I_z\,\dot{r} = a\!\left[(F_{yfl}+F_{yfr})\cos\delta + (F_{xfl}+F_{xfr})\sin\delta\right] - b(F_{yrl}+F_{yrr})
$$

$$
+ \frac{t_f}{2}\!\left[(F_{xfl}-F_{xfr})\cos\delta - (F_{yfl}-F_{yfr})\sin\delta\right] + \frac{t_r}{2}(F_{xrl}-F_{xrr})
$$

$$
+ M_{zfl} + M_{zfr} + M_{zrl} + M_{zrr}
$$

## Load Transfer

**COUPLING: ACCELERATIONS → F_z → TIRE FORCES → ACCELERATIONS**

Load transfer couples the vehicle accelerations back into the tire normal loads, which changes the tire force limits. This is an implicit loop — the system must be solved simultaneously.

**STATIC AXLE LOADS**

$$
F_{z,f}^{static} = \frac{mgb}{L}, \qquad F_{z,r}^{static} = \frac{mga}{L}
$$

**LONGITUDINAL LOAD TRANSFER — REARWARD UNDER ACCELERATION**

$$
\Delta F_z^{long} = \frac{m\,a_x\,h_{cg}}{L}
$$

$$
F_{z,f} \rightarrow F_{z,f}^{static} - \Delta F_z^{long}, \qquad F_{z,r} \rightarrow F_{z,r}^{static} + \Delta F_z^{long}
$$

**LATERAL LOAD TRANSFER — FOUR WHEEL REQUIRED**

$$
\Delta F_{z,f}^{lat} = \frac{m\,a_y\,h_{f}}{2t_f}, \qquad \Delta F_{z,r}^{lat} = \frac{m\,a_y\,h_{r}}{2t_r}
$$

**PER-WHEEL NORMAL LOADS**

$$
F_{z,fl} = \frac{F_{z,f} - \Delta F_z^{long}}{2} - \Delta F_{z,f}^{lat}
$$

$$
F_{z,fr} = \frac{F_{z,f} - \Delta F_z^{long}}{2} + \Delta F_{z,f}^{lat}
$$

$$
F_{z,rl} = \frac{F_{z,r} + \Delta F_z^{long}}{2} - \Delta F_{z,r}^{lat}
$$

$$
F_{z,rr} = \frac{F_{z,r} + \Delta F_z^{long}}{2} + \Delta F_{z,r}^{lat}
$$

**AERODYNAMIC DOWNFORCE — SPEED DEPENDENT**

$$
F_{aero} = \frac{1}{2}\rho\,C_L\,A_{ref}\,v^2
$$

$$
F_{z,f} \mathrel{+}= \xi_f \cdot F_{aero}, \qquad F_{z,r} \mathrel{+}= \xi_r \cdot F_{aero}
$$

$$
\xi_f + \xi_r = 1 \quad \text{(aero balance)}
$$

**NON-NEGATIVITY CONSTRAINT — WHEEL LIFT**

$$
F_{z,ij} \geq 0 \quad \forall\, i \in \{f,r\},\; j \in \{l,r\}
$$

## Optimiser Methods

**STEP 3 — WRAP AROUND VEHICLE MODEL → TRIM → MAX ACCELERATION**

For quasi-static GGV generation, steady state is imposed: ṙ = 0 and β̇ = 0. The car is in trimmed equilibrium — no yaw acceleration, constant sideslip. The acceleration magnitude A in direction φ is then maximised. The direction angle φ is polar coordinates in the GG plane.

**POLAR PARAMETERISATION OF THE GG PLANE**

$$
a_x = A\cos\varphi, \qquad a_y = A\sin\varphi, \qquad \varphi \in [0,2\pi)
$$

$$
r = \frac{a_y}{v} = \frac{A\sin\varphi}{v} \quad \text{(kinematic, trim condition)}
$$

### Option 1 — Friction Circle (Point Mass Only)

No vehicle model, no solver. The GG envelope is computed analytically from the friction limit and aero downforce.

**GG ENVELOPE — DIRECT FORMULA**

$$
A_{max}(\varphi, v) = \mu\,g + \frac{\mu\,\rho\,C_L\,A_{ref}\,v^2}{2m}
$$

$$
\text{The envelope is a speed-dependent circle — same in all directions } \varphi
$$

**ASYMMETRIC ELLIPSE — SEPARATE BRAKING AND DRIVE LIMITS**

$$
\left(\frac{a_x}{a_{x,lim}(\varphi)}\right)^2 + \left(\frac{a_y}{\mu_y g_{eff}}\right)^2 \leq 1
$$

$$
a_{x,lim} = \begin{cases} \mu_x g_{eff} \cdot \eta_{drive} & a_x > 0 \\ \mu_x g_{eff} & a_x < 0 \end{cases}
$$

$$
\text{where } \eta_{drive} < 1 \text{ for RWD/FWD, } \eta_{drive} = 1 \text{ for AWD}
$$

### Option 2 — Fixed Point Iteration (Bicycle + Linear Tire)

No external solver. Exploit the fact that with a linear tire, yaw balance gives a closed-form relationship between δ and β. Iterate to convergence.

**FIXED POINT ALGORITHM**

1. Given (v, φ, A): set target a_x = A cosφ, a_y = A sinφ, r = a_y/v
2. Initial guess: β = 0, δ = a_y·L/v² (Ackermann)
3. Compute α_f = δ − β − a·r/v, α_r = −β + b·r/v
4. Compute F_yf = −C_αf·α_f, F_yr = −C_αr·α_r
5. Yaw residual: ΔM = a·F_yf − b·F_yr. Correct δ ← δ − ΔM/(a·C_αf)
6. Lateral residual: Δa_y = (F_yf + F_yr)/m − a_y. Correct β ← β − Δa_y·m/(C_αf + C_αr)
7. Repeat from step 3 until |ΔM| < ε and |Δa_y| < ε
8. Check tire limits. If violated, reduce A and repeat.

### Option 3 — Root Finding with fsolve (Bicycle + Nonlinear/Pacejka)

Write the three equilibrium equations as residuals. At fixed (v, φ, A), solve for the free variables [δ, β, κ_r]. Bisect on A until a tire constraint becomes active.

**RESIDUAL VECTOR — THREE EQUATIONS, THREE UNKNOWNS [δ, β, κ_r]**

$$
g_1 = m A\cos\varphi - \left[F_{xf}\cos\delta - F_{yf}\sin\delta + F_{xr}\right] = 0
$$

$$
g_2 = m A\sin\varphi - \left[F_{yf}\cos\delta + F_{xf}\sin\delta + F_{yr}\right] = 0
$$

$$
g_3 = a(F_{yf}\cos\delta + F_{xf}\sin\delta) - b\,F_{yr} = 0
$$

**FSOLVE + BISECTION ALGORITHM**

1. Given (v, φ): set A_lo = 0, A_hi = μg_eff (initial upper bound)
2. Bisect: A_mid = (A_lo + A_hi)/2
3. Compute r = A_mid sinφ / v. Update F_z from load transfer using (A_mid cosφ, A_mid sinφ)
4. Call fsolve on [g1, g2, g3] with unknowns [δ, β, κ_r], warm-started from previous solution
5. Evaluate friction ellipse for each tire: h_i = (F_xi/μx F_zi)² + (F_yi/μy F_zi)²
6. If max(h_i) < 1: A_lo ← A_mid (feasible, can go higher). If max(h_i) > 1: A_hi ← A_mid (infeasible)
7. Repeat from step 2 until A_hi − A_lo < tolerance
8. Store A* = A_lo as the GGV point at (v, φ)

### Option 4 — Constrained NLP (Four Wheel + Pacejka)

Maximise A directly as the objective. All driver inputs and vehicle states are free variables simultaneously. Equality constraints enforce dynamics. Inequality constraints enforce tire limits. Suitable for fmincon (MATLAB) or IPOPT.

**OBJECTIVE**

$$
\max_{\mathbf{x}} \quad A
$$

$$
\mathbf{x} = [\delta,\; \kappa_{fl},\; \kappa_{fr},\; \kappa_{rl},\; \kappa_{rr},\; \beta]
$$

**EQUALITY CONSTRAINTS — DYNAMICS (g = 0)**

$$
g_1: \quad m A\cos\varphi = F_x^{total}(\mathbf{x}, v, A)
$$

$$
g_2: \quad m A\sin\varphi = F_y^{total}(\mathbf{x}, v, A)
$$

$$
g_3: \quad M_z^{total}(\mathbf{x}, v, A) = 0
$$

**INEQUALITY CONSTRAINTS — TIRE FRICTION ELLIPSE (h ≤ 0)**

$$
h_i: \quad \left(\frac{F_{x,i}}{\mu_x F_{z,i}}\right)^2 + \left(\frac{F_{y,i}}{\mu_y F_{z,i}}\right)^2 - 1 \leq 0 \quad \forall\; i \in \{fl,fr,rl,rr\}
$$

**BOUND CONSTRAINTS**

$$
\delta_{min} \leq \delta \leq \delta_{max}, \quad -1 \leq \kappa_{ij} \leq \kappa_{drive,max}, \quad F_{z,i} \geq 0
$$

**KKT CONDITIONS AT OPTIMUM — ACTIVE TIRE LIMITS**

$$
\nabla_{\mathbf{x}} A = \sum_j \lambda_j \nabla_{\mathbf{x}} g_j + \sum_i \mu_i \nabla_{\mathbf{x}} h_i
$$

$$
\mu_i \geq 0, \quad \mu_i\,h_i = 0 \quad \text{(tire not at limit → } \mu_i = 0\text{)}
$$

### Option 5 — Milliken Moment Method (MMM)

Grid evaluation, no optimiser. Sweep [δ, β] over a grid. At each grid point evaluate yaw moment N and lateral force Y. The trim line is where N = 0. Read the maximum Y off this line — that is the peak lateral acceleration in the trimmed state.

**MMM ALGORITHM**

1. Fix speed v. Define grid: β ∈ [−β_max, β_max], δ ∈ [−δ_max, δ_max]
2. For each (β_i, δ_j): compute α_f, α_r, then tire forces, then N = I_z·ṙ and Y = m·a_y
3. Interpolate the N = 0 contour across the (β, δ) grid — this is the trim line
4. Along the trim line, find max(Y) — this is a_y,max at speed v
5. For braking/acceleration: add F_x as a parameter and repeat

### Optimiser Compatibility Matrix

| Method | Point mass | Bicycle | Four wheel | Linear | Nonlinear | Pacejka |
|---|---|---|---|---|---|---|
| Friction circle | ✓ only | ✗ | ✗ | ✗ | circle | circle |
| Fixed point | ✗ | ✓ | ✗ | ✓ | marginal | ✗ |
| fsolve | ✗ | ✓ | limited | ✓ | ✓ | ✓ |
| NLP (fmincon) | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |
| MMM | ✗ | ✓ | ✓ | ✓ | ✓ | ✓ |

## GGV Surface Generation

**STEP 4 — OPTIMISER AT EVERY (v, φ) → FULL SURFACE**

The GGV surface is built independently of the track. It is a lookup table of maximum achievable acceleration as a function of speed and direction. Once built it is interpolated during the lap simulation.

**GGV GENERATION PROCEDURE**

1. Define speed grid: v ∈ {v_1, v_2, ..., v_N} — start from low speed (e.g. 10 m/s), step up through expected operating range
2. Define direction grid: φ_j = j · 2π/M for j = 0, 1, ..., M−1. Typically M = 36 to 72 points (5° to 10° resolution)
3. For each v_i: update static loads, aero downforce F_aero(v_i), load distribution
4. For each φ_j: run the chosen optimiser → returns A*_ij and the trim solution [δ*, β*, κ*]
5. Store GG point: a_x = A*_ij cosφ_j, a_y = A*_ij sinφ_j at speed v_i
6. After all φ sweeps at v_i: the set of (a_x, a_y) points is the GG envelope at v_i
7. Repeat for all v_i: stack all envelopes → GGV surface as 3D array GGV(v, a_y) → a_x_max
8. Build interpolant: given any (v, a_y), return maximum a_x and minimum a_x (braking). Use 2D interpolation (e.g. griddedInterpolant in MATLAB)

> Braking and acceleration are both computed by the same optimiser sweep — φ = π gives pure braking (a_x < 0), φ = 0 gives pure acceleration (a_x > 0). No assumptions are made about either direction.

## Circuit Model

**STEP 5 — CURVATURE κ(s) AGAINST DISTANCE s**

The circuit is represented as a 1D vector of curvature values against arc length along the racing line. The lap sim does not need X,Y coordinates — only κ(s).

### Constant Radius Circle

**CIRCLE**

$$
\kappa(s) = \frac{1}{R} = \text{const}, \qquad s \in [0, 2\pi R]
$$

### Piecewise Analytic (Straights and Corners)

**PIECEWISE CURVATURE**

$$
\kappa(s) = \begin{cases} 0 & \text{straight segment} \\ 1/R_k & \text{corner } k \text{ with radius } R_k \end{cases}
$$

### Real Circuit from (X, Y) Coordinates

Given GPS or geometric data as discrete points (X_i, Y_i), compute arc length and curvature numerically.

**ARC LENGTH PARAMETERISATION**

$$
s_i = \sum_{j=1}^{i} \sqrt{(X_j - X_{j-1})^2 + (Y_j - Y_{j-1})^2}
$$

**CURVATURE FROM PARAMETRIC CURVE**

$$
\kappa(s) = \frac{X'Y'' - Y'X''}{(X'^2 + Y'^2)^{3/2}}
$$

Primes are derivatives with respect to s. Compute numerically using central differences on the discrete (X_i, Y_i) data. Smooth κ(s) before use to remove GPS noise.

**SIGN CONVENTION**

$$
\kappa > 0: \text{ left turn}, \quad \kappa < 0: \text{ right turn}, \quad \kappa = 0: \text{ straight}
$$

$$
|\kappa| \text{ used for speed limit in Pass 1 (lateral acceleration is always centripetal)}
$$

## Pass 1 — Maximum Cornering Speed

**LATERAL LIMIT ONLY — NO LONGITUDINAL COUPLING**

At every point s on the circuit, find the maximum speed at which the vehicle can follow the curvature κ(s), using only the lateral axis of the GGV. Longitudinal acceleration is ignored entirely — this pass does not know what came before or after on the track.

**CENTRIPETAL ACCELERATION REQUIRED TO FOLLOW κ(s) AT SPEED v**

$$
a_y^{required}(s, v) = v^2 \cdot |\kappa(s)| = \frac{v^2}{R(s)}
$$

**MAXIMUM LATERAL ACCELERATION AVAILABLE FROM GGV AT SPEED v**

$$
a_{y,max}(v) = \text{GGV}(v, \varphi = 90°) \quad \text{(pure lateral, a_x = 0)}
$$

**PASS 1 — MAXIMUM SPEED AT EACH POINT s**

$$
v_{max,1}(s) : \quad v^2 |\kappa(s)| = a_{y,max}(v)
$$

$$
\text{Solved iteratively since } a_{y,max} \text{ depends on } v
$$

$$
\textbf{Iteration: } v^{(k+1)} = \sqrt{\frac{a_{y,max}(v^{(k)})}{|\kappa(s)|}} \quad \text{until convergence}
$$

$$
\text{On a straight: } |\kappa| = 0 \Rightarrow v_{max,1} = \infty \text{ — no lateral constraint there}
$$

> Pass 1 gives a speed ceiling at every point independently. There is no continuity between adjacent points — the profile can jump from very high speed on a straight to the corner limit with no transition. Passes 2 and 3 enforce that continuity.

## Passes 2 & 3 — Longitudinal Continuity

**FORWARD ACCELERATION SWEEP + BACKWARD BRAKING SWEEP**

Pass 1 gives no continuity between points — a car cannot instantly reach corner speed on a straight or instantly slow for a hairpin. Passes 2 and 3 enforce physical continuity by asking how fast the car can accelerate and brake between successive points, using the longitudinal axis of the GGV at the current lateral load.

### GGV Lookup During Passes

**LONGITUDINAL LIMIT FROM GGV — GIVEN CURRENT (v, a_y)**

$$
a_x^{accel}(v, a_y) = \text{GGV\_interp}(v, a_y, \text{drive side})
$$

$$
a_x^{brake}(v, a_y) = \text{GGV\_interp}(v, a_y, \text{brake side}) \quad \text{(negative value)}
$$

$$
\text{where } a_y = v^2 |\kappa(s)| \text{ at the current point}
$$

### Anchor Point

**STARTING POINT FOR BOTH SWEEPS**

$$
s^* = \arg\min_s\; v_{max,1}(s) \quad \text{— the tightest corner (lowest Pass 1 speed)}
$$

$$
v_{start} = v_{max,1}(s^*) \quad \text{— both passes begin here}
$$

### Pass 2 — Forward Sweep (Acceleration Out of Corners)

**PASS 2 — FORWARD MARCH FROM s* AROUND THE FULL LAP**

$$
v_{fwd}(s + \Delta s) = \min\!\left(v_{max,1}(s + \Delta s),\; \sqrt{v_{fwd}(s)^2 + 2\,a_x^{accel}(v_{fwd}(s),\, v_{fwd}(s)^2|\kappa(s)|)\,\Delta s}\right)
$$

### Pass 3 — Backward Sweep (Braking Into Corners)

**PASS 3 — BACKWARD MARCH FROM s* AROUND THE FULL LAP**

$$
v_{bwd}(s - \Delta s) = \min\!\left(v_{max,1}(s - \Delta s),\; \sqrt{v_{bwd}(s)^2 + 2\,|a_x^{brake}(v_{bwd}(s),\, v_{bwd}(s)^2|\kappa(s)|)|\,\Delta s}\right)
$$

### Final Speed Profile

**SPEED PROFILE — ELEMENT-WISE MINIMUM**

$$
v(s) = \min\!\left(v_{fwd}(s),\; v_{bwd}(s)\right)
$$

> **WHY MINIMUM**  
> At corner exit: forward sweep is binding — acceleration limited. At corner entry: backward sweep is binding — braking limited. At the apex: Pass 1 cornering limit is binding. The minimum selects the physically correct binding constraint at every point automatically. The car cannot exceed whichever constraint is tightest at each location.

## Lap Time

**LAP TIME INTEGRAL**

$$
t_{lap} = \int_0^{S_{lap}} \frac{ds}{v(s)} \approx \sum_{k=1}^{N} \frac{\Delta s}{v(s_k)}
$$

## Numerical Algorithms — Ready for MATLAB

**COMPLETE WORKFLOW FOR EACH VEHICLE + TIRE + OPTIMISER COMBINATION**

### Algorithm 1 — Point Mass + Friction Circle + Any Circuit

Simplest pipeline. No vehicle states. No optimiser. GGV is computed analytically.

**PHASE 1 — BUILD GGV (ANALYTIC)**

1. Define: m, μ_x, μ_y, C_L, A_ref, ρ, η_drive (drivetrain efficiency: 1.0 AWD, 0.5–0.7 RWD/FWD)
2. Define speed grid: v = linspace(v_min, v_max, N_v)
3. For each v_i: g_eff = g + ρ C_L A_ref v_i²/(2m) — effective gravity including aero
4. a_y_max(v_i) = μ_y · g_eff
5. a_x_accel_max(v_i) = μ_x · g_eff · η_drive
6. a_x_brake_max(v_i) = −μ_x · g_eff (all wheels braking)
7. Store GGV as: for each (v_i, a_y): a_x_max = a_x_accel · sqrt(1 − (a_y/a_y_max)²), a_x_min = a_x_brake · sqrt(1 − (a_y/a_y_max)²)

**PHASE 2 — CIRCUIT MODEL**

1. Circle: kappa = 1/R · ones(1, N_s), s = linspace(0, 2πR, N_s)
2. Real circuit: load (X,Y) coordinates. Compute s by cumulative arc length. Compute kappa numerically using central differences on X,Y. Smooth kappa with a moving average or spline.

**PHASE 3 — THREE PASSES**

1. Pass 1: for each s_k: solve v²|κ(s_k)| = a_y_max(v) iteratively for v. Store v_max1(k).
2. Find anchor: [~, k_star] = min(v_max1). Set v_fwd(k_star) = v_max1(k_star), v_bwd(k_star) = v_max1(k_star).
3. Pass 2 forward: for k = k_star+1 to k_star+N_s (wrapping modulo N_s): a_y_cur = v_fwd(k-1)²·|κ(k-1)|. Lookup a_x_accel from GGV at (v_fwd(k-1), a_y_cur). v_candidate = sqrt(v_fwd(k-1)² + 2·a_x_accel·Δs). v_fwd(k) = min(v_max1(k), v_candidate).
4. Pass 3 backward: for k = k_star-1 down to k_star-N_s (wrapping): a_y_cur = v_bwd(k+1)²·|κ(k+1)|. Lookup |a_x_brake| from GGV. v_candidate = sqrt(v_bwd(k+1)² + 2·|a_x_brake|·Δs). v_bwd(k) = min(v_max1(k), v_candidate).
5. Final profile: v_final(k) = min(v_fwd(k), v_bwd(k)) for all k.
6. Lap time: t_lap = sum(Δs ./ v_final)

### Algorithm 2 — Bicycle + Nonlinear Tire + fsolve + Real Circuit

**PHASE 1 — BUILD GGV WITH FSOLVE + BISECTION**

1. Define parameters: m, I_z, a, b, L, h_cg, μ_x, μ_y, C_α0, C_κ0, F_z0, R_e, C_L, A_ref, ρ, ξ_f, ξ_r
2. Define grids: v = linspace(v_min, v_max, N_v), phi = linspace(0, 2π−dφ, N_phi)
3. For each v_i: compute F_aero = 0.5·ρ·C_L·A_ref·v_i². Compute F_z0_f = m·g·b/L + ξ_f·F_aero, F_z0_r = m·g·a/L + ξ_r·F_aero (before load transfer)
4. For each phi_j: set target direction. Compute r_target = A·sin(phi_j)/v_i (function of A)
5. Bisect on A: A_lo = 0, A_hi = 2·μ_y·g_eff
6. At each bisection step A_mid: update F_z with longitudinal load transfer ΔFz = m·A_mid·cos(phi_j)·h_cg/L. Call fsolve([g1;g2;g3], x0=[delta_guess; beta_guess; kappa_r_guess])
7. Residuals g1, g2, g3: compute alpha_f, alpha_r from [delta, beta, v, r]. Compute F_yf, F_yr, F_xr from nonlinear tire model. Evaluate long/lat/yaw balance equations.
8. Evaluate friction ellipse: h_f = (F_xf/μx Fzf)² + (F_yf/μy Fzf)², h_r = (F_xr/μx Fzr)² + (F_yr/μy Fzr)². If max(h_f, h_r) < 1: A_lo ← A_mid else A_hi ← A_mid
9. After convergence: store GGV(i,j) = A_lo. Store a_x = A_lo·cos(phi_j), a_y = A_lo·sin(phi_j).
10. Build 2D interpolant: GGV_ax_max = griddedInterpolant({v_grid, ay_grid}, ax_max_table). Similarly for ax_min (braking side of phi sweep).

**PHASE 2 — CIRCUIT + THREE PASSES (same as Algorithm 1 Phase 2 & 3)**

1. Build κ(s) from circuit data as described above
2. Pass 1: for each s_k, solve v²|κ_k| = GGV_ay_max(v) iteratively
3. Passes 2 & 3: identical to Algorithm 1 but GGV lookup uses griddedInterpolant instead of analytic formula
4. Lap time: t_lap = sum(Δs ./ v_final)

### Algorithm 3 — Bicycle + Pacejka + NLP (fmincon)

**PHASE 1 — GGV WITH FMINCON**

1. Define Pacejka parameters for front and rear: Bx, Cx, Dx=μ_x·Fz, Ex, By, Cy, Dy=μ_y·Fz, Ey, and Mz parameters B_Mz, C_Mz, D_Mz for each axle
2. For each (v_i, phi_j): formulate fmincon problem. Objective: −A (minimise negative = maximise). Variables: x = [A; delta; beta; kappa_r] (4 variables, front undriven so kappa_f = 0)
3. Nonlinear equality constraints ceq = [g1; g2; g3] — evaluate Pacejka forces inside constraint function at current x, v_i, phi_j
4. Nonlinear inequality constraints c = [h_f − 1; h_r − 1] — friction ellipse per axle
5. Bounds: lb = [0; delta_min; beta_min; −1], ub = [2·g_eff; delta_max; beta_max; kappa_max]
6. Call fmincon with options: Algorithm = 'interior-point', SpecifyObjectiveGradient = false, SpecifyConstraintGradient = false. Warm-start x0 from previous (phi_j−1) solution.
7. Store A* = x(1). Recover a_x = A*·cos(phi_j), a_y = A*·sin(phi_j).
8. Build GGV interpolant as in Algorithm 2

### Algorithm 4 — Four Wheel + Pacejka + NLP

**PHASE 1 — GGV WITH FOUR WHEEL NLP**

1. Define: m, I_z, a, b, t_f, t_r, h_cg, h_f, h_r, C_L, A_ref, ρ, ξ_f, ξ_r. Pacejka parameters per wheel (or per axle if symmetric).
2. Variables: x = [A; delta; kappa_fl; kappa_fr; kappa_rl; kappa_rr; beta] — 7 variables
3. Inside constraint and objective evaluation: compute r = A·sin(phi)/v. Compute alpha_fl, alpha_fr from (delta, beta, v, r, t_f). Compute alpha_rl, alpha_rr from (beta, v, r, t_r).
4. Compute F_z for each wheel using per-wheel load transfer equations with current (A·cos(phi), A·sin(phi)) as a_x, a_y. Note: this creates implicit coupling — F_z depends on A which is being optimised. Accept this and evaluate sequentially inside each function call.
5. Call Pacejka for each wheel: [Fx_ij, Fy_ij, Mz_ij] = tire_magic(Fz_ij, alpha_ij, kappa_ij, params)
6. Rotate front forces: Fx_fl_body = Fx_fl·cosδ − Fy_fl·sinδ, etc.
7. Equality constraints ceq: [m·A·cos(phi) − Fx_total; m·A·sin(phi) − Fy_total; Mz_total] = 0 where Mz_total uses full four-wheel yaw equation including t_f/2 and t_r/2 terms and self-aligning moments
8. Inequality constraints: friction ellipse for all four wheels independently: (Fx_ij/μx Fz_ij)² + (Fy_ij/μy Fz_ij)² ≤ 1, and Fz_ij ≥ 0
9. Run fmincon. Warm-start from previous phi solution. Repeat for all (v_i, phi_j).
10. Build GGV interpolant. Run circuit three-pass lap sim as in Algorithm 2.

> In all NLP formulations: the load transfer creates an implicit dependency where F_z depends on A (the variable being optimised). This is handled by evaluating F_z inside each constraint call using the current iterate value of A. The NLP solver handles this correctly — do not pre-compute F_z outside the solver loop.

---

References: Jazar — Vehicle Dynamics (eqs 10.84–10.86, 10.108–10.115) · Rajamani — Vehicle Dynamics and Control Ch.2 · Pacejka — Tyre and Vehicle Dynamics Ch.4 · Brayshaw & Harrison (2005) — IMechE Proc. 