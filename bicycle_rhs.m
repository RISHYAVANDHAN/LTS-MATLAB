function xdot = bicycle_rhs(t, x, u_in, p)
%BICYCLE_RHS  Right-hand side of the bicycle-model vehicle dynamics.
%
%   xdot = BICYCLE_RHS(t, x, u_in, p)
%
%   This function computes the time derivatives of the 8-state bicycle
%   model used for lap-time simulation.
%
%   State vector
%   ------------
%   x = [u; v; r; wf; wr; X; Y; psi]
%       u    : longitudinal velocity in body frame [m/s]
%       v    : lateral velocity in body frame [m/s]
%       r    : yaw rate [rad/s]
%       wf   : front wheel angular speed [rad/s]
%       wr   : rear wheel angular speed [rad/s]
%       X    : global x position [m]
%       Y    : global y position [m]
%       psi  : yaw angle [rad]
%
%   Input vector
%   ------------
%   u_in = [delta; Tf; Tr]
%       delta : steering angle at the front axle [rad]
%       Tf    : front wheel drive/brake torque [N*m]
%       Tr    : rear wheel drive/brake torque [N*m]
%
%   Notes
%   -----
%   This is the main dynamic model used by ode45 or RK4.
%   It couples the vehicle rigid-body equations to the tire-force model.

%% ------------------------------------------------------------------------
%  1) Unpack states and inputs
%  ------------------------------------------------------------------------

u   = x(1);
v   = x(2);
r   = x(3);
wf  = x(4);
wr  = x(5);
X   = x(6);
Y   = x(7);
psi = x(8);

delta = u_in(1);
Tf    = u_in(2);
Tr    = u_in(3);

%% ------------------------------------------------------------------------
%  2) Guard against very small forward velocity in slip calculations
%  ------------------------------------------------------------------------

u_eff = sign(u) * max(abs(u), p.epsV);
if u_eff == 0
    u_eff = p.epsV;
end

%% ------------------------------------------------------------------------
%  3) Axle velocities in the body frame
%  ------------------------------------------------------------------------
%  Front axle velocity includes yaw-rate contribution +a*r
%  Rear axle velocity includes yaw-rate contribution -b*r

u_f = u;
v_f = v + p.a * r;

u_r = u;
v_r = v - p.b * r;

%% ------------------------------------------------------------------------
%  4) Slip angles
%  ------------------------------------------------------------------------
%  Front tire slip angle is measured relative to the steered wheel heading.
%  Rear tire slip angle is measured relative to the vehicle body heading.

alpha_f = atan2(v_f, max(abs(u_f), p.epsV)) - delta;
alpha_r = atan2(v_r, max(abs(u_r), p.epsV));

%% ------------------------------------------------------------------------
%  5) Longitudinal slip ratios
%  ------------------------------------------------------------------------
%  kappa = (tire circumferential speed - axle speed) / axle speed
%  For low speeds we use a small denominator guard.

kappa_f = (p.Rw * wf - u_f) / max(abs(u_f), p.epsV);
kappa_r = (p.Rw * wr - u_r) / max(abs(u_r), p.epsV);

%% ------------------------------------------------------------------------
%  6) Normal loads
%  ------------------------------------------------------------------------
%  First version: static loads only.
%  Later you can replace these with load transfer, aero, pitch effects, etc.

Fzf = p.Fzf0;
Fzr = p.Fzr0;

% Optional simple longitudinal load transfer placeholder.
% If you want to activate it later, you can estimate ax after force
% calculation and update Fzf/Fzr. For the first version we keep it simple.

%% ------------------------------------------------------------------------
%  7) Tire forces
%  ------------------------------------------------------------------------

[Fx_f, Fy_f] = tire_forces(Fzf, alpha_f, kappa_f, p, 'front');
[Fx_r, Fy_r] = tire_forces(Fzr, alpha_r, kappa_r, p, 'rear');

%% ------------------------------------------------------------------------
%  8) Rigid-body vehicle dynamics
%  ------------------------------------------------------------------------
%  Force balance in the body frame
%  Yaw moment balance about the CG
%
%  Front axle forces must be rotated by steering angle delta into the body
%  frame before contributing to the equations.

Fx_f_body = Fx_f * cos(delta) - Fy_f * sin(delta);
Fy_f_body = Fy_f * cos(delta) + Fx_f * sin(delta);

Fx_r_body = Fx_r;
Fy_r_body = Fy_r;

udot = (Fx_f_body + Fx_r_body) / p.m + v * r;
vdot = (Fy_f_body + Fy_r_body) / p.m - u * r;

rdot = (p.a * Fy_f_body - p.b * Fy_r_body) / p.Iz;

%% ------------------------------------------------------------------------
%  9) Wheel-speed dynamics
%  ------------------------------------------------------------------------
%  Torque balance about each wheel axis.
%  Positive torque accelerates the wheel.

wfdot = (Tf - p.Rw * Fx_f) / p.Jw;
wrdot = (Tr - p.Rw * Fx_r) / p.Jw;

%% ------------------------------------------------------------------------
%  10) Global position and yaw kinematics
%  ------------------------------------------------------------------------

Xdot   = u * cos(psi) - v * sin(psi);
Ydot   = u * sin(psi) + v * cos(psi);
psidot = r;

%% ------------------------------------------------------------------------
%  11) Pack derivatives
%  ------------------------------------------------------------------------

xdot = [udot; vdot; rdot; wfdot; wrdot; Xdot; Ydot; psidot];

end
