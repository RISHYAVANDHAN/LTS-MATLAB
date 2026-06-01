function out = simulate_corner()
%SIMULATE_CORNER  Constant-steer cornering test for the bicycle model.
%
%   out = SIMULATE_CORNER()
%
%   This test applies a fixed steering angle and constant drive torque to
%   check whether the vehicle generates:
%   - nonzero yaw rate
%   - nonzero lateral velocity
%   - a curved path in the global frame
%
%   Use this after SIMULATE_STRAIGHT to verify that the sign conventions
%   and tire-force directions are correct in cornering.

%% ------------------------------------------------------------------------
%  1) Load parameters
%  ------------------------------------------------------------------------

p = init_params();

%% ------------------------------------------------------------------------
%  2) Simulation horizon
%  ------------------------------------------------------------------------

Tend = 10.0;
tspan = [0 Tend];

%% ------------------------------------------------------------------------
%  3) Initial state
%  ------------------------------------------------------------------------

x0 = zeros(8,1);
x0(1) = 5.0;           % initial forward speed [m/s]
x0(2) = 0.0;           % lateral velocity [m/s]
x0(3) = 0.0;           % yaw rate [rad/s]
x0(4) = x0(1)/p.Rw;    % front wheel speed [rad/s]
x0(5) = x0(1)/p.Rw;    % rear wheel speed [rad/s]
x0(6) = 0.0;           % X [m]
x0(7) = 0.0;           % Y [m]
x0(8) = 0.0;           % yaw angle [rad]

%% ------------------------------------------------------------------------
%  4) Control input
%  ------------------------------------------------------------------------
%  Small steering angle first. Increase later if needed.

steer_deg = 25.0;
delta = deg2rad(steer_deg);
Tf = 0.0;
Tr = 80.0;

u_const = [delta; Tf; Tr];

rhs = @(t, x) bicycle_rhs(t, x, u_const, p);

%% ------------------------------------------------------------------------
%  5) Integrate
%  ------------------------------------------------------------------------

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t, x] = ode45(rhs, tspan, x0, opts);

%% ------------------------------------------------------------------------
%  6) Extract states
%  ------------------------------------------------------------------------

u   = x(:,1);
v   = x(:,2);
r   = x(:,3);
wf  = x(:,4);
wr  = x(:,5);
X   = x(:,6);
Y   = x(:,7);
psi = x(:,8);

speed = sqrt(u.^2 + v.^2);

%% ------------------------------------------------------------------------
%  7) Print checks
%  ------------------------------------------------------------------------

fprintf('Cornering test finished.\n');
fprintf('Steering angle        = %.2f deg\n', steer_deg);
fprintf('Final speed           = %.3f m/s\n', speed(end));
fprintf('Final lateral velocity= %.6f m/s\n', v(end));
fprintf('Final yaw rate        = %.6f rad/s\n', r(end));
fprintf('Final yaw angle       = %.6f rad\n', psi(end));
fprintf('Final Y displacement  = %.6f m\n', Y(end));

%% ------------------------------------------------------------------------
%  8) Plots
%  ------------------------------------------------------------------------

figure('Name','Cornering test','Color','w');

subplot(3,1,1)
plot(t, u, 'LineWidth', 1.5); hold on;
plot(t, v, 'LineWidth', 1.5);
plot(t, speed, '--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity [m/s]');
grid on;
legend('u','v','|V|','Location','best');

subplot(3,1,2)
plot(t, r, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Yaw rate [rad/s]');
grid on;

subplot(3,1,3)
plot(X, Y, 'LineWidth', 1.5);
xlabel('X [m]');
ylabel('Y [m]');
title('Path in global frame');
grid on;
axis equal;

%% ------------------------------------------------------------------------
%  9) Output struct
%  ------------------------------------------------------------------------

if nargout > 0
    out.t = t;
    out.x = x;
    out.u_const = u_const;
    out.p = p;
    out.speed = speed;
end

end
