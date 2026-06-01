function out = simulate_straight()
%SIMULATE_STRAIGHT  Straight-line test for the bicycle-model simulator.
%
%   out = SIMULATE_STRAIGHT()
%
%   This script/function runs a simple straight-line acceleration test to
%   verify that:
%   - the vehicle model is numerically stable
%   - the tire force signs are correct
%   - the wheel-speed dynamics behave sensibly
%   - the state ordering in bicycle_rhs.m is correct
%
%   The test uses zero steering and constant rear drive torque.

%% ------------------------------------------------------------------------
%  1) Load parameters
%  ------------------------------------------------------------------------

p = init_params();

%% ------------------------------------------------------------------------
%  2) Define simulation horizon
%  ------------------------------------------------------------------------

Tend = 8.0;                 % total simulation time [s]
tspan = [0 Tend];

%% ------------------------------------------------------------------------
%  3) Initial state
%  ------------------------------------------------------------------------

% State vector:
% x = [u; v; r; wf; wr; X; Y; psi]

x0 = zeros(8,1);
x0(1) = 0.5;    % small initial forward speed [m/s]
x0(2) = 0.0;    % lateral velocity [m/s]
x0(3) = 0.0;    % yaw rate [rad/s]
x0(4) = x0(1)/p.Rw;  % front wheel speed [rad/s] consistent with speed
x0(5) = x0(1)/p.Rw;  % rear wheel speed [rad/s] consistent with speed
x0(6) = 0.0;    % X [m]
x0(7) = 0.0;    % Y [m]
x0(8) = 0.0;    % yaw angle [rad]

%% ------------------------------------------------------------------------
%  4) Input schedule
%  ------------------------------------------------------------------------
%  For a first test, use constant steering = 0 and constant rear drive.
%  Front torque is set to zero.

u_const = [0.0; 0.0; 120.0];  % [delta; Tf; Tr]

% Wrap the dynamics for ode45
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

%% ------------------------------------------------------------------------
%  7) Compute simple post-processing quantities
%  ------------------------------------------------------------------------

speed = sqrt(u.^2 + v.^2);

%% ------------------------------------------------------------------------
%  8) Print basic checks
%  ------------------------------------------------------------------------

fprintf('Straight-line test finished.\n');
fprintf('Final speed           = %.3f m/s\n', speed(end));
fprintf('Final lateral velocity= %.6f m/s\n', v(end));
fprintf('Final yaw rate        = %.6f rad/s\n', r(end));
fprintf('Final Y displacement  = %.6f m\n', Y(end));

%% ------------------------------------------------------------------------
%  9) Plots
%  ------------------------------------------------------------------------

figure('Name','Straight-line test','Color','w');

subplot(3,1,1)
plot(t, u, 'LineWidth', 1.5); hold on;
plot(t, speed, '--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Speed [m/s]');
grid on;
legend('u','|V|','Location','best');

subplot(3,1,2)
plot(t, v, 'LineWidth', 1.5); hold on;
plot(t, r, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Lateral vel / yaw rate');
grid on;
legend('v [m/s]','r [rad/s]','Location','best');

subplot(3,1,3)
plot(X, Y, 'LineWidth', 1.5);
xlabel('X [m]');
ylabel('Y [m]');
title('Path in global frame');
grid on;
axis equal;

%% ------------------------------------------------------------------------
% 10) Output struct
% ------------------------------------------------------------------------

if nargout > 0
    out.t = t;
    out.x = x;
    out.u_const = u_const;
    out.p = p;
    out.speed = speed;
end

end
