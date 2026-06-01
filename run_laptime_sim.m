function result = run_laptime_sim(xy)
%RUN_LAPTIME_SIM  End-to-end lap-time simulation driver.
%
%   result = RUN_LAPTIME_SIM(xy)
%
%   Inputs
%   ------
%   xy : Nx2 array of track centerline points [m]
%
%   Output
%   ------
%   result : struct containing all intermediate and final outputs
%
%   Workflow
%   --------
%   1) Load vehicle/tire parameters
%   2) Build track geometry
%   3) Build GGV envelope
%   4) Compute curvature-based speed limits
%   5) Run forward pass
%   6) Run backward pass
%   7) Merge the passes
%   8) Compute lap time
%   9) Plot results

%% ------------------------------------------------------------------------
%  1) Parameters
%  ------------------------------------------------------------------------

p = init_params();

%% ------------------------------------------------------------------------
%  2) Track
%  ------------------------------------------------------------------------

if nargin < 1 || isempty(xy)
    % Default fallback: simple circle for smoke testing only.
    % Replace this with imported GPS / centerline data for real runs.
    R = 30;
    theta = linspace(0, 2*pi, 300)';
    xy = [R*cos(theta), R*sin(theta)];
end

track = track_loader(xy, 'SmoothFactor', 10, 'CloseTrack', true);

if track.N < 5
    error('run_laptime_sim:BadTrack', 'Track is too short or invalid.');
end

%% ------------------------------------------------------------------------
%  3) GGV envelope
%  ------------------------------------------------------------------------

v_grid = linspace(0, 40, 250);
ggv = ggv_envelope(p, v_grid);

%% ------------------------------------------------------------------------
%  4) Curvature-based speed limits
%  ------------------------------------------------------------------------

v_lim = compute_speed_limits(track, ggv);

%% ------------------------------------------------------------------------
%  5) Forward pass
%  ------------------------------------------------------------------------

fwd = lap_forward_pass(track, v_lim, ggv, p);

%% ------------------------------------------------------------------------
%  6) Backward pass
%  ------------------------------------------------------------------------

bwd = lap_backward_pass(track, v_lim, ggv, p);

%% ------------------------------------------------------------------------
%  7) Merge
%  ------------------------------------------------------------------------

sol = lap_merge(track, fwd, bwd);

%% ------------------------------------------------------------------------
%  8) Lap time
%  ------------------------------------------------------------------------

Tlap = compute_lap_time(track, sol);

%% ------------------------------------------------------------------------
%  9) Display
%  ------------------------------------------------------------------------

fprintf('Lap-time simulation complete.\n');
fprintf('Estimated lap time = %.3f s\n', Tlap);

%% ------------------------------------------------------------------------
%  10) Plots
%  ------------------------------------------------------------------------

figure('Name','Lap-time simulation','Color','w');

subplot(3,1,1)
plot(track.s, sol.v, 'LineWidth', 1.5); hold on;
plot(track.s, v_lim, '--', 'LineWidth', 1.2);
xlabel('s [m]');
ylabel('Speed [m/s]');
grid on;
legend('Final speed','Curvature limit','Location','best');

a = subplot(3,1,2);
plot(track.s, track.kappa, 'LineWidth', 1.5);
xlabel('s [m]');
ylabel('Curvature [1/m]');
grid on;

subplot(3,1,3)
plot(track.x, track.y, 'LineWidth', 1.5); hold on;
plot(track.x_left, track.y_left, ':');
plot(track.x_right, track.y_right, ':');
xlabel('X [m]');
ylabel('Y [m]');
title('Track and boundaries');
grid on;
axis equal;
legend('Centerline','Left boundary','Right boundary','Location','best');

%% ------------------------------------------------------------------------
%  11) Output
%  ------------------------------------------------------------------------

if nargout > 0
    result.p = p;
    result.track = track;
    result.ggv = ggv;
    result.v_lim = v_lim;
    result.fwd = fwd;
    result.bwd = bwd;
    result.sol = sol;
    result.Tlap = Tlap;
end

end
