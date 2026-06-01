function sol = lap_backward_pass(track, v_lim, ggv, p)
%LAP_BACKWARD_PASS  Backward braking pass for a lap-time speed profile.
%
%   sol = LAP_BACKWARD_PASS(track, v_lim, ggv, p)
%
%   Inputs
%   ------
%   track : struct from track_loader.m
%   v_lim : curvature-based speed limit [m/s]
%   ggv   : struct from ggv_envelope_v2.m
%   p     : parameter struct from init_params.m
%
%   Output
%   ------
%   sol.v_bwd : backward-pass speed profile [m/s]
%   sol.t_bwd : cumulative time [s]
%   sol.ax_bwd : longitudinal acceleration used [m/s^2]
%
%   This version uses the speed-dependent GGV braking limit instead of a
%   fixed braking cap. That keeps the backward pass consistent with the
%   nonlinear tire model and the envelope generation step.

s = track.s(:);
ds = track.ds(:);
N = numel(s);

if numel(v_lim) ~= N
    error('lap_backward_pass:SizeMismatch', 'v_lim must have same length as track.s');
end

if ~isfield(ggv, 'v') || ~isfield(ggv, 'ax_neg')
    error('lap_backward_pass:MissingGGV', 'ggv must contain fields .v and .ax_neg');
end

v_grid = ggv.v(:);
ax_grid = ggv.ax_neg(:);

v_bwd = zeros(N,1);
ax_bwd = zeros(N,1);
t_bwd = zeros(N,1);

% Finish speed: small positive value, clipped to the local limit
v_bwd(N) = min(v_lim(N), 0.5 * max(v_lim));

for i = N:-1:2
    % Current node speed limited by curvature envelope
    v_i = min(v_bwd(i), v_lim(i));

    % Interpolate braking acceleration magnitude from the GGV envelope
    a_brake = interp1(v_grid, ax_grid, v_i, 'linear', 'extrap');
    a_brake = max(a_brake, 0.0);

    % Backward kinematic update over the previous segment:
    % v_{i-1}^2 = v_i^2 + 2*a_brake*ds
    v_prev = sqrt(max(v_i^2 + 2*a_brake*ds(i-1), 0));

    % Respect previous-node curvature limit
    v_bwd(i-1) = min(v_prev, v_lim(i-1));
    ax_bwd(i-1) = -a_brake;

    % Time update over the segment using average speed
    v_avg = max(0.5 * (v_bwd(i-1) + v_i), 1e-3);
    t_bwd(i-1) = t_bwd(i) + ds(i-1) / v_avg;
end

% Last entry
ax_bwd(N) = ax_bwd(max(N-1,1));

sol.v_bwd = v_bwd;
sol.t_bwd = t_bwd;
sol.ax_bwd = ax_bwd;
sol.v_lim = v_lim;
sol.ggv = ggv;

end
