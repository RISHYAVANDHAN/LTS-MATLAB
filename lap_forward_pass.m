function sol = lap_forward_pass(track, v_lim, ggv, p)
%LAP_FORWARD_PASS  Forward acceleration pass for a lap-time speed profile.
%
%   sol = LAP_FORWARD_PASS(track, v_lim, ggv, p)
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
%   sol.v_fwd : forward-pass speed profile [m/s]
%   sol.t_fwd : cumulative time [s]
%   sol.ax_fwd : longitudinal acceleration used [m/s^2]
%
%   This version uses the speed-dependent GGV forward acceleration limit
%   instead of a constant mu*g cap. That makes the forward pass consistent
%   with the nonlinear tire model and the envelope generation step.

s = track.s(:);
ds = track.ds(:);
N = numel(s);

if numel(v_lim) ~= N
    error('lap_forward_pass:SizeMismatch', 'v_lim must have same length as track.s');
end

if ~isfield(ggv, 'v') || ~isfield(ggv, 'ax_pos')
    error('lap_forward_pass:MissingGGV', 'ggv must contain fields .v and .ax_pos');
end

v_grid = ggv.v(:);
ax_grid = ggv.ax_pos(:);

v_fwd = zeros(N,1);
ax_fwd = zeros(N,1);
t_fwd = zeros(N,1);

% Initial speed: small positive value, clipped to the local limit
v_fwd(1) = min(v_lim(1), 0.5);

for i = 1:N-1
    % Current speed limited by the local curvature envelope
    v_i = min(v_fwd(i), v_lim(i));

    % Interpolate the available forward acceleration from the GGV envelope
    a_max = interp1(v_grid, ax_grid, v_i, 'linear', 'extrap');
    a_max = max(a_max, 0.0);

    % Use the available acceleration over the current segment
    v_next = sqrt(max(v_i^2 + 2*a_max*ds(i), 0));

    % Respect next-node curvature limit
    v_fwd(i+1) = min(v_next, v_lim(i+1));
    ax_fwd(i) = a_max;

    % Time update using mean speed over the segment
    v_avg = max(0.5 * (v_i + v_fwd(i+1)), 1e-3);
    t_fwd(i+1) = t_fwd(i) + ds(i) / v_avg;
end

% Last value
ax_fwd(N) = ax_fwd(max(N-1,1));

sol.v_fwd = v_fwd;
sol.t_fwd = t_fwd;
sol.ax_fwd = ax_fwd;
sol.v_lim = v_lim;
sol.ggv = ggv;

end
