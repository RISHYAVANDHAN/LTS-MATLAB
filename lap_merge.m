function sol = lap_merge(track, fwd, bwd)
%LAP_MERGE  Merge forward and backward speed profiles.
%
%   sol = LAP_MERGE(track, fwd, bwd)
%
%   Inputs
%   ------
%   track : struct from track_loader.m
%   fwd   : struct from lap_forward_pass.m
%   bwd   : struct from lap_backward_pass.m
%
%   Output
%   ------
%   sol.v        : final feasible speed profile [m/s]
%   sol.t        : cumulative lap time [s]
%   sol.ax       : selected longitudinal acceleration [m/s^2]
%   sol.source   : indicator of whether forward or backward bound is active
%
%   Rule:
%   The final speed at each point is the minimum of the forward-pass and
%   backward-pass speed limits. This ensures both acceleration and braking
%   feasibility.

s = track.s(:);
N = numel(s);

vf = fwd.v_fwd(:);
vb = bwd.v_bwd(:);

if numel(vf) ~= N || numel(vb) ~= N
    error('lap_merge:SizeMismatch', 'Speed profiles must match track length.');
end

% Final feasible profile
v = min(vf, vb);

% Cumulative time using trapezoidal-style average speed over each segment
% with a small speed floor to avoid division by zero.
t = zeros(N,1);
for i = 1:N-1
    v_avg = max(0.5 * (v(i) + v(i+1)), 1e-3);
    ds = track.ds(i);
    t(i+1) = t(i) + ds / v_avg;
end

% Select which bound is active at each point
source = strings(N,1);
for i = 1:N
    if abs(vf(i) - vb(i)) < 1e-9
        source(i) = "both";
    elseif vf(i) < vb(i)
        source(i) = "forward";
    else
        source(i) = "backward";
    end
end

% Longitudinal acceleration estimate from finite differences in v^2 vs s
ax = zeros(N,1);
for i = 1:N-1
    ds = track.ds(i);
    if ds > 0
        ax(i) = (v(i+1)^2 - v(i)^2) / (2*ds);
    else
        ax(i) = 0;
    end
end
ax(N) = ax(N-1);

% Package output
sol.v = v;
sol.t = t;
sol.ax = ax;
sol.source = source;
sol.v_forward = vf;
sol.v_backward = vb;

end
