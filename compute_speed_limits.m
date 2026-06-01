function v_lim = compute_speed_limits(track, ggv)
%COMPUTE_SPEED_LIMITS  Curvature-based speed limit along the track.
%
%   v_lim = COMPUTE_SPEED_LIMITS(track, ggv)
%
%   Inputs
%   ------
%   track : struct from track_loader.m
%   ggv   : struct from ggv_envelope.m or ggv_envelope_v2.m
%
%   Output
%   ------
%   v_lim : maximum allowable speed at each track point [m/s]
%
%   This file converts path curvature into a speed ceiling using the
%   lateral acceleration envelope from the GGV diagram.
%
%   Core relation:
%       a_y = v^2 * |kappa| <= a_y,max(v)
%
%   so:
%       v_lim = sqrt(a_y,max / |kappa|)
%
%   Because a_y,max may itself vary with speed, this implementation uses a
%   conservative representative value from the GGV envelope for the first
%   working version. Once you have the full GGV optimizer, this file can be
%   upgraded to solve the implicit speed-limit relation point-by-point.

kappa = track.kappa(:);
N = numel(kappa);

if isfield(ggv, 'ay_max') && ~isempty(ggv.ay_max)
    ay_ref = min(ggv.ay_max);
else
    error('compute_speed_limits:MissingField', 'ggv must contain ay_max.');
end

if isfield(ggv, 'v') && ~isempty(ggv.v)
    v_cap = max(ggv.v);
else
    v_cap = inf;
end

v_lim = zeros(N,1);
for i = 1:N
    k = abs(kappa(i));
    if k < 1e-10
        % On straight sections there is no curvature-limited speed cap from
        % lateral acceleration alone.
        v_lim(i) = v_cap;
    else
        v_lim(i) = sqrt(max(ay_ref, 0) / k);
        v_lim(i) = min(v_lim(i), v_cap);
    end
end

% Replace any numerical issues with the maximum available speed.
v_lim(~isfinite(v_lim)) = v_cap;

end
