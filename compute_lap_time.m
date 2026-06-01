function Tlap = compute_lap_time(track, profile)
%COMPUTE_LAP_TIME  Compute total lap time from a speed profile.
%
%   Tlap = COMPUTE_LAP_TIME(track, profile)
%
%   Inputs
%   ------
%   track   : struct from track_loader.m
%   profile : struct from lap_merge.m
%
%   Output
%   ------
%   Tlap    : total lap time [s]
%
%   This is the final scalar output for the quasi-steady lap-time solver.

s = track.s(:);
v = profile.v(:);
N = numel(s);

if numel(v) ~= N
    error('compute_lap_time:SizeMismatch', 'Speed profile must match track length.');
end

Tlap = 0.0;
for i = 1:N-1
    ds = track.ds(i);
    v_avg = max(0.5 * (v(i) + v(i+1)), 1e-3);
    Tlap = Tlap + ds / v_avg;
end

end