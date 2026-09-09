function Tlap = compute_lap_time(track, sol)
%COMPUTE_LAP_TIME  Integral lap time (HTML #lt2).
%   Tlap = ∫ ds / v

    s = track.s;
    v = sol.v;
    N = numel(s);
    Tlap = 0;
    for i = 1:N-1
        ds = track.ds(i);
        v_avg = max(0.5*(v(i)+v(i+1)), 1e-3);
        Tlap = Tlap + ds / v_avg;
    end
end