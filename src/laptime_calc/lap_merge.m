function sol = lap_merge(track, fwd, bwd)
%LAP_MERGE  Merge forward and backward profiles (HTML #p23).
%   v = min(v_fwd, v_bwd)

    vf = fwd.v_fwd;
    vb = bwd.v_bwd;
    v = min(vf, vb);
    N = track.N;
    t = zeros(N,1);
    for i = 1:N-1
        v_avg = max(0.5*(v(i)+v(i+1)), 1e-3);
        t(i+1) = t(i) + track.ds(i) / v_avg;
    end
    ax = zeros(N,1);
    for i = 1:N-1
        ax(i) = (v(i+1)^2 - v(i)^2) / (2*track.ds(i));
    end
    ax(N) = ax(N-1);
    sol.v = v;
    sol.t = t;
    sol.ax = ax;
end