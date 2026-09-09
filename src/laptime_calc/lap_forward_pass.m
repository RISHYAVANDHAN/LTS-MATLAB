function sol = lap_forward_pass(track, v_lim, ggv, p)
%LAP_FORWARD_PASS  Pass 2: acceleration out of corners (HTML #p23).

    N = track.N;
    ds = track.ds;
    v = zeros(N,1);
    t = zeros(N,1);
    ax = zeros(N,1);

    v(1) = min(v_lim(1), 0.5);
    for i = 1:N-1
        v_i = min(v(i), v_lim(i));
        ay = v_i^2 * abs(track.kappa(i));
        a_max = ggv.interp_ax_pos(v_i, ay);
        if isnan(a_max) || a_max < 0
            a_max = 0;
        end
        v_next = sqrt(max(v_i^2 + 2 * a_max * ds(i), 0));
        v(i+1) = min(v_next, v_lim(i+1));
        ax(i) = a_max;
        v_avg = max(0.5*(v_i + v(i+1)), 1e-3);
        t(i+1) = t(i) + ds(i) / v_avg;
    end
    ax(N) = ax(N-1);
    sol.v_fwd = v;
    sol.t_fwd = t;
    sol.ax_fwd = ax;
end