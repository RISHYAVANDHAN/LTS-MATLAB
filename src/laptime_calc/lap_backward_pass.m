function sol = lap_backward_pass(track, v_lim, ggv, p)
%LAP_BACKWARD_PASS  Pass 3: braking into corners (HTML #p23).

    N = track.N;
    ds = track.ds;
    v = zeros(N,1);
    t = zeros(N,1);
    ax = zeros(N,1);

    v(N) = min(v_lim(N), 0.5);
    for i = N:-1:2
        v_i = min(v(i), v_lim(i));
        ay = v_i^2 * abs(track.kappa(i));
        a_brake = ggv.interp_ax_neg(v_i, ay);
        if isnan(a_brake) || a_brake >= 0
            a_brake = -0.5 * p.environment.g;
        end
        v_prev = sqrt(max(v_i^2 + 2 * a_brake * ds(i-1), 0));
        v(i-1) = min(v_prev, v_lim(i-1));
        ax(i-1) = a_brake;
        v_avg = max(0.5*(v(i-1) + v_i), 1e-3);
        t(i-1) = t(i) - ds(i-1) / v_avg;
    end
    % Recompute cumulative time from start
    t_cum = zeros(N,1);
    for i = 1:N-1
        v_avg = max(0.5*(v(i)+v(i+1)), 1e-3);
        t_cum(i+1) = t_cum(i) + ds(i)/v_avg;
    end
    sol.v_bwd = v;
    sol.t_bwd = t_cum;
    sol.ax_bwd = ax;
end