function v_lim = compute_speed_limits(track, ggv, p)
%COMPUTE_SPEED_LIMITS  Pass 1: v²|κ| = ay_max(v) (HTML #p1).

    v_lim = zeros(track.N, 1);
    ay_max_func = @(v) interp1(ggv.v_grid, ggv.ay_max, v, 'linear', 'extrap');

    for i = 1:track.N
        k = abs(track.kappa(i));
        if k < 1e-8
            v_lim(i) = max(ggv.v_grid);
        else
            v_guess = 10;
            for iter = 1:20
                v_new = sqrt(ay_max_func(v_guess) / k);
                if abs(v_new - v_guess) < 1e-4
                    break;
                end
                v_guess = v_new;
            end
            v_lim(i) = v_guess;
        end
    end
end