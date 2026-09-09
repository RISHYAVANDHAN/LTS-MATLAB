function [A_max, trim] = optimiser_fixed_point(v, phi, p)
%OPTIMISER_FIXED_POINT  Fixed‑point iteration for bicycle + linear tyre (HTML #o2).

    if ~strcmp(p.model.vehicle_type, 'bicycle') || ~strcmp(p.tyre.model, 'linear')
        error('Fixed‑point requires bicycle + linear tyre.');
    end

    A_lo = 0; A_hi = 2 * p.tyre.front.mu_y * p.environment.g;
    for iter_bisect = 1:50
        A = (A_lo + A_hi)/2;
        ax = A * cos(phi);
        ay = A * sin(phi);
        r = ay / max(v, 1e-3);

        beta = 0;
        delta = ay * p.vehicle.geometry.L / v^2;
        for k = 1:100
            u = v * cos(beta);
            v_y = v * sin(beta);
            alpha_f = delta - atan2(v_y + p.vehicle.geometry.a * r, u);
            alpha_r = -atan2(v_y - p.vehicle.geometry.b * r, u);
            Fy_f = -p.tyre.front.C_alpha * alpha_f;
            Fy_r = -p.tyre.rear.C_alpha * alpha_r;

            Mz = p.vehicle.geometry.a * Fy_f - p.vehicle.geometry.b * Fy_r;
            delta_new = delta - Mz / (p.vehicle.geometry.a * p.tyre.front.C_alpha);
            Fy_total = Fy_f + Fy_r;
            beta_new = beta - (Fy_total / p.vehicle.mass - ay) * p.vehicle.mass / (p.tyre.front.C_alpha + p.tyre.rear.C_alpha);
            if abs(delta_new-delta) < 1e-6 && abs(beta_new-beta) < 1e-6
                break;
            end
            delta = delta_new; beta = beta_new;
        end

        [Fzf, Fzr] = compute_normal_loads(ax, ay, v, p);
        Fy_f_lim = p.tyre.front.mu_y * Fzf;
        Fy_r_lim = p.tyre.rear.mu_y * Fzr;
        if abs(Fy_f) <= Fy_f_lim && abs(Fy_r) <= Fy_r_lim
            A_lo = A;
        else
            A_hi = A;
        end
    end
    A_max = A_lo;
    trim = struct('delta', delta, 'beta', beta, 'kappa_r', 0);
end