function [A_max, trim] = optimiser_fsolve(v, phi, p)
%OPTIMISER_FSOLVE  Bisection + fsolve (HTML #o3) for bicycle + nonlinear/Pacejka.

    if ~strcmp(p.model.vehicle_type, 'bicycle')
        error('fsolve optimiser requires bicycle vehicle.');
    end

    A_lo = 0;
    A_hi = 2 * p.tyre.front.mu_y * p.environment.g;
    x0 = [0; 0; 0];  % [delta; beta; kappa_r]

    for iter = 1:30
        A = (A_lo + A_hi)/2;
        fun = @(x) vehicle_bicycle_residuals(x, v, phi, A, p);
        options = p.solver.fsolve_options;
        [x_sol, ~, exitflag] = fsolve(fun, x0, options);
        if exitflag <= 0
            A_hi = A;
            continue;
        end

        delta = x_sol(1); beta = x_sol(2); kappa_r = x_sol(3);
        r = A * sin(phi) / max(v, 1e-3);
        u = v * cos(beta);
        v_y = v * sin(beta);
        alpha_f = delta - atan2(v_y + p.vehicle.geometry.a * r, u);
        alpha_r = -atan2(v_y - p.vehicle.geometry.b * r, u);
        [Fzf, Fzr] = compute_normal_loads(A*cos(phi), A*sin(phi), v, p);
        [Fx_f, Fy_f, ~] = tire_model(Fzf, alpha_f, 0, p, 'front');
        [Fx_r, Fy_r, ~] = tire_model(Fzr, alpha_r, kappa_r, p, 'rear');

        load_f = (Fx_f/(p.tyre.front.mu_x*Fzf))^2 + (Fy_f/(p.tyre.front.mu_y*Fzf))^2;
        load_r = (Fx_r/(p.tyre.rear.mu_x*Fzr))^2 + (Fy_r/(p.tyre.rear.mu_y*Fzr))^2;
        if max(load_f, load_r) < 1
            A_lo = A;
            x0 = x_sol;
        else
            A_hi = A;
        end
    end

    A_max = A_lo;
    trim = struct('delta', x0(1), 'beta', x0(2), 'kappa_r', x0(3));
end