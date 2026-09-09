function [A_max, trim] = optimiser_nlp(v, phi, p)
%OPTIMISER_NLP  Constrained NLP (HTML #o4) for bicycle or four‑wheel.

    switch p.model.vehicle_type
        case 'bicycle'
            nVar = 4;  % A, delta, beta, kappa_r
            lb = [0; -p.steering.delta_max; -0.3; -1];
            ub = [inf;  p.steering.delta_max;  0.3;  1];
            x0 = [5; 0; 0; 0];
            obj = @(x) -x(1);
            nonlcon = @(x) nlp_constraints_bicycle(x, v, phi, p);
        case 'four_wheel'
            nVar = 7;  % A, delta, kappa_fl, kappa_fr, kappa_rl, kappa_rr, beta
            lb = [0; -p.steering.delta_max; -1; -1; -1; -1; -0.3];
            ub = [inf;  p.steering.delta_max;  1;  1;  1;  1;  0.3];
            x0 = [5; 0; 0; 0; 0; 0; 0];
            obj = @(x) -x(1);
            nonlcon = @(x) nlp_constraints_fourwheel(x, v, phi, p);
        otherwise
            error('NLP not supported for %s', p.model.vehicle_type);
    end

    options = p.solver.fmincon_options;
    [x_opt, ~, exitflag] = fmincon(obj, x0, [], [], [], [], lb, ub, nonlcon, options);
    if exitflag <= 0
        warning('fmincon did not converge for v=%.2f, phi=%.2f', v, phi);
    end

    A_max = x_opt(1);
    if strcmp(p.model.vehicle_type, 'bicycle')
        trim = struct('delta', x_opt(2), 'beta', x_opt(3), 'kappa_r', x_opt(4));
    else
        trim = struct('delta', x_opt(2), 'beta', x_opt(7), ...
                      'kappa_fl', x_opt(3), 'kappa_fr', x_opt(4), ...
                      'kappa_rl', x_opt(5), 'kappa_rr', x_opt(6));
    end
end

function [c, ceq] = nlp_constraints_bicycle(x, v, phi, p)
    A = x(1); delta = x(2); beta = x(3); kappa_r = x(4);
    res = vehicle_bicycle_residuals([delta; beta; kappa_r], v, phi, A, p);
    ceq = res;
    % Inequality: tyre ellipse
    r = A * sin(phi) / max(v, 1e-3);
    u = v * cos(beta);
    v_y = v * sin(beta);
    alpha_f = delta - atan2(v_y + p.vehicle.geometry.a * r, u);
    alpha_r = -atan2(v_y - p.vehicle.geometry.b * r, u);
    [Fzf, Fzr] = compute_normal_loads(A*cos(phi), A*sin(phi), v, p);
    [Fx_f, Fy_f, ~] = tire_model(Fzf, alpha_f, 0, p, 'front');
    [Fx_r, Fy_r, ~] = tire_model(Fzr, alpha_r, kappa_r, p, 'rear');
    c_f = (Fx_f/(p.tyre.front.mu_x*Fzf))^2 + (Fy_f/(p.tyre.front.mu_y*Fzf))^2 - 1;
    c_r = (Fx_r/(p.tyre.rear.mu_x*Fzr))^2 + (Fy_r/(p.tyre.rear.mu_y*Fzr))^2 - 1;
    c = [c_f; c_r];
end

function [c, ceq] = nlp_constraints_fourwheel(x, v, phi, p)
    A = x(1); delta = x(2);
    kappa_fl = x(3); kappa_fr = x(4); kappa_rl = x(5); kappa_rr = x(6); beta = x(7);
    res = vehicle_four_wheel_residuals([delta; kappa_fl; kappa_fr; kappa_rl; kappa_rr; beta], v, phi, A, p);
    ceq = res;
    % Inequality for each wheel
    r = A * sin(phi) / max(v, 1e-3);
    u = v * cos(beta);
    v_y = v * sin(beta);
    a = p.vehicle.geometry.a; b = p.vehicle.geometry.b;
    t_f = p.vehicle.geometry.track.f / 2; t_r = p.vehicle.geometry.track.r / 2;
    [~, ~, Fz_fl, Fz_fr, Fz_rl, Fz_rr] = compute_normal_loads(A*cos(phi), A*sin(phi), v, p);
    alpha_fl = delta - atan2(v_y + a*r + t_f*r, u);
    alpha_fr = delta - atan2(v_y + a*r - t_f*r, u);
    alpha_rl = -atan2(v_y - b*r + t_r*r, u);
    alpha_rr = -atan2(v_y - b*r - t_r*r, u);
    [Fx_fl, Fy_fl, ~] = tire_model(Fz_fl, alpha_fl, kappa_fl, p, 'front');
    [Fx_fr, Fy_fr, ~] = tire_model(Fz_fr, alpha_fr, kappa_fr, p, 'front');
    [Fx_rl, Fy_rl, ~] = tire_model(Fz_rl, alpha_rl, kappa_rl, p, 'rear');
    [Fx_rr, Fy_rr, ~] = tire_model(Fz_rr, alpha_rr, kappa_rr, p, 'rear');
    c_fl = (Fx_fl/(p.tyre.front.mu_x*Fz_fl))^2 + (Fy_fl/(p.tyre.front.mu_y*Fz_fl))^2 - 1;
    c_fr = (Fx_fr/(p.tyre.front.mu_x*Fz_fr))^2 + (Fy_fr/(p.tyre.front.mu_y*Fz_fr))^2 - 1;
    c_rl = (Fx_rl/(p.tyre.rear.mu_x*Fz_rl))^2 + (Fy_rl/(p.tyre.rear.mu_y*Fz_rl))^2 - 1;
    c_rr = (Fx_rr/(p.tyre.rear.mu_x*Fz_rr))^2 + (Fy_rr/(p.tyre.rear.mu_y*Fz_rr))^2 - 1;
    c = [c_fl; c_fr; c_rl; c_rr];
end