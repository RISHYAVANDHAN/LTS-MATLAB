function res = vehicle_four_wheel_residuals(x, v, phi, A, p)
%VEHICLE_FOUR_WHEEL_RESIDUALS  Equilibrium residuals (HTML #v4).
%   x = [delta; kappa_fl; kappa_fr; kappa_rl; kappa_rr; beta]

    delta = x(1);
    kappa_fl = x(2); kappa_fr = x(3);
    kappa_rl = x(4); kappa_rr = x(5);
    beta = x(6);

    r = A * sin(phi) / max(v, 1e-3);
    u = v * cos(beta);
    v_y = v * sin(beta);
    a = p.vehicle.geometry.a;
    b = p.vehicle.geometry.b;
    t_f = p.vehicle.geometry.track.f / 2;
    t_r = p.vehicle.geometry.track.r / 2;

    ax = A * cos(phi);
    ay = A * sin(phi);
    [~, ~, Fz_fl, Fz_fr, Fz_rl, Fz_rr] = compute_normal_loads(ax, ay, v, p);

    % Slip angles (with track width)
    alpha_fl = delta - atan2(v_y + a*r + t_f*r, u);
    alpha_fr = delta - atan2(v_y + a*r - t_f*r, u);
    alpha_rl = -atan2(v_y - b*r + t_r*r, u);
    alpha_rr = -atan2(v_y - b*r - t_r*r, u);

    % Tyre forces
    [Fx_fl, Fy_fl, ~] = tire_model(Fz_fl, alpha_fl, kappa_fl, p, 'front');
    [Fx_fr, Fy_fr, ~] = tire_model(Fz_fr, alpha_fr, kappa_fr, p, 'front');
    [Fx_rl, Fy_rl, ~] = tire_model(Fz_rl, alpha_rl, kappa_rl, p, 'rear');
    [Fx_rr, Fy_rr, ~] = tire_model(Fz_rr, alpha_rr, kappa_rr, p, 'rear');

    % Rotate front
    Fx_fl_body = Fx_fl * cos(delta) - Fy_fl * sin(delta);
    Fy_fl_body = Fy_fl * cos(delta) + Fx_fl * sin(delta);
    Fx_fr_body = Fx_fr * cos(delta) - Fy_fr * sin(delta);
    Fy_fr_body = Fy_fr * cos(delta) + Fx_fr * sin(delta);

    % Total forces
    Fx_total = Fx_fl_body + Fx_fr_body + Fx_rl + Fx_rr;
    Fy_total = Fy_fl_body + Fy_fr_body + Fy_rl + Fy_rr;

    % Yaw moment (full HTML equation)
    Mz_front = a * (Fy_fl_body + Fy_fr_body) + t_f * (Fx_fl_body - Fx_fr_body);
    Mz_rear  = -b * (Fy_rl + Fy_rr) + t_r * (Fx_rl - Fx_rr);
    Mz_total = Mz_front + Mz_rear;

    res = [p.vehicle.mass * ax - Fx_total;
           p.vehicle.mass * ay - Fy_total;
           Mz_total];
end