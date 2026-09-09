function res = vehicle_bicycle_residuals(x, v, phi, A, p)
%VEHICLE_BICYCLE_RESIDUALS  Equilibrium residuals (HTML #v3).
%   x = [delta; beta; kappa_r]  (RWD)
%   Returns [g_long; g_lat; g_yaw] = 0.

    delta = x(1);
    beta  = x(2);
    kappa_r = x(3);

    r = A * sin(phi) / max(v, 1e-3);
    u = v * cos(beta);
    v_y = v * sin(beta);

    % Slip angles (HTML equations)
    alpha_f = delta - atan2(v_y + p.vehicle.geometry.a * r, u);
    alpha_r = -atan2(v_y - p.vehicle.geometry.b * r, u);

    % Normal loads
    ax = A * cos(phi);
    ay = A * sin(phi);
    [Fzf, Fzr] = compute_normal_loads(ax, ay, v, p);

    % Tyre forces
    kappa_f = 0;  % front undriven
    [Fx_f, Fy_f, ~] = tire_model(Fzf, alpha_f, kappa_f, p, 'front');
    [Fx_r, Fy_r, ~] = tire_model(Fzr, alpha_r, kappa_r, p, 'rear');

    % Rotate front
    Fx_f_body = Fx_f * cos(delta) - Fy_f * sin(delta);
    Fy_f_body = Fy_f * cos(delta) + Fx_f * sin(delta);

    % Residuals (HTML equations)
    g_long = p.vehicle.mass * A * cos(phi) - (Fx_f_body + Fx_r);
    g_lat  = p.vehicle.mass * A * sin(phi) - (Fy_f_body + Fy_r);
    g_yaw  = p.vehicle.geometry.a * Fy_f_body - p.vehicle.geometry.b * Fy_r;

    res = [g_long; g_lat; g_yaw];
end