function [Fzf, Fzr, Fz_fl, Fz_fr, Fz_rl, Fz_rr] = compute_normal_loads(ax, ay, v, p)
%COMPUTE_NORMAL_LOADS  Longitudinal/lateral load transfer + aero (HTML #lt).

    m = p.vehicle.mass;
    g = p.environment.g;
    a = p.vehicle.geometry.a;
    b = p.vehicle.geometry.b;
    L = a + b;
    h = p.vehicle.geometry.h_cg;
    t_f = p.vehicle.geometry.track.f;
    t_r = p.vehicle.geometry.track.r;

    % Static
    Fzf0 = m * g * b / L;
    Fzr0 = m * g * a / L;

    % Aero
    if p.aero.enabled
        F_aero = 0.5 * p.environment.rho * p.aero.ClA * v^2;
        Fzf_aero = p.aero.balance_f * F_aero;
        Fzr_aero = (1 - p.aero.balance_f) * F_aero;
    else
        Fzf_aero = 0; Fzr_aero = 0;
    end

    % Longitudinal transfer
    if p.model.use_load_transfer
        dFz_long = m * ax * h / L;
    else
        dFz_long = 0;
    end

    % Lateral transfer
    if p.model.use_load_transfer
        % Simplified: use h for both axles
        dFz_lat_f = (m * ay * h * (b/L)) / (2 * t_f);
        dFz_lat_r = (m * ay * h * (a/L)) / (2 * t_r);
    else
        dFz_lat_f = 0; dFz_lat_r = 0;
    end

    % Axle loads
    Fzf = Fzf0 + Fzf_aero - dFz_long;
    Fzr = Fzr0 + Fzr_aero + dFz_long;

    % Per‑wheel
    Fz_fl = Fzf/2 - dFz_lat_f;
    Fz_fr = Fzf/2 + dFz_lat_f;
    Fz_rl = Fzr/2 - dFz_lat_r;
    Fz_rr = Fzr/2 + dFz_lat_r;

    % Clamp to non‑negative
    Fzf  = max(Fzf, 0);
    Fzr  = max(Fzr, 0);
    Fz_fl = max(Fz_fl, 0);
    Fz_fr = max(Fz_fr, 0);
    Fz_rl = max(Fz_rl, 0);
    Fz_rr = max(Fz_rr, 0);
end