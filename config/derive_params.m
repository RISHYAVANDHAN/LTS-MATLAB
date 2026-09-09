function p = derive_params(p)
%DERIVE_PARAMS  Compute derived quantities from init_params.

    m = p.vehicle.mass;
    g = p.environment.g;
    a = p.vehicle.geometry.a;
    b = p.vehicle.geometry.b;
    L = a + b;

    p.vehicle.geometry.L = L;
    p.vehicle.Fzf0 = m * g * b / L;
    p.vehicle.Fzr0 = m * g * a / L;

    % Check if aero balance sums to 1
    if abs(p.aero.balance_f + (1 - p.aero.balance_f) - 1) > 1e-10
        warning('Aero balance does not sum to 1; resetting to 0.5');
        p.aero.balance_f = 0.5;
    end
end