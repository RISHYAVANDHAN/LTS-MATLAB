function [A_max, trim] = optimiser_friction_circle(v, phi, p)
%OPTIMISER_FRICTION_CIRCLE  Analytic circle (HTML #o1).
%   A_max = mu * g_eff,  g_eff = g + aero/mass.

    if ~strcmp(p.model.vehicle_type, 'point_mass')
        error('Friction circle only for point mass.');
    end
    mu = p.tyre.front.mu_y;
    g = p.environment.g;
    if p.aero.enabled
        F_aero = 0.5 * p.environment.rho * p.aero.ClA * v^2;
        g_eff = g + F_aero / p.vehicle.mass;
    else
        g_eff = g;
    end
    A_max = mu * g_eff;
    trim = struct('delta', 0, 'beta', 0, 'kappa', 0);
end