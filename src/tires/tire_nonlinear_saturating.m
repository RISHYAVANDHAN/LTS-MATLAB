function [Fx, Fy, Mz] = tire_nonlinear_saturating(Fz, alpha, kappa, p, axle)
%TIRE_NONLINEAR_SATURATING  Tanh‑based saturating tyre (HTML #t2).
%   Pure slip: Fy = -mu_y Fz tanh( (C_alpha/(mu_y Fz0))*(Fz/Fz0)*alpha )
%   Combined slip: friction ellipse scaling.

    switch axle
        case 'front'
            C_alpha = p.tyre.front.C_alpha;
            C_kappa = p.tyre.front.C_kappa;
            mu_x = p.tyre.front.mu_x;
            mu_y = p.tyre.front.mu_y;
        case 'rear'
            C_alpha = p.tyre.rear.C_alpha;
            C_kappa = p.tyre.rear.C_kappa;
            mu_x = p.tyre.rear.mu_x;
            mu_y = p.tyre.rear.mu_y;
        otherwise
            error('Unknown axle');
    end

    Fz_eff = max(Fz, 1e-3);
    Fz0 = p.vehicle.Fzf0;  % reference load (approximate)

    % Pure slip
    Fx_pure = mu_x * Fz_eff * tanh( (C_kappa / (mu_x * Fz0)) * (Fz_eff / Fz0) * kappa );
    Fy_pure = -mu_y * Fz_eff * tanh( (C_alpha / (mu_y * Fz0)) * (Fz_eff / Fz0) * alpha );

    % Combined slip (friction ellipse)
    if p.model.use_combined_slip
        Fx_norm = Fx_pure / (mu_x * Fz_eff);
        Fy_norm = Fy_pure / (mu_y * Fz_eff);
        lam = sqrt(Fx_norm^2 + Fy_norm^2);
        if lam > 1
            scale = 1 / lam;
            Fx = scale * Fx_pure;
            Fy = scale * Fy_pure;
        else
            Fx = Fx_pure;
            Fy = Fy_pure;
        end
    else
        Fx = Fx_pure;
        Fy = Fy_pure;
    end

    Mz = 0;
end