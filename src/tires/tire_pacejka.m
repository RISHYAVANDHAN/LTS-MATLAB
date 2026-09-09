function [Fx, Fy, Mz] = tire_pacejka(Fz, alpha, kappa, p, axle)
%TIRE_PACEJKA  Magic Formula (HTML #t3).
%   Pure slip: F = D*sin(C*atan(B*x - E*(B*x - atan(B*x))))
%   Combined slip: uses sigma = sqrt((kappa/(1+kappa))^2 + (tan(alpha)/(1+kappa))^2).

    switch axle
        case 'front'
            coeff = p.tyre.pacejka.front;
        case 'rear'
            coeff = p.tyre.pacejka.rear;
        otherwise
            error('Unknown axle');
    end

    Fz_eff = max(Fz, 1e-3);
    mu_x = p.tyre.front.mu_x;  % we use axle‑specific if needed
    mu_y = p.tyre.front.mu_y;

    % Longitudinal pure
    D_x = mu_x * Fz_eff;
    B_x = coeff.Bx; C_x = coeff.Cx; E_x = coeff.Ex;
    Fx_pure = D_x * sin(C_x * atan(B_x * kappa - E_x * (B_x * kappa - atan(B_x * kappa))));

    % Lateral pure
    D_y = mu_y * Fz_eff;
    B_y = coeff.By; C_y = coeff.Cy; E_y = coeff.Ey;
    Fy_pure = -D_y * sin(C_y * atan(B_y * alpha - E_y * (B_y * alpha - atan(B_y * alpha))));

    % Combined slip (HTML equation after Pacejka)
    if p.model.use_combined_slip
        sigma = sqrt( (kappa/(1+kappa))^2 + (tan(alpha)/(1+kappa))^2 );
        if sigma > 1e-6
            Fx = (kappa/(1+kappa) / sigma) * Fx_pure;
            Fy = (tan(alpha)/(1+kappa) / sigma) * Fy_pure;
        else
            Fx = Fx_pure;
            Fy = Fy_pure;
        end
    else
        Fx = Fx_pure;
        Fy = Fy_pure;
    end

    % Aligning moment (own Magic Formula)
    t0 = 0.02;  % pneumatic trail at zero slip
    D_Mz = -t0 * p.tyre.front.C_alpha;  % approximate
    B_Mz = coeff.BMz; C_Mz = coeff.CMz; E_Mz = coeff.EMz;
    Mz = D_Mz * sin(C_Mz * atan(B_Mz * alpha - E_Mz * (B_Mz * alpha - atan(B_Mz * alpha))));
end