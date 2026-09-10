function [Fx, Fy, Mz] = tire_linear(~, alpha, kappa, p, axle)
%TIRE_LINEAR  Linear tyre model (HTML #t1).
%   Fy = -C_alpha * alpha,  Fx = -C_kappa * kappa,  Mz = t_p * C_alpha * alpha.

    switch axle
        case 'front'
            C_alpha = p.tyre.front.C_alpha;
            C_kappa = p.tyre.front.C_kappa;
        case 'rear'
            C_alpha = p.tyre.rear.C_alpha;
            C_kappa = p.tyre.rear.C_kappa;
        otherwise
            error('Unknown axle');
    end

    Fx = -C_kappa * kappa;
    Fy = -C_alpha * alpha;
    t_p = 0.02;  % pneumatic trail [m]
    Mz = t_p * C_alpha * alpha;
end