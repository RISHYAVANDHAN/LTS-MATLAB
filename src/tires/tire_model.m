function [Fx, Fy, Mz] = tire_model(Fz, alpha, kappa, p, axle)
%TIRE_MODEL  Dispatch to selected tyre model.

    switch p.tyre.model
        case 'linear'
            [Fx, Fy, Mz] = tire_linear(Fz, alpha, kappa, p, axle);
        case 'nonlinear_saturating'
            [Fx, Fy, Mz] = tire_nonlinear_saturating(Fz, alpha, kappa, p, axle);
        case 'pacejka'
            [Fx, Fy, Mz] = tire_pacejka(Fz, alpha, kappa, p, axle);
        otherwise
            error('Unknown tyre model: %s', p.tyre.model);
    end
end