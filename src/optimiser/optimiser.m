function [A_max, trim] = optimiser(v, phi, p)
%OPTIMISER  Dispatch to selected optimiser.

    switch p.model.optimiser
        case 'friction_circle'
            [A_max, trim] = optimiser_friction_circle(v, phi, p);
        case 'fixed_point'
            [A_max, trim] = optimiser_fixed_point(v, phi, p);
        case 'fsolve'
            [A_max, trim] = optimiser_fsolve(v, phi, p);
        case 'nlp'
            [A_max, trim] = optimiser_nlp(v, phi, p);
        case 'mmm'
            [A_max, trim] = optimiser_mmm(v, phi, p);
        otherwise
            error('Unknown optimiser: %s', p.model.optimiser);
    end
end