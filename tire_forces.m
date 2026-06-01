function [Fx, Fy] = tire_forces(Fz, alpha, kappa, p, axle)
%TIRE_FORCES  Nonlinear tire force model for the bicycle-model simulator.
%
%   [Fx, Fy] = TIRE_FORCES(Fz, alpha, kappa, p, axle)
%
%   Inputs
%   ------
%   Fz    : normal load on the tire [N]
%   alpha : slip angle [rad]
%   kappa : longitudinal slip ratio [-]
%   p     : parameter struct from init_params.m
%   axle  : 'front' or 'rear' (selects axle-specific parameters)
%
%   Outputs
%   -------
%   Fx    : longitudinal tire force [N]
%   Fy    : lateral tire force [N]
%
%   Notes
%   -----
%   This version is a practical nonlinear tire model for early lap-time
%   simulation and GGV generation.
%
%   It uses:
%   - linear stiffness around zero slip
%   - smooth nonlinear saturation using tanh()
%   - a combined-slip reduction using an ellipse-style scaling
%
%   It is not a full Magic Formula implementation, but it is a strong
%   upgrade over a purely linear tire law with a hard force cap.

% -------------------------------------------------------------------------
% 1) Safety / defaults
% -------------------------------------------------------------------------

Fz = max(Fz, 0.0);
Fz_eff = max(Fz, 1.0);
epsF = 1e-9;

% Helper for missing fields
getOr = @(s, name, default) local_getfield_or(s, name, default);

% -------------------------------------------------------------------------
% 2) Select axle-specific parameters
% -------------------------------------------------------------------------

switch lower(string(axle))
    case "front"
        Cx_lin = getOr(p, 'Cxf', 8000.0);
        Cy_lin = getOr(p, 'Caf', 60000.0);
        mu_x   = getOr(p, 'mu_xf', getOr(p, 'mu', 1.8));
        mu_y   = getOr(p, 'mu_yf', getOr(p, 'mu', 1.8));
    case "rear"
        Cx_lin = getOr(p, 'Cxr', 8000.0);
        Cy_lin = getOr(p, 'Car', 60000.0);
        mu_x   = getOr(p, 'mu_xr', getOr(p, 'mu', 1.8));
        mu_y   = getOr(p, 'mu_yr', getOr(p, 'mu', 1.8));
    otherwise
        error('tire_forces:InvalidAxle', 'axle must be ''front'' or ''rear''.');
end

% Optional nonlinear-shape parameters.
% If not provided, use moderate defaults.
Bx = getOr(p, 'Bx', 10.0);
By = getOr(p, 'By', 8.0);

% Saturation sharpness: larger means faster approach to the peak.
% If you later calibrate a proper Magic Formula, these can be replaced.
Sx = getOr(p, 'Sx', 1.0);
Sy = getOr(p, 'Sy', 1.0);

% -------------------------------------------------------------------------
% 3) Pure longitudinal and lateral force build-up
% -------------------------------------------------------------------------
% The force initially grows linearly with slip, then smoothly saturates.
% This gives a nonlinear tire without requiring full Pacejka parameters.

Fx_lin = Cx_lin * kappa;
Fy_lin = -Cy_lin * alpha;

Fx_pure = mu_x * Fz_eff * tanh(Sx * Fx_lin / (mu_x * Fz_eff + epsF));
Fy_pure = mu_y * Fz_eff * tanh(Sy * Fy_lin / (mu_y * Fz_eff + epsF));

% -------------------------------------------------------------------------
% 4) Combined-slip reduction
% -------------------------------------------------------------------------
% Ensure simultaneous longitudinal and lateral force demand stays inside
% an ellipse-like tire capacity region.

Fx_n = Fx_pure / (mu_x * Fz_eff + epsF);
Fy_n = Fy_pure / (mu_y * Fz_eff + epsF);
lam = sqrt(Fx_n^2 + Fy_n^2);

if lam <= 1.0
    Fx = Fx_pure;
    Fy = Fy_pure;
else
    scale = 1.0 / lam;
    Fx = scale * Fx_pure;
    Fy = scale * Fy_pure;
end

% -------------------------------------------------------------------------
% 5) Final safety clamp
% -------------------------------------------------------------------------

Fcap_x = mu_x * Fz_eff;
Fcap_y = mu_y * Fz_eff;
Fx = min(max(Fx, -Fcap_x), Fcap_x);
Fy = min(max(Fy, -Fcap_y), Fcap_y);

end

% ========================================================================
% Local helper
% ========================================================================
function val = local_getfield_or(s, name, default)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = default;
end
end
