function ggv = ggv_envelope(p, v_grid)
%GGV_ENVELOPE  Build a robust G-G-V envelope for the lap-time simulator.
%
%   ggv = GGV_ENVELOPE(p, v_grid)
%
%   This version is intentionally robust so the lap-time pipeline runs
%   without the singular-matrix issues that can happen in a brittle steady-
%   state trim solve at low speed.
%
%   It returns a speed-dependent envelope based on the nonlinear tire
%   capacity, static load distribution, and simple aero scaling.
%
%   Once the full pipeline is working, this file can be upgraded again to a
%   more detailed optimizer-based steady-state point solver.

if nargin < 2 || isempty(v_grid)
    v_grid = linspace(0, 35, 150);
end

v_grid = v_grid(:);
N = numel(v_grid);

% Preallocate output arrays
ggv.v      = v_grid;
ggv.ax_pos = zeros(N,1);
ggv.ax_neg = zeros(N,1);
ggv.ay_max = zeros(N,1);
ggv.Fx_max = zeros(N,1);
ggv.Fx_min = zeros(N,1);
ggv.Fy_max = zeros(N,1);

% Preallocate per-speed diagnostic points with identical fields so MATLAB
% does not complain about dissimilar structures.
pt_template = struct( ...
    'v', [], ...
    'ax_pos', [], ...
    'ax_neg', [], ...
    'ay_max', [], ...
    'Fx_max', [], ...
    'Fx_min', [], ...
    'Fy_max', [], ...
    'Fzf', [], ...
    'Fzr', [], ...
    'Fd_aero', []);
ggv.points = repmat(pt_template, N, 1);

for i = 1:N
    v = v_grid(i);

    % ---------------------------------------------------------------------
    % Speed-dependent normal load and aero terms
    % ---------------------------------------------------------------------
    Fz_aero = 0.5 * p.rho * p.ClA * v^2;
    Fd_aero = 0.5 * p.rho * p.CdA * v^2;

    % Split downforce equally between axles for the first version.
    Fzf = p.Fzf0 + 0.5 * Fz_aero;
    Fzr = p.Fzr0 + 0.5 * Fz_aero;

    % ---------------------------------------------------------------------
    % Force capacities from the nonlinear tire parameters
    % ---------------------------------------------------------------------
    % Use separate front/rear friction coefficients and keep the result
    % conservative and smooth.
    Fx_drive_cap = max(p.mu_xf * Fzf + p.mu_xr * Fzr - Fd_aero, 0.0);
    Fx_brake_cap = max(p.mu_xf * Fzf + p.mu_xr * Fzr + Fd_aero, 0.0);
    Fy_corner_cap = max(p.mu_yf * Fzf + p.mu_yr * Fzr, 0.0);

    % ---------------------------------------------------------------------
    % Convert to accelerations
    % ---------------------------------------------------------------------
    ax_pos = Fx_drive_cap / p.m;
    ax_neg = Fx_brake_cap / p.m;
    ay_max = Fy_corner_cap / p.m;

    % Store envelope values
    ggv.ax_pos(i) = ax_pos;
    ggv.ax_neg(i) = ax_neg;
    ggv.ay_max(i) = ay_max;
    ggv.Fx_max(i) = Fx_drive_cap;
    ggv.Fx_min(i) = Fx_brake_cap;
    ggv.Fy_max(i) = Fy_corner_cap;

    % Store diagnostics with matching structure fields
    ggv.points(i) = struct( ...
        'v', v, ...
        'ax_pos', ax_pos, ...
        'ax_neg', ax_neg, ...
        'ay_max', ay_max, ...
        'Fx_max', Fx_drive_cap, ...
        'Fx_min', Fx_brake_cap, ...
        'Fy_max', Fy_corner_cap, ...
        'Fzf', Fzf, ...
        'Fzr', Fzr, ...
        'Fd_aero', Fd_aero);
end

fprintf('GGV envelope built from robust nonlinear tire capacity.\n');

end
