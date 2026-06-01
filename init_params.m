function p = init_params()
%INIT_PARAMS  Central parameter definition for lap-time simulation.
%
%   p = INIT_PARAMS() returns a struct containing the vehicle,
%   tire, numerical, and track-related parameters used by the
%   bicycle-model lap-time simulator.
%
%   Keep all constants here so the dynamics and solver files remain free
%   of hard-coded numbers.

%% ------------------------------------------------------------------------
%  Vehicle parameters
%  ------------------------------------------------------------------------

% Mass properties
p.m  = 300.0;      % vehicle mass [kg]      <-- replace with your car
p.Iz = 120.0;      % yaw inertia [kg*m^2]    <-- replace with your car

% Geometry
p.a  = 0.80;       % CG to front axle [m]
p.b  = 0.70;       % CG to rear axle [m]
p.L  = p.a + p.b;  % wheelbase [m]

% Wheel / drivetrain
p.Rw = 0.228;      % effective wheel radius [m]
p.Jw = 1.20;       % equivalent wheel inertia per wheel [kg*m^2]

% CG height / gravity / simple load transfer support
p.h  = 0.25;       % CG height [m]
p.g  = 9.81;       % gravity [m/s^2]

% Aerodynamics (optional; can be set to zero for first model)
p.rho = 1.225;     % air density [kg/m^3]
p.CdA = 0.0;       % drag area [m^2] (set later if needed)
p.ClA = 0.0;       % downforce area [m^2] (set later if needed)
p.xAero = 0.0;     % aero application point from CG [m]

%% ------------------------------------------------------------------------
%  Tire model parameters
%  ------------------------------------------------------------------------
%  These are the parameters used by the nonlinear tire force law in
%  tire_forces.m. If you have measured or fitted values, replace these.

% Small-slip linear stiffnesses (used inside the nonlinear saturation law)
p.Caf = 60000.0;   % front lateral stiffness [N/rad]
p.Car = 60000.0;   % rear lateral stiffness [N/rad]
p.Cxf = 8000.0;    % front longitudinal stiffness [N]
p.Cxr = 8000.0;    % rear longitudinal stiffness [N]

% Peak friction coefficients by axle/direction
p.mu_xf = 1.80;
p.mu_yf = 1.80;
p.mu_xr = 1.80;
p.mu_yr = 1.80;

% Nonlinear shape parameters for the simplified saturating tire law.
% These are not full Magic Formula coefficients; they control how quickly
% the force approaches saturation.
p.Bx = 10.0;
p.By = 8.0;
p.Sx = 1.0;
p.Sy = 1.0;

% Numerical guard for slip calculations
p.epsV = 1e-3;

%% ------------------------------------------------------------------------
%  Actuator limits
%  ------------------------------------------------------------------------

% Steering
p.delta_max = deg2rad(18.0);        % max steering angle [rad]
p.delta_rate_max = deg2rad(300.0);  % max steering rate [rad/s]

% Drive / brake torques
p.T_drive_max = 350.0;    % max drive torque at wheel [N*m]
p.T_brake_max = 500.0;    % max brake torque magnitude [N*m]

%% ------------------------------------------------------------------------
%  Normal load initialization
%  ------------------------------------------------------------------------

% Static axle loads
p.Fzf0 = p.m * p.g * p.b / p.L;  % front static normal load [N]
p.Fzr0 = p.m * p.g * p.a / p.L;  % rear static normal load [N]

%% ------------------------------------------------------------------------
%  Numerical settings
%  ------------------------------------------------------------------------

% Integration / discretization
p.dt = 0.01;       % default time step [s]
p.tol = 1e-8;      % general solver tolerance [dimensionless]

%% ------------------------------------------------------------------------
%  Track / lap-time settings
%  ------------------------------------------------------------------------

p.trackWidth = 1.5;    % half-width or lane width placeholder [m]
p.ds = 0.25;           % spatial discretization step [m]
p.useTrackBoundaries = true;

%% ------------------------------------------------------------------------
%  Notes
%  ------------------------------------------------------------------------
%  1) For the first working model, use the nonlinear saturating tire law
%     with static loads.
%  2) Once straight-line and cornering tests work, add load transfer.
%  3) Once the vehicle model is stable, connect the track and lap solver.

end
