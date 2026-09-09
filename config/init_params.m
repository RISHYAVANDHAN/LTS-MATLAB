function p = init_params()
%INIT_PARAMS  Default vehicle, tyre, aero, and solver parameters.

    p.vehicle.mass = 300.0;
    p.vehicle.inertia.yaw = 120.0;
    p.vehicle.geometry.a = 0.80;
    p.vehicle.geometry.b = 0.70;
    p.vehicle.geometry.L = p.vehicle.geometry.a + p.vehicle.geometry.b;
    p.vehicle.geometry.h_cg = 0.25;
    p.vehicle.geometry.track.f = 1.2;
    p.vehicle.geometry.track.r = 1.2;
    p.vehicle.wheels.R_eff = 0.228;
    p.vehicle.wheels.J = 1.20;

    p.environment.g = 9.81;
    p.environment.rho = 1.225;

    p.aero.enabled = false;
    p.aero.CdA = 0.0;
    p.aero.ClA = 0.0;
    p.aero.x_CP = 0.0;
    p.aero.balance_f = 0.5;

    p.tyre.model = 'nonlinear_saturating';
    p.tyre.front.C_alpha = 60000;
    p.tyre.rear.C_alpha  = 60000;
    p.tyre.front.C_kappa = 8000;
    p.tyre.rear.C_kappa  = 8000;
    p.tyre.front.mu_x = 1.8;
    p.tyre.front.mu_y = 1.8;
    p.tyre.rear.mu_x  = 1.8;
    p.tyre.rear.mu_y  = 1.8;
    p.tyre.nonlinear.Bx = 10.0;
    p.tyre.nonlinear.By = 8.0;
    p.tyre.nonlinear.Sx = 1.0;
    p.tyre.nonlinear.Sy = 1.0;
    % Pacejka (simplified, will be used if model='pacejka')
    p.tyre.pacejka.front.Bx = 10; p.tyre.pacejka.front.Cx = 1.3; p.tyre.pacejka.front.Dx = 1.8; p.tyre.pacejka.front.Ex = 0.97;
    p.tyre.pacejka.front.By = 10; p.tyre.pacejka.front.Cy = 1.3; p.tyre.pacejka.front.Dy = 1.8; p.tyre.pacejka.front.Ey = 0.97;
    p.tyre.pacejka.front.BMz = 8; p.tyre.pacejka.front.CMz = 1.2; p.tyre.pacejka.front.DMz = 0.05; p.tyre.pacejka.front.EMz = 0.9;
    p.tyre.pacejka.rear = p.tyre.pacejka.front;

    p.steering.delta_max = deg2rad(18);
    p.steering.delta_rate_max = deg2rad(300);

    p.powertrain.T_drive_max = 350;
    p.powertrain.T_brake_max = 500;
    p.powertrain.drivetrain_efficiency = 1.0;

    p.track.ds = 0.25;
    p.track.use_boundaries = false;

    p.solver.quasiStatic.spatial_step = p.track.ds;
    p.solver.transient.time_step = 0.01;
    p.solver.tolerance = 1e-8;
    p.solver.fsolve_options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-8, 'TolX', 1e-8);
    p.solver.fmincon_options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'interior-point', ...
        'SpecifyObjectiveGradient', false, 'SpecifyConstraintGradient', false, ...
        'TolFun', 1e-8, 'TolX', 1e-8, 'MaxIter', 500);

    p.model.name = 'quasiStatic';
    p.model.vehicle_type = 'bicycle';
    p.model.tyre_model = 'nonlinear_saturating';
    p.model.optimiser = 'fsolve';
    p.model.use_aero = false;
    p.model.use_load_transfer = false;
    p.model.use_combined_slip = true;

    % These will be filled in derive_params
    p.vehicle.Fzf0 = [];
    p.vehicle.Fzr0 = [];
end