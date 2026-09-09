function validate_params(p)
%VALIDATE_PARAMS  Check physical consistency.

    assert(p.vehicle.mass > 0, 'Mass must be > 0');
    assert(p.vehicle.inertia.yaw > 0, 'Yaw inertia must be > 0');
    assert(p.vehicle.geometry.a > 0 && p.vehicle.geometry.b > 0, 'Wheelbase portions must be positive');
    assert(p.tyre.front.C_alpha > 0 && p.tyre.rear.C_alpha > 0, 'Cornering stiffness must be > 0');
    assert(p.tyre.front.mu_y > 0 && p.tyre.rear.mu_y > 0, 'Friction coefficients must be > 0');
    assert(p.solver.tolerance > 0, 'Tolerance must be positive');
    assert(p.model.use_load_transfer == false || p.vehicle.geometry.h_cg > 0, 'CG height needed for load transfer');
    fprintf('All parameters validated.\n');
end