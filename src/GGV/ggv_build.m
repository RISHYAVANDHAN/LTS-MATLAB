function ggv = ggv_build(p)
%GGV_BUILD  Build GGV surface over speed and direction.

    v_grid = linspace(5, 40, 50);
    phi_grid = linspace(0, 2*pi, 72);
    nV = numel(v_grid);
    nP = numel(phi_grid);

    A_table = zeros(nV, nP);
    trim_table = cell(nV, nP);

    for i = 1:nV
        v = v_grid(i);
        for j = 1:nP
            phi = phi_grid(j);
            [A, trim] = optimiser(v, phi, p);
            A_table(i,j) = A;
            trim_table{i,j} = trim;
        end
        fprintf('GGV: v=%.1f m/s done.\n', v);
    end

    ax_table = A_table .* cos(phi_grid);
    ay_table = A_table .* sin(phi_grid);

    ax_pos = []; ay_pos = []; v_pos = [];
    ax_neg = []; ay_neg = []; v_neg = [];
    for i = 1:nV
        for j = 1:nP
            ax = ax_table(i,j);
            ay = ay_table(i,j);
            if ax >= 0
                ax_pos = [ax_pos; ax];
                ay_pos = [ay_pos; ay];
                v_pos = [v_pos; v_grid(i)];
            else
                ax_neg = [ax_neg; ax];
                ay_neg = [ay_neg; ay];
                v_neg = [v_neg; v_grid(i)];
            end
        end
    end

    % CHANGE: extrapolate linearly instead of returning NaN
    F_pos = scatteredInterpolant(v_pos, ay_pos, ax_pos, 'linear', 'linear');
    F_neg = scatteredInterpolant(v_neg, ay_neg, ax_neg, 'linear', 'linear');

    ggv.v_grid = v_grid;
    ggv.phi_grid = phi_grid;
    ggv.A = A_table;
    ggv.ax = ax_table;
    ggv.ay = ay_table;
    ggv.interp_ax_pos = F_pos;
    ggv.interp_ax_neg = F_neg;
    ggv.trim = trim_table;

    % Max lateral (for Pass 1)
    ay_max = zeros(nV,1);
    for i = 1:nV
        [~, idx] = min(abs(phi_grid - pi/2));
        ay_max(i) = A_table(i, idx);
    end
    ggv.ay_max = ay_max;
end