function result = run_laptime_sim(xy, p, options)
%RUN_LAPTIME_SIM  Master entry point for the GGV lap‑time simulator.
%
%   result = RUN_LAPTIME_SIM()                   default params & circular track
%   result = RUN_LAPTIME_SIM(xy)                 use your track (Nx2)
%   result = RUN_LAPTIME_SIM(xy, p)              use your parameter struct
%   result = RUN_LAPTIME_SIM(xy, p, options)     set plot/verbose
%
%   OUTPUT: result struct with all intermediate data.

    % ---- Set paths ----
    rootDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(rootDir, 'config'));
    addpath(fullfile(rootDir, 'src', 'tires'));
    addpath(fullfile(rootDir, 'src', 'vehicle'));
    addpath(fullfile(rootDir, 'src', 'optimiser'));
    addpath(fullfile(rootDir, 'src', 'GGV'));
    addpath(fullfile(rootDir, 'src', 'track'));
    addpath(fullfile(rootDir, 'src', 'laptime_calc'));

    % ---- Defaults ----
    if nargin < 1 || isempty(xy)
        R = 30;
        theta = linspace(0, 2*pi, 300)';
        xy = [R*cos(theta), R*sin(theta)];
    end
    if nargin < 2 || isempty(p)
        p = init_params();
        p = derive_params(p);
        validate_params(p);
    end
    if nargin < 3
        options = struct();
    end
    if ~isfield(options, 'plot'), options.plot = true; end
    if ~isfield(options, 'verbose'), options.verbose = true; end

    % ---- Track ----
    if options.verbose, fprintf('Building track...\n'); end
    track = track_loader(xy, 'SmoothFactor', 10, 'CloseTrack', true);

    % ---- GGV ----
    if options.verbose, fprintf('Building GGV surface...\n'); end
    ggv = ggv_build(p);

    % ---- Pass 1 ----
    if options.verbose, fprintf('Pass 1: cornering speed limits...\n'); end
    v_lim = compute_speed_limits(track, ggv, p);

    % ---- Pass 2 ----
    if options.verbose, fprintf('Pass 2: forward acceleration...\n'); end
    fwd = lap_forward_pass(track, v_lim, ggv, p);

    % ---- Pass 3 ----
    if options.verbose, fprintf('Pass 3: backward braking...\n'); end
    bwd = lap_backward_pass(track, v_lim, ggv, p);

    % ---- Merge ----
    if options.verbose, fprintf('Merging passes...\n'); end
    sol = lap_merge(track, fwd, bwd);

    % ---- Lap time ----
    Tlap = compute_lap_time(track, sol);

    % ---- Output ----
    result = struct('p', p, 'track', track, 'ggv', ggv, 'v_lim', v_lim, ...
                    'fwd', fwd, 'bwd', bwd, 'sol', sol, 'Tlap', Tlap, 'options', options);
    fprintf('Lap time = %.3f s\n', Tlap);

    % ---- Plot ----
    if options.plot
        figure('Name', 'Lap-time simulation', 'Color', 'w');
        subplot(3,1,1);
        plot(track.s, sol.v, 'LineWidth', 1.5); hold on;
        plot(track.s, v_lim, '--', 'LineWidth', 1.2);
        xlabel('s [m]'); ylabel('Speed [m/s]'); grid on;
        legend('Final', 'Curvature limit', 'Location', 'best');
        subplot(3,1,2);
        plot(track.s, track.kappa, 'LineWidth', 1.5);
        xlabel('s [m]'); ylabel('Curvature [1/m]'); grid on;
        subplot(3,1,3);
        plot(track.x, track.y, 'LineWidth', 1.5); axis equal; grid on;
        xlabel('X [m]'); ylabel('Y [m]'); title('Track');
    end
end