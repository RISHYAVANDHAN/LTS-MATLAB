function track = track_loader(xy_or_file, varargin)
%TRACK_LOADER  Build a smooth track representation from centerline points.
%
%   track = TRACK_LOADER(xy)
%   track = TRACK_LOADER(filename)
%   track = TRACK_LOADER(..., 'Name', Value, ...)
%
%   Inputs
%   ------
%   xy_or_file : either:
%       - Nx2 numeric array of [X Y] coordinates [m]
%       - string or char array containing path to .xlsx or .xls file
%
%   Optional name-value pairs
%   -------------------------
%   'CloseTrack'   : true/false, close the loop (default true)
%   'SmoothFactor' : oversampling factor (default 10)
%   'XCol', 'YCol' : column names/indices for Excel files (auto‑detect if not given)
%
%   Output
%   ------
%   track : struct with .s, .x, .y, .kappa, .heading, .ds, .N, .L
%
%   If a filename is given, the function reads the file, auto‑detects X/Y
%   columns, and processes the track.

    %% --------------------------------------------------------------------
    %  1) Parse inputs
    %  --------------------------------------------------------------------
    p = inputParser;
    p.addRequired('xy_or_file', @(z) isnumeric(z) || ischar(z) || isstring(z));
    p.addParameter('CloseTrack', true, @(z) islogical(z) && isscalar(z));
    p.addParameter('SmoothFactor', 10, @(z) isnumeric(z) && isscalar(z) && z>=1);
    p.addParameter('XCol', [], @(z) ischar(z) || isstring(z) || isnumeric(z));
    p.addParameter('YCol', [], @(z) ischar(z) || isstring(z) || isnumeric(z));
    p.parse(xy_or_file, varargin{:});

    closeTrack = p.Results.CloseTrack;
    smoothFactor = round(p.Results.SmoothFactor);

    %% --------------------------------------------------------------------
    %  2) Get numeric xy array (from file or direct input)
    %  --------------------------------------------------------------------
    input = p.Results.xy_or_file;

    if isnumeric(input)
        % Direct numeric input – use as is
        xy = input;
        if size(xy,2) ~= 2
            error('track_loader:BadInput', 'Numeric input must be Nx2 [X Y].');
        end
    else
        % It's a file – read it
        filename = char(input);
        if ~exist(filename, 'file')
            error('track_loader:FileNotFound', 'File not found: %s', filename);
        end

        fprintf('Loading track from %s...\n', filename);
        T = readtable(filename);

        % Get X and Y columns
        xCol = p.Results.XCol;
        yCol = p.Results.YCol;

        if isempty(xCol) || isempty(yCol)
            % Auto‑detect
            colNames = T.Properties.VariableNames;
            findCol = @(patterns) find(cellfun(@(c) any(contains(lower(c), patterns)), colNames), 1);

            xIdx = findCol({'x', 'easting', 'long', 'lon'});
            yIdx = findCol({'y', 'northing', 'lat'});

            if isempty(xIdx) || isempty(yIdx) || xIdx == yIdx
                % Fallback: first two numeric columns
                numericVars = varfun(@isnumeric, T, 'OutputFormat', 'uniform');
                numericCols = find(numericVars);
                if numel(numericCols) < 2
                    error('track_loader:NoNumericCols', 'Could not find numeric columns for X and Y.');
                end
                xIdx = numericCols(1);
                yIdx = numericCols(2);
                fprintf('Auto‑detected: using "%s" and "%s" as X and Y.\n', ...
                        colNames{xIdx}, colNames{yIdx});
            else
                fprintf('Auto‑detected: using "%s" and "%s" as X and Y.\n', ...
                        colNames{xIdx}, colNames{yIdx});
            end
        else
            % User specified columns – resolve
            if isnumeric(xCol)
                xIdx = xCol;
            else
                xIdx = find(strcmpi(T.Properties.VariableNames, char(xCol)));
            end
            if isnumeric(yCol)
                yIdx = yCol;
            else
                yIdx = find(strcmpi(T.Properties.VariableNames, char(yCol)));
            end
        end

        % Extract data
        xData = T{:, xIdx};
        yData = T{:, yIdx};

        % Remove NaNs
        valid = ~isnan(xData) & ~isnan(yData);
        xy = [xData(valid), yData(valid)];

        if size(xy,1) < 3
            error('track_loader:NotEnoughPoints', 'Need at least 3 valid points.');
        end
        fprintf('Loaded %d points.\n', size(xy,1));
    end

    %% --------------------------------------------------------------------
    %  3) Close the track if requested
    %  --------------------------------------------------------------------
    x = xy(:,1);
    y = xy(:,2);

    if closeTrack
        if hypot(x(1)-x(end), y(1)-y(end)) > 1e-8
            x = [x; x(1)];
            y = [y; y(1)];
        end
    end

    Nraw = numel(x);

    %% --------------------------------------------------------------------
    %  4) Raw arc length and reparameterize
    %  --------------------------------------------------------------------
    ds_raw = hypot(diff(x), diff(y));
    s_raw = [0; cumsum(ds_raw)];
    L = s_raw(end);

    if L <= 0
        error('track_loader:DegenerateTrack', 'Track length is zero or invalid.');
    end

    Ns = max(3, smoothFactor * (Nraw - 1));
    s = linspace(0, L, Ns)';

    % Interpolate onto uniform arc length
    x_s = interp1(s_raw, x, s, 'pchip');
    y_s = interp1(s_raw, y, s, 'pchip');

    %% --------------------------------------------------------------------
    %  5) Derivatives and curvature
    %  --------------------------------------------------------------------
    dx_ds = gradient(x_s, s);
    dy_ds = gradient(y_s, s);
    d2x_ds2 = gradient(dx_ds, s);
    d2y_ds2 = gradient(dy_ds, s);

    heading = atan2(dy_ds, dx_ds);
    heading = unwrap(heading);

    % Curvature: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
    denom = (dx_ds.^2 + dy_ds.^2).^(3/2);
    denom = max(denom, 1e-12);
    kappa = (dx_ds .* d2y_ds2 - dy_ds .* d2x_ds2) ./ denom;

    % For closed track, make first/last consistent
    if closeTrack
        kappa(1) = 0.5 * (kappa(1) + kappa(end));
        kappa(end) = kappa(1);
    end

    ds = diff(s);
    ds = max(ds, 1e-6);

    %% --------------------------------------------------------------------
    %  6) Package output
    %  --------------------------------------------------------------------
    track.s = s;
    track.x = x_s;
    track.y = y_s;
    track.kappa = kappa;
    track.heading = heading;
    track.ds = ds;
    track.N = Ns;
    track.L = L;
    track.closeTrack = closeTrack;
    track.smoothFactor = smoothFactor;
end