function track = track_loader(xy, varargin)
%TRACK_LOADER  Build a smooth track representation from centerline points.
%
%   track = TRACK_LOADER(xy)
%   track = TRACK_LOADER(xy, 'Name', Value, ...)
%
%   Inputs
%   ------
%   xy : Nx2 array of centerline points [m]
%        xy(:,1) = X coordinates
%        xy(:,2) = Y coordinates
%
%   Optional name-value pairs
%   -------------------------
%   'SmoothFactor' : integer oversampling factor for spline interpolation
%                    (default = 10)
%   'CloseTrack'    : true/false, whether to enforce a closed loop
%                    (default = true)
%
%   Output
%   ------
%   track : struct containing arc length, heading, curvature, and segment
%           information for lap-time simulation.
%
%   This version is more practical for a lap-time solver than a raw finite-
%   difference geometry extractor because it:
%   - optionally closes the centerline
%   - reparameterizes by arc length
%   - spline-smooths the path
%   - computes heading and curvature on the smoothed path

%% ------------------------------------------------------------------------
%  1) Parse inputs
%  ------------------------------------------------------------------------

if size(xy,2) ~= 2
    error('track_loader:BadInput', 'xy must be an Nx2 array of [X Y] points.');
end

p = inputParser;
p.addRequired('xy', @(z) isnumeric(z) && size(z,2) == 2 && size(z,1) >= 3);
p.addParameter('SmoothFactor', 10, @(z) isnumeric(z) && isscalar(z) && z >= 1);
p.addParameter('CloseTrack', true, @(z) islogical(z) && isscalar(z));
p.parse(xy, varargin{:});

smoothFactor = round(p.Results.SmoothFactor);
closeTrack = p.Results.CloseTrack;

x = xy(:,1);
y = xy(:,2);

%% ------------------------------------------------------------------------
%  2) Force closed loop if requested
%  ------------------------------------------------------------------------

if closeTrack
    if hypot(x(1)-x(end), y(1)-y(end)) > 1e-8
        x = [x; x(1)];
        y = [y; y(1)];
    end
end

Nraw = numel(x);

%% ------------------------------------------------------------------------
%  3) Raw arc length parameter
%  ------------------------------------------------------------------------

ds_raw = hypot(diff(x), diff(y));
s_raw = [0; cumsum(ds_raw)];
L = s_raw(end);

if L <= 0
    error('track_loader:DegenerateTrack', 'Track length is zero or invalid.');
end

%% ------------------------------------------------------------------------
%  4) Spline reparameterization by arc length
%  ------------------------------------------------------------------------

Ns = max(3, smoothFactor * (Nraw-1));
s = linspace(0, L, Ns)';

% Shape-preserving interpolation for a smoother path
x_s = interp1(s_raw, x, s, 'pchip');
y_s = interp1(s_raw, y, s, 'pchip');

%% ------------------------------------------------------------------------
%  5) Derivatives and curvature
%  ------------------------------------------------------------------------

% First derivatives with respect to arc length
dx_ds = gradient(x_s, s);
dy_ds = gradient(y_s, s);

% Heading angle
psi = unwrap(atan2(dy_ds, dx_ds));

% Curvature kappa = dpsi/ds
kappa = gradient(psi, s);

%% ------------------------------------------------------------------------
%  6) Segment lengths and midpoint curvature
%  ------------------------------------------------------------------------

ds = hypot(diff(x_s), diff(y_s));
s_mid = 0.5 * (s(1:end-1) + s(2:end));
kappa_mid = 0.5 * (kappa(1:end-1) + kappa(2:end));

%% ------------------------------------------------------------------------
%  7) Approximate left/right boundaries from a constant half-width
%  ------------------------------------------------------------------------
%  This is a placeholder geometric track envelope. If you have real
%  boundaries, replace this with imported left/right edge coordinates.

if isfield(p.Results, 'TrackWidth') %#ok<*ISFIELD>
    trackWidth = p.Results.TrackWidth;
else
    trackWidth = NaN;
end

% Default width can be overwritten later in the solver.
trackHalfWidth = 1.5;

nx = -sin(psi);
ny =  cos(psi);

x_left  = x_s + trackHalfWidth * nx;
y_left  = y_s + trackHalfWidth * ny;
x_right = x_s - trackHalfWidth * nx;
y_right = y_s - trackHalfWidth * ny;

%% ------------------------------------------------------------------------
%  8) Package output
%  ------------------------------------------------------------------------

track.x = x_s;
track.y = y_s;
track.s = s;
track.psi = psi;
track.kappa = kappa;
track.ds = ds;
track.s_mid = s_mid;
track.kappa_mid = kappa_mid;
track.L = L;
track.N = Ns;
track.x_left = x_left;
track.y_left = y_left;
track.x_right = x_right;
track.y_right = y_right;
track.halfWidth = trackHalfWidth;
track.closeTrack = closeTrack;
track.smoothFactor = smoothFactor;

end
