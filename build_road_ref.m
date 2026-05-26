function road_ref = build_road_ref()
% =========================================================================
%  BUILD_ROAD_REF
%  Reads x_center, y_center, heading (radians, N×1 column vectors)
%  from the MATLAB base workspace and produces the [N×3] road_ref matrix
%  required by the MPC controller.
%
%  USAGE
%    road_ref = build_road_ref();
%
%  REQUIRED WORKSPACE VARIABLES
%    x_center  – [N×1] double  – road centreline X  [m]
%    y_center  – [N×1] double  – road centreline Y  [m]
%    heading   – [N×1] double  – road heading       [rad]
%
%  OUTPUT
%    road_ref  – [M×3] double  – [x, y, heading_rad]
%                Resampled to uniform 1 m arc-length spacing.
%                Also written to base workspace as "road_ref".
%
%  WHAT THIS FUNCTION DOES
%    1. Reads the three variables from workspace
%    2. Removes duplicate consecutive points (GPS noise)
%    3. Fills any NaN/Inf heading values from path tangent
%    4. Smooths heading (unwrap → 5-pt moving average → re-wrap)
%    5. Resamples to exactly 1 m arc-length spacing (pchip)
%    6. Writes road_ref back to base workspace
%    7. Prints a summary table
% =========================================================================

%% Step 1 – Read from workspace -------------------------------------------
fprintf('[build_road_ref] Reading workspace variables...\n');

x_center = evalin('base', 'x_center');   % N×1 double, metres
y_center = evalin('base', 'y_center');   % N×1 double, metres
heading  = evalin('base', 'heading');    % N×1 double, radians

N = length(x_center);
assert(length(y_center) == N && length(heading) == N, ...
    'x_center, y_center and heading must all have the same length.');

fprintf('  Loaded %d waypoints.\n', N);

%% Step 2 – Remove consecutive duplicate XY points -----------------------
% (Can appear in GPS / map data)
dxy  = sqrt(diff(x_center).^2 + diff(y_center).^2);
keep = [true; dxy > 1e-6];          % keep first, then any that moved
x_c  = x_center(keep);
y_c  = y_center(keep);
h_c  = heading(keep);
if sum(~keep) > 0
    fprintf('  Removed %d duplicate points.\n', sum(~keep));
end

%% Step 3 – Fill NaN / Inf heading from path tangent ----------------------
bad = ~isfinite(h_c);
if any(bad)
    fprintf('  Filling %d NaN/Inf heading values from path tangent.\n', sum(bad));
    h_tangent = path_tangent(x_c, y_c);
    h_c(bad) = h_tangent(bad);
end

%% Step 4 – Smooth heading ------------------------------------------------
h_c = smooth_heading(h_c);

%% Step 5 – Uniform arc-length resampling to 1 m --------------------------
arc  = [0; cumsum(sqrt(diff(x_c).^2 + diff(y_c).^2))];
total_len = arc(end);

% Remove any remaining duplicate arc values after de-duplication
[arc_u, ui] = unique(arc);
x_u = x_c(ui);
y_u = y_c(ui);
h_u = unwrap(h_c(ui));     % unwrap before interpolation to avoid jumps

DS          = 1.0;                          % [m] target spacing
arc_new     = (0 : DS : total_len)';

x_rs = interp1(arc_u, x_u, arc_new, 'pchip');
y_rs = interp1(arc_u, y_u, arc_new, 'pchip');
h_rs = interp1(arc_u, h_u, arc_new, 'pchip');
h_rs = mod(h_rs + pi, 2*pi) - pi;          % re-wrap to [-pi, pi]

%% Step 6 – Assemble and write to workspace --------------------------------
road_ref = [x_rs, y_rs, h_rs];
assignin('base', 'road_ref', road_ref);

%% Step 7 – Summary --------------------------------------------------------
fprintf('[build_road_ref] Done.\n');
fprintf('  %-25s %d  →  %d waypoints\n', 'Points (in → out):', N, length(x_rs));
fprintf('  %-25s %.1f m\n',              'Total path length:',  total_len);
fprintf('  %-25s %.1f m\n',              'Waypoint spacing:',   DS);
fprintf('  %-25s [%.1f, %.1f] m\n',     'X range:',  min(x_rs), max(x_rs));
fprintf('  %-25s [%.1f, %.1f] m\n',     'Y range:',  min(y_rs), max(y_rs));
fprintf('  %-25s [%.1f, %.1f] deg\n',   'Heading range:', ...
        rad2deg(min(h_rs)), rad2deg(max(h_rs)));
fprintf('  road_ref saved to base workspace.\n\n');

end  % build_road_ref


% =========================================================================
%  HELPERS
% =========================================================================
function psi = path_tangent(x, y)
% Forward-difference heading from XY coordinates.
    n   = length(x);
    psi = zeros(n, 1);
    for i = 1:n-1
        psi(i) = atan2(y(i+1) - y(i), x(i+1) - x(i));
    end
    psi(n) = psi(n-1);
end


function psi_s = smooth_heading(psi)
% Unwrap → 5-pt moving-average → re-wrap.
    psi_uw = unwrap(psi);
    w      = min(5, length(psi_uw));
    kernel = ones(w, 1) / w;
    psi_sm = conv(psi_uw, kernel, 'same');
    % Restore edges corrupted by convolution
    h = floor(w/2);
    psi_sm(1:h)       = psi_uw(1:h);
    psi_sm(end-h+1:end) = psi_uw(end-h+1:end);
    psi_s = mod(psi_sm + pi, 2*pi) - pi;
end