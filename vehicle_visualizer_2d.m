function vehicle_visualizer_2d(block)
% =========================================================================
%  VEHICLE 2D VISUALIZER  –  Level-2 MATLAB S-Function
%
%  Draws the actual road from road_ref (x_center, y_center, heading)
%  with ±3 m lane boundaries.  Road_ref is read live from base workspace
%  (built once by build_road_ref.m before the simulation starts).
%
%  INPUTS
%   1  X        – vehicle global X      [m]   ← vehicle Out1
%   2  Y        – vehicle global Y      [m]   ← vehicle Out2
%   3  psi      – vehicle heading       [rad] ← vehicle Out4
%   4  speed    – vehicle speed         [m/s] ← vehicle Out3
%   5  delta_sw – steering wheel angle  [rad] ← MPC Out1 (or steering input)
%   6  mu       – road friction coeff   [-]   ← your mu signal
%
%  NO OUTPUTS  (display sink)
%
%  ROAD DISPLAY
%   - Left  boundary : center + 3 m perpendicular to heading
%   - Right boundary : center - 3 m perpendicular to heading
%   - Yellow dashes  : centre line
%   - Green dots     : MPC lookahead horizon (if road_ref available)
%   Road half-width is set by ROAD_HW constant below (default 3.0 m).
% =========================================================================
setup(block);
end


% =========================================================================
function setup(block)
    block.NumInputPorts  = 6;
    block.NumOutputPorts = 0;
    block.SetPreCompInpPortInfoToDynamic;

    for k = 1:6
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = true;
    end

    block.NumContStates  = 0;
    block.SampleTimes    = [0.04 0];     % 25 Hz display
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% =========================================================================
%  POST PROPAGATION SETUP  (DWork must be declared here)
%
%  DWork layout:
%   1  fig_num   [1]     – figure window number
%   2  step      [1]     – frame counter
%   3  trail_x   [TRAIL] – vehicle X history
%   4  trail_y   [TRAIL] – vehicle Y history
%   5  skid_x    [TRAIL] – skid mark X (NaN when no skid)
%   6  skid_y    [TRAIL] – skid mark Y
% =========================================================================
function PostPropSetup(block)
    TRAIL = 600;
    block.NumDworks = 6;

    specs = { 'fig_num',  1; ...
              'step',     1; ...
              'trail_x',  TRAIL; ...
              'trail_y',  TRAIL; ...
              'skid_x',   TRAIL; ...
              'skid_y',   TRAIL };
    for k = 1:6
        block.Dwork(k).Name            = specs{k,1};
        block.Dwork(k).Dimensions      = specs{k,2};
        block.Dwork(k).DatatypeID      = 0;
        block.Dwork(k).Complexity      = 'Real';
        block.Dwork(k).UsedAsDiscState = true;
    end
end


% =========================================================================
function InitConditions(block)
    TRAIL = 600;

    fig = figure('Name',        'Vehicle 2D Visualizer – Road View', ...
                 'Color',       [0.10 0.10 0.10], ...
                 'Position',    [50 50 1100 760], ...
                 'NumberTitle', 'off', ...
                 'MenuBar',     'none', ...
                 'ToolBar',     'figure');

    ax = axes('Parent',    fig, ...
              'Color',     [0.14 0.15 0.14], ...
              'XColor',    [0.60 0.60 0.60], ...
              'YColor',    [0.60 0.60 0.60], ...
              'GridColor', [0.26 0.26 0.26], ...
              'Position',  [0.05 0.10 0.91 0.85]);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X  [m]', 'Color', [0.8 0.8 0.8], 'FontSize', 11);
    ylabel(ax, 'Y  [m]', 'Color', [0.8 0.8 0.8], 'FontSize', 11);
    title(ax, 'Vehicle Dynamics  –  Road View', ...
          'Color', [1 1 1], 'FontSize', 13, 'FontWeight', 'bold');

    % Axes handle lives in fig.UserData (avoids DWork double-precision truncation)
    fig.UserData = ax;

    block.Dwork(1).Data = double(fig.Number);
    block.Dwork(2).Data = 0;
    block.Dwork(3).Data = zeros(TRAIL, 1);
    block.Dwork(4).Data = zeros(TRAIL, 1);
    block.Dwork(5).Data = nan(TRAIL, 1);
    block.Dwork(6).Data = nan(TRAIL, 1);
end


% =========================================================================
%  UPDATE  (25 Hz)
% =========================================================================
function Update(block)
    X        = block.InputPort(1).Data;
    Y        = block.InputPort(2).Data;
    psi      = block.InputPort(3).Data;
    speed    = block.InputPort(4).Data;
    delta_sw = block.InputPort(5).Data;
    mu       = block.InputPort(6).Data;

    % Recover figure + axes
    fig = findobj('Type','figure','Number', block.Dwork(1).Data);
    if isempty(fig) || ~isvalid(fig); return; end
    ax = fig.UserData;
    if isempty(ax) || ~isvalid(ax);   return; end

    TRAIL = 600;

    % Roll trail buffer
    tx = [block.Dwork(3).Data(2:end); X];
    ty = [block.Dwork(4).Data(2:end); Y];
    block.Dwork(3).Data = tx;
    block.Dwork(4).Data = ty;

    % Roll skid marks (appear when fast + low friction)
    sx = block.Dwork(5).Data;
    sy = block.Dwork(6).Data;
    if speed > 4 && mu < 0.50
        sx = [sx(2:end); X + 0.3*(rand-0.5)];
        sy = [sy(2:end); Y + 0.3*(rand-0.5)];
    else
        sx = [sx(2:end); NaN];
        sy = [sy(2:end); NaN];
    end
    block.Dwork(5).Data = sx;
    block.Dwork(6).Data = sy;

    % Load road reference (read fresh every frame – cheap pointer copy)
    road_ref = load_road_ref();

    draw_scene(ax, X, Y, psi, speed, delta_sw, mu, ...
               tx, ty, sx, sy, road_ref, block.CurrentTime, TRAIL);
end


% =========================================================================
function Terminate(block)
    fig = findobj('Type','figure','Number', block.Dwork(1).Data);
    if isempty(fig) || ~isvalid(fig); return; end
    ax = fig.UserData;
    if isempty(ax)  || ~isvalid(ax);  return; end
    text(ax, mean(ax.XLim), mean(ax.YLim), 'SIMULATION ENDED', ...
         'Color',[1 0.3 0.3],'FontSize',22,'FontWeight','bold', ...
         'HorizontalAlignment','center', ...
         'BackgroundColor',[0 0 0 0.65],'EdgeColor','none');
    drawnow;
end


% =========================================================================
%  DRAW SCENE
% =========================================================================
function draw_scene(ax, X, Y, psi, speed, delta_sw, mu, ...
                    trail_x, trail_y, skid_x, skid_y, road_ref, t_sim, TRAIL)

    ROAD_HW   = 3.0;    % [m]  half-width each side of centre line
    SR        = 15.0;
    delta     = delta_sw / SR;
    speed_kmh = speed * 3.6;

    cla(ax);

    % ---- Camera window (follows vehicle, scales with speed) ----
    vr = max(30, speed * 3.5);
    xl = [X - vr,      X + vr*1.6];
    yl = [Y - vr*0.8,  Y + vr*0.8];
    xlim(ax, xl);
    ylim(ax, yl);

    % ================================================================
    %  ROAD DRAWING
    %  Left  boundary = centre + ROAD_HW * perpendicular_left
    %  Right boundary = centre - ROAD_HW * perpendicular_left
    %  Perpendicular left of heading psi: (-sin(psi),  cos(psi))
    %  Perpendicular right:               ( sin(psi), -cos(psi))
    % ================================================================
    if ~isempty(road_ref)
        xc  = road_ref(:,1);
        yc  = road_ref(:,2);
        hd  = road_ref(:,3);

        % Left and right boundary coordinates
        lx  = xc - ROAD_HW * sin(hd);
        ly  = yc + ROAD_HW * cos(hd);
        rx  = xc + ROAD_HW * sin(hd);
        ry  = yc - ROAD_HW * cos(hd);

        % Only draw points inside (or near) the camera window
        % (big performance win on long roads)
        margin = vr * 1.2;
        vis = xc > xl(1)-margin & xc < xl(2)+margin & ...
              yc > yl(1)-margin & yc < yl(2)+margin;

        if any(vis)
            % Find contiguous visible segment with a small buffer around it
            idx_vis = find(vis);
            i_lo = max(1,   idx_vis(1)   - 5);
            i_hi = min(length(xc), idx_vis(end) + 5);
            seg  = i_lo:i_hi;

            % Road asphalt fill (polygon: left boundary + reversed right)
            fill(ax, [lx(seg); flipud(rx(seg))], ...
                     [ly(seg); flipud(ry(seg))], ...
                 [0.20 0.21 0.20], 'EdgeColor','none', 'FaceAlpha',1.0);

            % Road boundaries (white solid lines)
            plot(ax, lx(seg), ly(seg), '-', ...
                 'Color',[0.88 0.88 0.88], 'LineWidth',1.6);
            plot(ax, rx(seg), ry(seg), '-', ...
                 'Color',[0.88 0.88 0.88], 'LineWidth',1.6);

            % Centre dashes (yellow)
            plot(ax, xc(seg), yc(seg), '--', ...
                 'Color',[0.92 0.87 0.12], 'LineWidth',1.1);
        end

    else
        % Fallback: straight road (so visualiser works before road_ref is built)
        fill(ax, [xl(1) xl(2) xl(2) xl(1)], ...
                 [yl(1) yl(1) yl(2) yl(2)], ...
             [0.18 0.18 0.18], 'EdgeColor','none', 'FaceAlpha',0.45);
        plot(ax, xl, [ ROAD_HW  ROAD_HW], '-', ...
             'Color',[0.88 0.88 0.88],'LineWidth',1.4);
        plot(ax, xl, [-ROAD_HW -ROAD_HW], '-', ...
             'Color',[0.88 0.88 0.88],'LineWidth',1.4);
        plot(ax, xl, [0 0], '--', ...
             'Color',[0.92 0.87 0.12],'LineWidth',1.0);
        text(ax, X, yl(2)-4, 'road\_ref not loaded – run build\_road\_ref()', ...
             'Color',[1 0.6 0.1],'FontSize',9,'HorizontalAlignment','center');
    end

    % ================================================================
    %  SKID MARKS
    % ================================================================
    vld = ~isnan(skid_x);
    if any(vld)
        scatter(ax, skid_x(vld), skid_y(vld), 6, ...
                [0.18 0.18 0.18], 'filled', 'MarkerFaceAlpha',0.80);
    end

    % ================================================================
    %  VEHICLE TRAIL  (blue gradient, oldest = dim, newest = bright)
    % ================================================================
    n_seg = 45;
    slen  = floor(TRAIL / n_seg);
    for s = 1:n_seg
        i1  = (s-1)*slen + 1;
        i2  = min(s*slen + 1, TRAIL);
        alp = 0.10 + 0.90*(s/n_seg);
        plot(ax, trail_x(i1:i2), trail_y(i1:i2), '-', ...
             'Color',[0, 0.55*alp, alp, alp], 'LineWidth',1.4);
    end
    % Start-of-trail dot
    plot(ax, trail_x(1), trail_y(1), 'o', ...
         'Color',[0.4 1.0 0.4],'MarkerSize',5,'LineWidth',1.5);

    % ================================================================
    %  VEHICLE BODY
    % ================================================================
    axle_f = 1.08;  axle_r = 1.62;
    car_W  = 1.80;  ovhg_f = 0.85;  ovhg_r = 1.05;

    bx   = [ (axle_f+ovhg_f)  (axle_f+ovhg_f) -(axle_r+ovhg_r) -(axle_r+ovhg_r)];
    by   = [  car_W/2         -car_W/2          -car_W/2          car_W/2        ];
    rf_x = [(axle_f+ovhg_f-0.8) (axle_f+ovhg_f-0.8) -(axle_r+ovhg_r-1.1) -(axle_r+ovhg_r-1.1)];
    rf_y = [ car_W/2-0.2        -car_W/2+0.2          -car_W/2+0.2          car_W/2-0.2        ];
    ws_x = [(axle_f+ovhg_f-0.8)  (axle_f+ovhg_f-0.8)];
    ws_y = [ car_W/2-0.1         -car_W/2+0.1];

    Rv  = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    bw  = Rv*[bx;by]    + [X;Y];
    rw  = Rv*[rf_x;rf_y]+ [X;Y];
    ww  = Rv*[ws_x;ws_y]+ [X;Y];

    bc = body_color(mu);
    fill(ax, [bw(1,:) bw(1,1)], [bw(2,:) bw(2,1)], bc, ...
         'EdgeColor',[0.92 0.92 0.92],'LineWidth',1.8,'FaceAlpha',0.93);
    fill(ax, [rw(1,:) rw(1,1)], [rw(2,:) rw(2,1)], bc*0.63, ...
         'EdgeColor',[0.72 0.72 0.72],'LineWidth',0.9,'FaceAlpha',0.93);
    plot(ax, ww(1,:), ww(2,:), '-', 'Color',[0.5 0.8 1.0],'LineWidth',2.1);

    % ================================================================
    %  WHEELS
    % ================================================================
    tf2 = 1.55/2;   tr2 = 1.56/2;
    wp  = [ axle_f,  tf2;  axle_f, -tf2;  -axle_r,  tr2;  -axle_r, -tr2];
    wa  = [delta, delta, 0, 0];
    wL  = 0.22;   wWh = 0.08;

    for w = 1:4
        Rw2 = [cos(wa(w)) -sin(wa(w)); sin(wa(w)) cos(wa(w))];
        crn = Rw2 * [wL/2 wL/2 -wL/2 -wL/2; wWh/2 -wWh/2 -wWh/2 wWh/2];
        wcv = [crn(1,:)+wp(w,1); crn(2,:)+wp(w,2)];
        wcw = Rv*wcv + [X;Y];
        fill(ax, [wcw(1,:) wcw(1,1)], [wcw(2,:) wcw(2,1)], ...
             [0.10 0.10 0.10], 'EdgeColor',[0.52 0.52 0.52],'LineWidth',1.0);
        sp = Rv*(Rw2*[0 wL*0.42; 0 0]+[wp(w,1);wp(w,2)]) + [X;Y];
        plot(ax, sp(1,:), sp(2,:), '-', 'Color',[0.90 0.60 0.10],'LineWidth',1.9);
    end

    % ================================================================
    %  CG MARKER  +  HEADING ARROW  +  VELOCITY VECTOR
    % ================================================================
    plot(ax, X, Y, '+', 'Color',[1.0 1.0 0.0],'MarkerSize',10,'LineWidth',2.2);

    alen = 3.0 + speed*0.22;
    draw_arrow(ax, X, Y, X+alen*cos(psi), Y+alen*sin(psi), [0.2 1.0 0.3], 2.0, 0.55);

    if speed > 1.0
        % Velocity vector in orange – angle differs from heading when drifting
        vlen = min(speed*0.35, 14);
        plot(ax, [X, X+vlen*cos(psi)], [Y, Y+vlen*sin(psi)], ...
             '-', 'Color',[1.0 0.35 0.10],'LineWidth',2.0);
    end

    % ================================================================
    %  HUD  (top-left corner)
    % ================================================================
    psi_disp = mod(psi + pi, 2*pi) - pi;    % wrap to [-180, 180] deg

    hud = sprintf(['  SPEED   %6.1f km/h\n' ...
                   '  HEADING %+6.1f deg\n'  ...
                   '  STEER   %+6.1f deg\n'  ...
                   '  MU      %.2f'], ...
                  speed_kmh, rad2deg(psi_disp), rad2deg(delta_sw), mu);

    text(ax, xl(1)+0.01*(xl(2)-xl(1)), yl(2)-0.02*(yl(2)-yl(1)), hud, ...
         'Color',[0.12 1.0 0.42],'FontName','Courier New','FontSize',10, ...
         'FontWeight','bold','VerticalAlignment','top', ...
         'BackgroundColor',[0.05 0.05 0.05],'EdgeColor',[0.28 0.28 0.28],'Margin',5);

    % Time stamp (top-right)
    text(ax, xl(2)-0.01*(xl(2)-xl(1)), yl(2)-0.02*(yl(2)-yl(1)), ...
         sprintf('t = %.1f s', t_sim), ...
         'Color',[0.85 0.85 0.20],'FontName','Courier New','FontSize',10, ...
         'HorizontalAlignment','right','VerticalAlignment','top', ...
         'BackgroundColor',[0.05 0.05 0.05],'EdgeColor',[0.28 0.28 0.28],'Margin',5);

    % ================================================================
    %  STATUS BARS (grip + speed)
    % ================================================================
    bw_bar = 0.22*(xl(2)-xl(1));
    bh_bar = 0.012*(yl(2)-yl(1));
    x0     = xl(1) + 0.01*(xl(2)-xl(1));
    y0     = yl(1) + 0.02*(yl(2)-yl(1));

    % Grip bar
    fill(ax, [x0 x0+bw_bar x0+bw_bar x0], [y0 y0 y0+bh_bar y0+bh_bar], ...
         [0.20 0.20 0.20], 'EdgeColor',[0.48 0.48 0.48]);
    fill(ax, [x0 x0+bw_bar*min(mu/1.2,1) x0+bw_bar*min(mu/1.2,1) x0], ...
         [y0 y0 y0+bh_bar y0+bh_bar], bc, 'EdgeColor','none');
    text(ax, x0, y0+bh_bar*1.9, sprintf('Grip  mu = %.2f', mu), ...
         'Color',[0.78 0.78 0.78],'FontSize',8);

    % Speed bar
    x1 = x0 + bw_bar + 0.02*(xl(2)-xl(1));
    sf = min(speed_kmh/200, 1);
    fill(ax, [x1 x1+bw_bar x1+bw_bar x1], [y0 y0 y0+bh_bar y0+bh_bar], ...
         [0.20 0.20 0.20], 'EdgeColor',[0.48 0.48 0.48]);
    fill(ax, [x1 x1+bw_bar*sf x1+bw_bar*sf x1], [y0 y0 y0+bh_bar y0+bh_bar], ...
         [1-sf, sf*0.75, 0.08], 'EdgeColor','none');
    text(ax, x1, y0+bh_bar*1.9, sprintf('Speed  %.0f km/h', speed_kmh), ...
         'Color',[0.78 0.78 0.78],'FontSize',8);

    drawnow limitrate;
end


% =========================================================================
%  READ road_ref FROM BASE WORKSPACE
%  Returns [] if not found (visualiser falls back to straight road)
% =========================================================================
function rr = load_road_ref()
    try
        rr = evalin('base', 'road_ref');
        if size(rr,2) < 3;  rr = [];  end
    catch
        rr = [];
    end
end


% =========================================================================
%  HELPERS
% =========================================================================
function draw_arrow(ax, x1, y1, x2, y2, col, lw, hs)
    plot(ax, [x1 x2], [y1 y2], '-', 'Color',col, 'LineWidth',lw);
    ang = atan2(y2-y1, x2-x1);
    fill(ax, [x2, x2-hs*cos(ang+0.4), x2-hs*cos(ang-0.4)], ...
             [y2, y2-hs*sin(ang+0.4), y2-hs*sin(ang-0.4)], ...
         col, 'EdgeColor','none');
end

function c = body_color(mu)
    if     mu >= 0.8;  c = [0.18 0.52 0.85];   % blue  – dry road
    elseif mu >= 0.4;  c = [0.85 0.55 0.10];   % amber – wet road
    else;              c = [0.80 0.18 0.18];    % red   – ice
    end
end