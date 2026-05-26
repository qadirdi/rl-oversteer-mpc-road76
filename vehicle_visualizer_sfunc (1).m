function vehicle_visualizer_sfunc(block)
% Level-2 MATLAB S-Function for Vehicle Body 3DOF Visualization
% This block visualizes the vehicle dynamics from the Info Bus

setup(block);

function setup(block)
    % Register number of input ports
    block.NumInputPorts = 1;
    
    % Setup input port properties
    block.InputPort(1).DatatypeID = 0; % Bus signal
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = false;
    block.InputPort(1).SamplingMode = 'Sample';
    
    % Register number of output ports (none needed for visualization)
    block.NumOutputPorts = 0;
    
    % Setup block sample time
    block.SampleTimes = [0.05 0]; % 20 Hz update rate
    
    % Register methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Update', @Update);
    block.RegBlockMethod('Terminate', @Terminate);

function DoPostPropSetup(block)
    % Setup Dwork for storing visualization handles
    block.NumDworks = 1;
    block.Dwork(1).Name = 'FigHandle';
    block.Dwork(1).Dimensions = 1;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';

function InitConditions(block)
    % Create figure for visualization
    figHandle = figure('Name', 'Vehicle Body 3DOF Visualization', ...
                       'NumberTitle', 'off', ...
                       'Position', [100 100 1200 800], ...
                       'Color', 'w');
    
    % Store figure handle
    block.Dwork(1).Data = figHandle.Number;
    
    % Create subplots
    figure(figHandle);
    
    % Main trajectory plot
    subplot(2,2,[1,3]);
    hold on; grid on; axis equal;
    title('Vehicle Trajectory and Position', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Y Distance [m]', 'FontSize', 10);
    ylabel('X Distance [m]', 'FontSize', 10);
    
    % Initialize plot elements with tags for easy updating
    plot(0, 0, 'b-', 'LineWidth', 1.5, 'Tag', 'Trajectory');
    
    % Vehicle body (rectangle)
    rectangle('Position', [0 0 1 1], 'FaceColor', [0.7 0.7 0.9], ...
              'EdgeColor', 'k', 'LineWidth', 2, 'Tag', 'VehicleBody');
    
    % Velocity vector
    quiver(0, 0, 0, 1, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Tag', 'VelocityVector');
    
    % Wheel positions
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'WheelFL');
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'WheelFR');
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'WheelRL');
    plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'Tag', 'WheelRR');
    
    % Parameters display - Velocity
    subplot(2,2,2);
    axis off;
    text(0.1, 0.9, 'Vehicle Parameters', 'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.75, 'Velocity:', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.65, '0.0 m/s', 'FontSize', 10, 'Tag', 'VelText');
    text(0.1, 0.55, 'Yaw Angle:', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.45, '0.0 deg', 'FontSize', 10, 'Tag', 'YawText');
    text(0.1, 0.35, 'Slip Angle (β):', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.25, '0.0 deg', 'FontSize', 10, 'Tag', 'BetaText');
    text(0.1, 0.15, 'Yaw Rate:', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.05, '0.0 rad/s', 'FontSize', 10, 'Tag', 'YawRateText');
    
    % Forces display
    subplot(2,2,4);
    axis off;
    text(0.1, 0.9, 'Forces & Accelerations', 'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.75, 'Long. Accel (ax):', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.65, '0.0 g', 'FontSize', 10, 'Tag', 'AxText');
    text(0.1, 0.55, 'Lat. Accel (ay):', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.45, '0.0 g', 'FontSize', 10, 'Tag', 'AyText');
    text(0.1, 0.35, 'Steering Angle FL:', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.25, '0.0 deg', 'FontSize', 10, 'Tag', 'SteerText');
    text(0.1, 0.15, 'Position (X, Y):', 'FontSize', 11, 'FontWeight', 'bold');
    text(0.1, 0.05, '(0.0, 0.0) m', 'FontSize', 10, 'Tag', 'PosText');

function Update(block)
    try
        % Get figure handle
        figNum = block.Dwork(1).Data;
        if ~ishghandle(figNum)
            return;
        end
        
        figure(figNum);
        
        % Extract data from bus signal
        busData = block.InputPort(1).Data;
        
        % Inertial frame data
        X = busData.InertFrm.Cg.Disp.X;
        Y = busData.InertFrm.Cg.Disp.Y;
        psi = busData.InertFrm.Ang.psi; % Yaw angle
        
        % Body frame data
        Vx = busData.BdyFrm.Cg.Vel.xdot;
        Vy = busData.BdyFrm.Cg.Vel.ydot;
        V = sqrt(Vx^2 + Vy^2); % Total velocity
        
        beta = busData.BdyFrm.Ang.Beta; % Slip angle
        r = busData.BdyFrm.AngVel.r; % Yaw rate
        
        ax = busData.BdyFrm.Acc.ax; % Longitudinal acceleration (in g)
        ay = busData.BdyFrm.Acc.ay; % Lateral acceleration (in g)
        
        % Wheel positions
        xFL = busData.FrntAxl.Lft.Disp.x;
        yFL = busData.FrntAxl.Lft.Disp.y;
        xFR = busData.FrntAxl.Rght.Disp.x;
        yFR = busData.FrntAxl.Rght.Disp.y;
        xRL = busData.RearAxl.Lft.Disp.x;
        yRL = busData.RearAxl.Lft.Disp.y;
        xRR = busData.RearAxl.Rght.Disp.x;
        yRR = busData.RearAxl.Rght.Disp.y;
        
        % Steering angle
        steerFL = busData.FrntAxl.Steer.WhlAngFL;
        
        % Update trajectory plot
        subplot(2,2,[1,3]);
        
        % Update trajectory line
        trajHandle = findobj('Tag', 'Trajectory');
        if ~isempty(trajHandle)
            xData = get(trajHandle, 'XData');
            yData = get(trajHandle, 'YData');
            xData = [xData, Y]; % Note: Y is earth-fixed Y-axis
            yData = [yData, X]; % Note: X is earth-fixed X-axis
            % Keep last 500 points
            if length(xData) > 500
                xData = xData(end-499:end);
                yData = yData(end-499:end);
            end
            set(trajHandle, 'XData', xData, 'YData', yData);
        end
        
        % Vehicle dimensions (approximate)
        L = 4.5; % Vehicle length
        W = 1.8; % Vehicle width
        
        % Update vehicle body rectangle
        bodyHandle = findobj('Tag', 'VehicleBody');
        if ~isempty(bodyHandle)
            % Calculate vehicle corners
            corners = [-L/2, -W/2;
                       L/2, -W/2;
                       L/2, W/2;
                       -L/2, W/2];
            
            % Rotate corners by yaw angle
            R = [cos(psi), -sin(psi);
                 sin(psi), cos(psi)];
            rotated = (R * corners')';
            
            % Translate to vehicle position
            corners_world = rotated + [Y, X];
            
            % Update rectangle
            set(bodyHandle, 'Position', [corners_world(1,1), corners_world(1,2), ...
                sqrt((corners_world(2,1)-corners_world(1,1))^2 + (corners_world(2,2)-corners_world(1,2))^2), ...
                sqrt((corners_world(4,1)-corners_world(1,1))^2 + (corners_world(4,2)-corners_world(1,2))^2)]);
            delete(bodyHandle); % Delete old rectangle
            
            % Draw new polygon
            patch(corners_world(:,1), corners_world(:,2), [0.7 0.7 0.9], ...
                  'EdgeColor', 'k', 'LineWidth', 2, 'Tag', 'VehicleBody');
        end
        
        % Update velocity vector
        velHandle = findobj('Tag', 'VelocityVector');
        if ~isempty(velHandle)
            % Velocity in earth frame
            vel_scale = 3; % Scale factor for visualization
            Vx_earth = V * cos(psi + beta);
            Vy_earth = V * sin(psi + beta);
            set(velHandle, 'XData', Y, 'YData', X, ...
                'UData', Vy_earth * vel_scale, 'VData', Vx_earth * vel_scale);
        end
        
        % Update wheel positions
        wheelFL = findobj('Tag', 'WheelFL');
        wheelFR = findobj('Tag', 'WheelFR');
        wheelRL = findobj('Tag', 'WheelRL');
        wheelRR = findobj('Tag', 'WheelRR');
        
        % Transform wheel positions to earth frame
        R = [cos(psi), -sin(psi);
             sin(psi), cos(psi)];
        
        posFL_earth = R * [xFL; yFL] + [Y; X];
        posFR_earth = R * [xFR; yFR] + [Y; X];
        posRL_earth = R * [xRL; yRL] + [Y; X];
        posRR_earth = R * [xRR; yRR] + [Y; X];
        
        if ~isempty(wheelFL), set(wheelFL, 'XData', posFL_earth(1), 'YData', posFL_earth(2)); end
        if ~isempty(wheelFR), set(wheelFR, 'XData', posFR_earth(1), 'YData', posFR_earth(2)); end
        if ~isempty(wheelRL), set(wheelRL, 'XData', posRL_earth(1), 'YData', posRL_earth(2)); end
        if ~isempty(wheelRR), set(wheelRR, 'XData', posRR_earth(1), 'YData', posRR_earth(2)); end
        
        % Update axis limits to follow vehicle
        xlim([Y-20, Y+20]);
        ylim([X-20, X+20]);
        
        % Update parameter displays
        velText = findobj('Tag', 'VelText');
        yawText = findobj('Tag', 'YawText');
        betaText = findobj('Tag', 'BetaText');
        yawRateText = findobj('Tag', 'YawRateText');
        axText = findobj('Tag', 'AxText');
        ayText = findobj('Tag', 'AyText');
        steerText = findobj('Tag', 'SteerText');
        posText = findobj('Tag', 'PosText');
        
        if ~isempty(velText), set(velText, 'String', sprintf('%.2f m/s (%.2f km/h)', V, V*3.6)); end
        if ~isempty(yawText), set(yawText, 'String', sprintf('%.2f deg', rad2deg(psi))); end
        if ~isempty(betaText), set(betaText, 'String', sprintf('%.2f deg', rad2deg(beta))); end
        if ~isempty(yawRateText), set(yawRateText, 'String', sprintf('%.3f rad/s', r)); end
        if ~isempty(axText), set(axText, 'String', sprintf('%.3f g', ax)); end
        if ~isempty(ayText), set(ayText, 'String', sprintf('%.3f g', ay)); end
        if ~isempty(steerText), set(steerText, 'String', sprintf('%.2f deg', rad2deg(steerFL))); end
        if ~isempty(posText), set(posText, 'String', sprintf('(%.2f, %.2f) m', X, Y)); end
        
        drawnow limitrate;
    catch ME
        warning('Error in vehicle visualization: %s', ME.message);
    end

function Terminate(block)
    % Close figure when simulation ends
    figNum = block.Dwork(1).Data;
    if ishghandle(figNum)
        close(figNum);
    end
