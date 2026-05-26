function mu_road_profile_sfun(block)
% =========================================================================
%  MU ROAD PROFILE  –  Level-2 MATLAB S-Function
%
%  Outputs a spatially-varying friction coefficient based on the vehicle's
%  current position.  Replaces the constant mu block in the Simulink model.
%
%  ALGEBRAIC LOOP FIX:
%   DirectFeedthrough = false on both inputs.
%   Outputs() returns the mu stored from the PREVIOUS Update() step.
%   Update() reads X,Y and computes the new mu for next step.
%   This introduces one sample (0.02 s) delay – irrelevant for friction.
%
%  INPUTS
%   In1  X  – vehicle global X [m]  ← vehicle Out1
%   In2  Y  – vehicle global Y [m]  ← vehicle Out2
%
%  OUTPUTS
%   Out1 mu – friction for current road position [-]
%             → vehicle In4  (replaces constant mu block)
%
%  WORKSPACE VARIABLES (set by run_vulnerability_pipeline before each sim)
%   road_ref       [N×3]  built by build_road_ref()
%   mu_road_array  [N×1]  friction value per waypoint
% =========================================================================
setup(block);
end


function setup(block)
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    for k = 1:2
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        % FALSE – Outputs() reads only DWork (stored state), not InputPorts.
        % This breaks the algebraic loop with vehicle_dynamics_sfun.
        block.InputPort(k).DirectFeedthrough = false;
    end

    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0;
    block.OutputPort(1).Complexity = 'Real';

    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0.02 0];     % 50 Hz
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% =========================================================================
%  DWORK
%   1  prev_idx  [1]  – closest waypoint index from previous Update()
%   2  prev_mu   [1]  – mu value from previous Update(), returned by Outputs()
% =========================================================================
function PostPropSetup(block)
    block.NumDworks = 2;

    block.Dwork(1).Name            = 'prev_idx';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name            = 'prev_mu';
    block.Dwork(2).Dimensions      = 1;
    block.Dwork(2).DatatypeID      = 0;
    block.Dwork(2).Complexity      = 'Real';
    block.Dwork(2).UsedAsDiscState = true;
end


function InitConditions(block)
    start_wp = 1;
    try; start_wp = max(1,round(evalin('base','sim_start_wp'))); catch; end
    block.Dwork(1).Data = double(start_wp);  % prev_idx — teleport-aware
    block.Dwork(2).Data = 0.7;   % default nominal friction at start
end


% =========================================================================
%  OUTPUTS  –  returns mu stored by previous Update(), NO feedthrough
% =========================================================================
function Outputs(block)
    block.OutputPort(1).Data = block.Dwork(2).Data;
end


% =========================================================================
%  UPDATE  –  reads X,Y, finds closest waypoint, stores mu for next Outputs()
% =========================================================================
function Update(block)
    X = block.InputPort(1).Data;
    Y = block.InputPort(2).Data;

    try
        road_ref      = evalin('base', 'road_ref');
        mu_road_array = evalin('base', 'mu_road_array');
    catch
        % Workspace vars not found – keep previous mu
        return;
    end

    N    = size(road_ref, 1);
    prev = max(1, min(N, round(block.Dwork(1).Data)));

    % Bounded search using previous index as hint
    ilo = max(1,   prev - 5);
    ihi = min(N,   prev + 80);
    dx  = road_ref(ilo:ihi, 1) - X;
    dy  = road_ref(ilo:ihi, 2) - Y;
    [~, li] = min(dx.^2 + dy.^2);
    idx = ilo + li - 1;

    % Store for next Outputs() call
    mu_val = mu_road_array(min(idx, length(mu_road_array)));
    block.Dwork(1).Data = double(idx);
    block.Dwork(2).Data = max(0.05, min(1.5, mu_val));
end


function Terminate(~)
end