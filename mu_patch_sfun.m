function mu_patch_sfun(block)
% =========================================================================
%  MU_PATCH_SFUN  –  Level-2 MATLAB S-Function
%
%  Generates the road friction signal mu(t) for a localised low-friction
%  patch.  Replaces the constant "mu" input going into vehicle_dynamics_sfun
%  In4.  Parameters are read from the MATLAB base workspace so the master
%  sweep script can change them between sim() calls without rebuilding the
%  model.
%
%  ── WORKSPACE VARIABLES READ AT INIT ─────────────────────────────────────
%    mu_patch_s0         onset arc-length [m]
%    mu_patch_mu_low     minimum friction in patch  (0.2 – 0.7)
%    mu_patch_L          patch length [m]
%    mu_patch_mu_nominal baseline friction outside patch  (default 0.7)
%    mu_patch_trans_len  sigmoid transition length [m]    (default 2.0)
%    road_ref            [N×3] centreline from build_road_ref
%    road_arc            [N×1] arc-length at each waypoint
%
%  ── INPUT ─────────────────────────────────────────────────────────────────
%   In1  X       vehicle global X  [m]   ← vehicle Out1
%   In2  Y       vehicle global Y  [m]   ← vehicle Out2
%
%  ── OUTPUTS ───────────────────────────────────────────────────────────────
%   Out1  mu          current friction coefficient  → vehicle In4
%   Out2  s_vehicle   current arc-length position  [m]  (monitoring)
%
%  ── DIALOG PARAMETERS ─────────────────────────────────────────────────────
%   (none – all parameters come from workspace variables listed above)
% =========================================================================
setup(block);
end


function setup(block)
    block.NumInputPorts  = 2;   % X, Y
    block.NumOutputPorts = 2;   % mu, s_vehicle
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    for k = 1:2
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = true;
    end
    for k = 1:2
        block.OutputPort(k).Dimensions = 1;
        block.OutputPort(k).DatatypeID = 0;
        block.OutputPort(k).Complexity = 'Real';
    end

    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0 1];   % continuous inherited

    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% ── DWork ──────────────────────────────────────────────────────────────────
%   1  last_idx   [1]   previous closest waypoint index (search hint)
%   2–6 patch_params [5] [s0, mu_low, L_patch, mu_nominal, trans_len]
function PostPropSetup(block)
    block.NumDworks = 2;

    block.Dwork(1).Name            = 'last_idx';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name            = 'patch_params';
    block.Dwork(2).Dimensions      = 5;
    block.Dwork(2).DatatypeID      = 0;
    block.Dwork(2).Complexity      = 'Real';
    block.Dwork(2).UsedAsDiscState = true;
end


function InitConditions(block)
    % Read patch parameters from workspace at simulation start
    s0        = ws_get('mu_patch_s0',        100.0);
    mu_low    = ws_get('mu_patch_mu_low',     0.4);
    L_patch   = ws_get('mu_patch_L',          20.0);
    mu_nom    = ws_get('mu_patch_mu_nominal', 0.7);
    trans_len = ws_get('mu_patch_trans_len',  2.0);

    block.Dwork(1).Data = 1;                            % last_idx
    block.Dwork(2).Data = [s0, mu_low, L_patch, mu_nom, trans_len];
end


function Outputs(block)
    X = block.InputPort(1).Data;
    Y = block.InputPort(2).Data;

    pp = block.Dwork(2).Data;
    s0        = pp(1);
    mu_low    = pp(2);
    L_patch   = pp(3);
    mu_nom    = pp(4);
    trans_len = pp(5);

    % Find arc-length position of vehicle
    last_idx  = max(1, round(block.Dwork(1).Data));
    s_vehicle = vehicle_arc_length(X, Y, last_idx);

    % Compute mu using sigmoid patch profile
    mu_out = sigmoid_patch(s_vehicle, s0, mu_nom, mu_low, L_patch, trans_len);

    block.OutputPort(1).Data = mu_out;
    block.OutputPort(2).Data = s_vehicle;
end


function Terminate(~)
end


% =========================================================================
%  COMPUTE VEHICLE ARC-LENGTH POSITION
% =========================================================================
function s = vehicle_arc_length(X, Y, last_idx)
    persistent rr arc_s;

    if isempty(rr)
        try
            rr    = evalin('base', 'road_ref');
            arc_s = evalin('base', 'road_arc');
        catch
            s = 0;
            return;
        end
    end

    if isempty(rr) || isempty(arc_s)
        s = 0;  return;
    end

    N   = size(rr,1);
    ilo = max(1,   last_idx - 3);
    ihi = min(N,   last_idx + 80);

    dx = rr(ilo:ihi,1) - X;
    dy = rr(ilo:ihi,2) - Y;
    [~, li] = min(dx.^2 + dy.^2);
    closest = ilo + li - 1;

    s = arc_s(closest);
end


% =========================================================================
%  SIGMOID PATCH PROFILE
% =========================================================================
function mu = sigmoid_patch(s, s0, mu_nom, mu_low, L_patch, trans_len)
    trans_len = min(trans_len, L_patch/2 - 0.05);
    trans_len = max(trans_len, 0.1);

    k     = 2 * log(19) / trans_len;      % 95% transition in trans_len m
    delta = mu_nom - mu_low;
    rise  = 1 / (1 + exp(-k * (s - s0)));
    fall  = 1 - 1 / (1 + exp(-k * (s - (s0 + L_patch))));
    mu    = mu_nom - delta * rise * fall;
    mu    = max(0.05, min(1.5, mu));
end


% =========================================================================
%  SAFE WORKSPACE READ
% =========================================================================
function v = ws_get(name, default_val)
    try
        v = evalin('base', name);
    catch
        v = default_val;
        fprintf('[mu_patch_sfun] Warning: "%s" not found, using %.3f\n', ...
                name, default_val);
    end
end