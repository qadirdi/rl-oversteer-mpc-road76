function data_logger_sfun(block)
% =========================================================================
%  DATA LOGGER  –  Level-2 MATLAB S-Function
%
%  Records 10 signals during each simulation run and saves them as
%  struct "sim_log" in the base workspace when the simulation ends.
%  run_vulnerability_pipeline.m reads sim_log after each sim() call.
%
%  ── INPUTS  (connect exactly in this order) ──────────────────────────────
%   In  1   X        – vehicle Out1  – global X           [m]
%   In  2   Y        – vehicle Out2  – global Y           [m]
%   In  3   speed    – vehicle Out3  – total speed |v|    [m/s]
%   In  4   psi      – vehicle Out4  – heading            [rad]
%   In  5   vy       – vehicle Out8  – lateral velocity   [m/s]
%   In  6   r        – vehicle Out9  – yaw rate           [rad/s]
%   In  7   delta_sw – MPC     Out1  – steering wheel     [rad]
%   In  8   cte      – MPC     Out4  – cross-track error  [m]
%   In  9   he       – MPC     Out5  – heading error      [rad]
%   In 10   mu       – mu_road_profile Out1 – friction    [-]
%
%  ── NO OUTPUTS (pure sink) ────────────────────────────────────────────────
%
%  ── sim_log STRUCT  (saved to workspace on simulation end) ───────────────
%    .t          [n×1]  time vector                     [s]
%    .X          [n×1]  global X                        [m]
%    .Y          [n×1]  global Y                        [m]
%    .speed      [n×1]  total speed                     [m/s]
%    .psi        [n×1]  heading                         [rad]
%    .vy         [n×1]  lateral velocity                [m/s]
%    .r          [n×1]  yaw rate                        [rad/s]
%    .delta_sw   [n×1]  steering wheel angle            [rad]
%    .cte        [n×1]  cross-track error               [m]
%    .he         [n×1]  heading error                   [rad]
%    .mu         [n×1]  spatial friction                [-]
%    .beta       [n×1]  body side-slip  atan2(vy,vx)   [rad]  ← computed
%    .alpha_f    [n×1]  front slip angle (bicycle approx)[rad] ← computed
%    .alpha_r    [n×1]  rear  slip angle (bicycle approx)[rad] ← computed
% =========================================================================
setup(block);
end


% =========================================================================
function setup(block)
    block.NumInputPorts  = 10;
    block.NumOutputPorts = 0;
    block.SetPreCompInpPortInfoToDynamic;

    for k = 1:10
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = true;
    end

    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0.10 0];     % 10 Hz – matches MPC sample rate
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% =========================================================================
%  DWORK LAYOUT
%   DWork  1         : n_samples counter    [1]
%   DWork  2..11     : one buffer per input [MAX_N]   (channels 1–10)
%   DWork  12        : time buffer          [MAX_N]
% =========================================================================
function PostPropSetup(block)
    MAX_N = 6000;   % 300 s at 20 Hz – more than enough for any run
    block.NumDworks = 12;

    % Counter
    block.Dwork(1).Name            = 'n';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    % 10 signal channels (DWork 2–11)
    ch_names = {'X','Y','speed','psi','vy','r','delta_sw','cte','he','mu'};
    for k = 2:11
        block.Dwork(k).Name            = ch_names{k-1};
        block.Dwork(k).Dimensions      = MAX_N;
        block.Dwork(k).DatatypeID      = 0;
        block.Dwork(k).Complexity      = 'Real';
        block.Dwork(k).UsedAsDiscState = true;
    end

    % Time channel (DWork 12)
    block.Dwork(12).Name            = 'time';
    block.Dwork(12).Dimensions      = MAX_N;
    block.Dwork(12).DatatypeID      = 0;
    block.Dwork(12).Complexity      = 'Real';
    block.Dwork(12).UsedAsDiscState = true;
end


% =========================================================================
function InitConditions(block)
    for k = 1:12
        block.Dwork(k).Data = zeros(block.Dwork(k).Dimensions, 1);
    end
end


% =========================================================================
function Update(block)
    MAX_N = 6000;
    n = block.Dwork(1).Data;
    if n >= MAX_N;  return;  end   % buffer full – stop silently

    n = n + 1;
    block.Dwork(1).Data = n;

    % Store 10 input channels
    for k = 1:10
        block.Dwork(k+1).Data(n) = block.InputPort(k).Data;
    end
    % Store time
    block.Dwork(12).Data(n) = block.CurrentTime;
end


% =========================================================================
%  TERMINATE – build sim_log and save to workspace
% =========================================================================
function Terminate(block)
    n = round(block.Dwork(1).Data);
    if n < 5
        warning('[data_logger] Only %d samples logged. Check block is connected.', n);
        return;
    end

    % ── Assemble raw fields ───────────────────────────────────────────────
    s.t        = block.Dwork(12).Data(1:n);
    s.X        = block.Dwork(2).Data(1:n);
    s.Y        = block.Dwork(3).Data(1:n);
    s.speed    = block.Dwork(4).Data(1:n);
    s.psi      = block.Dwork(5).Data(1:n);
    s.vy       = block.Dwork(6).Data(1:n);
    s.r        = block.Dwork(7).Data(1:n);
    s.delta_sw = block.Dwork(8).Data(1:n);
    s.cte      = block.Dwork(9).Data(1:n);
    s.he       = block.Dwork(10).Data(1:n);
    s.mu       = block.Dwork(11).Data(1:n);

    % ── Derived fields (vehicle geometry constants) ───────────────────────
    a_wb  = 1.08;    % [m] CG to front axle
    b_wb  = 1.62;    % [m] CG to rear axle
    SR    = 15.0;    % steering ratio
    vx    = max(abs(s.speed), 0.5);   % approx longitudinal speed

    % Body side-slip angle
    s.beta    = atan2(s.vy, vx);

    % Front / rear tire slip angles (kinematic bicycle approximation)
    delta_road = s.delta_sw / SR;
    s.alpha_f  = -atan((s.vy + a_wb .* s.r) ./ vx) + delta_road;
    s.alpha_r  = -atan((s.vy - b_wb .* s.r) ./ vx);

    % ── Save to workspace ─────────────────────────────────────────────────
    assignin('base', 'sim_log', s);
    fprintf('[data_logger] sim_log saved: %d samples,  t = %.1f – %.1f s\n', ...
            n, s.t(1), s.t(end));
end