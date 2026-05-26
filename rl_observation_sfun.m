function rl_observation_sfun(block)
% =========================================================================
%  RL_OBSERVATION_SFUN  —  Physics-informed 16-dim observation
%
%  WIRING (backward compatible with original — same In1-11, add In12-15):
%   In1   rl_active      ← sector_switch Out1
%   In2   seg_id         ← sector_switch Out2   (unused, kept for compat.)
%   In3   seg_progress   ← sector_switch Out3
%   In4   mu_challenge   ← sector_switch Out4
%   In5   instability    ← sector_switch Out5
%   In6   mu_margin      ← sector_switch Out6
%   In7   cte [m]        ← MPC Out4
%   In8   he  [rad]      ← MPC Out5
%   In9   speed [m/s]    ← vehicle Out3
%   In10  vy  [m/s]      ← vehicle Out8   (used to compute beta)
%   In11  r   [rad/s]    ← vehicle Out9
%   In12  a1             ← RL Agent Out1[1]   NEW — wire from RL Demux
%   In13  a2             ← RL Agent Out1[2]   NEW
%   In14  a3             ← RL Agent Out1[3]   NEW
%   In15  a4             ← RL Agent Out1[4]   NEW
%
%  Out1  obs [16×1]  → RL Agent Observation
%  Out2  rl_active   → Terminator
%
%  Obs vector:
%   [1]  cte/6           [2]  he/pi           [3]  beta/0.349 (20deg)
%   [4]  r/1.5           [5]  speed/25        [6]  mu_challenge/0.60
%   [7]  grip_usage      [8]  v_excess/8      [9]  mu_margin/0.30
%   [10] seg_progress    [11] beta_rate       [12] cte_rate
%   [13] a1              [14] a2              [15] a3             [16] a4
% =========================================================================
setup(block);
end

function setup(block)
    block.NumInputPorts  = 15;
    block.NumOutputPorts = 2;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    for k = 1:15
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = false;
    end
    block.OutputPort(1).Dimensions  = 16;
    block.OutputPort(1).DatatypeID  = 0;
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(2).Dimensions  = 1;
    block.OutputPort(2).DatatypeID  = 0;
    block.OutputPort(2).Complexity  = 'Real';
    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0.10 0];
    block.SimStateCompliance = 'DefaultSimState';
    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
end

function PostPropSetup(block)
    block.NumDworks = 4;
    block.Dwork(1).Name='obs';       block.Dwork(1).Dimensions=16;
    block.Dwork(2).Name='rl_act';    block.Dwork(2).Dimensions=1;
    block.Dwork(3).Name='prev_beta'; block.Dwork(3).Dimensions=1;
    block.Dwork(4).Name='prev_cte';  block.Dwork(4).Dimensions=1;
    for i=1:4
        block.Dwork(i).DatatypeID=0;
        block.Dwork(i).Complexity='Real';
        block.Dwork(i).UsedAsDiscState=true;
    end
end

function InitConditions(block)
    block.Dwork(1).Data = zeros(16,1);
    block.Dwork(2).Data = 0;
    block.Dwork(3).Data = 0;
    block.Dwork(4).Data = 0;
end

function Outputs(block)
    block.OutputPort(1).Data = block.Dwork(1).Data;
    block.OutputPort(2).Data = block.Dwork(2).Data;
end

function Update(block)
    rl_active  = block.InputPort(1).Data;
    %In2 seg_id unused
    seg_prog   = block.InputPort(3).Data;
    mu_now     = block.InputPort(4).Data;
    %In5 instability unused (replaced by physics signals)
    mu_margin  = block.InputPort(6).Data;
    cte        = block.InputPort(7).Data;   % [m]
    he         = block.InputPort(8).Data;   % [rad]
    speed      = block.InputPort(9).Data;   % [m/s]
    vy         = block.InputPort(10).Data;  % [m/s]
    r          = block.InputPort(11).Data;  % [rad/s]
    a1         = block.InputPort(12).Data;
    a2         = block.InputPort(13).Data;
    a3         = block.InputPort(14).Data;
    a4         = block.InputPort(15).Data;

    g  = 9.81;
    dt = 0.10;

    % Body slip angle from lateral velocity
    vx   = sqrt(max(0, speed^2 - vy^2));
    beta = atan2(vy, max(0.5, vx));   % [rad]

    % Lateral acceleration demand vs grip limit
    a_lat = speed * abs(r);
    a_lim = max(0.03, mu_now) * g;
    grip_usage = a_lat / a_lim;

    % Physics safe speed
    if abs(r) > 0.02 && speed > 2.0
        R_est  = speed / abs(r);
        v_safe = sqrt(a_lim * R_est);
    else
        v_safe = 99;
    end
    v_excess = (speed - v_safe) / 8.0;

    % Rates
    prev_beta = block.Dwork(3).Data;
    prev_cte  = block.Dwork(4).Data;
    beta_rate = (abs(beta) - abs(prev_beta)) / (dt * 0.5);
    cte_rate  = (abs(cte)  - abs(prev_cte))  / (dt * 2.0);

    obs = zeros(16,1);
    obs(1)  = clamp(cte       / 6.0,    -1, 1);
    obs(2)  = clamp(he        / pi,     -1, 1);
    obs(3)  = clamp(beta      / 0.349,  -1, 1);  % 0.349 rad = 20 deg
    obs(4)  = clamp(r         / 1.5,    -1, 1);
    obs(5)  = clamp(speed     / 25.0,    0, 1);
    obs(6)  = clamp(mu_now    / 0.60,    0, 1);
    obs(7)  = clamp(grip_usage,          0, 1);
    obs(8)  = clamp(v_excess,           -1, 1);
    obs(9)  = clamp(mu_margin / 0.30,   -1, 1);
    obs(10) = clamp(seg_prog,            0, 1);
    obs(11) = clamp(beta_rate,          -1, 1);
    obs(12) = clamp(cte_rate,           -1, 1);
    obs(13) = clamp(a1,                 -1, 1);
    obs(14) = clamp(a2,                 -1, 1);
    obs(15) = clamp(a3,                 -1, 1);
    obs(16) = clamp(a4,                 -1, 1);

    block.Dwork(1).Data = obs;
    block.Dwork(2).Data = rl_active;
    block.Dwork(3).Data = beta;
    block.Dwork(4).Data = cte;
end

function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end