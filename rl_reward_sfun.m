function rl_reward_sfun(block)
% =========================================================================
%  RL_REWARD_SFUN  —  Physics-grounded reward
%
%  WIRING (backward compatible for In1-10, add In11-12 new):
%   In1   rl_active     ← sector_switch Out1
%   In2   instability   ← sector_switch Out5
%   In3   mu_margin     ← sector_switch Out6
%   In4   obs[1]=cte/6  ← Selector(obs,1)        [from obs Out1]
%   In5   obs[2]=he/pi  ← Selector(obs,2)
%   In6   obs[5]=spd/25 ← Selector(obs,5)
%   In7   a1            ← RL Agent Out1[1]         (was In7 before)
%   In8   a2            ← RL Agent Out1[2]
%   In9   a3            ← RL Agent Out1[3]
%   In10  a4            ← RL Agent Out1[4]
%   In11  obs[7]=grip   ← Selector(obs,7)          NEW wire from obs
%   In12  obs[8]=vexc   ← Selector(obs,8)          NEW wire from obs
%   In13  obs[3]=beta/0.349 ← Selector(obs,3)      NEW wire from obs
%
%  Out1  reward  → RL Agent Reward
%  Out2  is_done → RL Agent IsDone
% =========================================================================
setup(block);
end

function setup(block)
    block.NumInputPorts  = 13;
    block.NumOutputPorts = 2;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    for k = 1:13
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = true;
    end
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0;
    block.OutputPort(1).Complexity = 'Real';
    block.OutputPort(2).Dimensions = 1;
    block.OutputPort(2).DatatypeID = 0;
    block.OutputPort(2).Complexity = 'Real';
    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0.10 0];
    block.SimStateCompliance = 'DefaultSimState';
    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Terminate',            @Terminate);
end

function PostPropSetup(block)
    names = {'reward','is_done','prev_cte_n','prev_beta_n','in_seg_prev','done_fired'};
    block.NumDworks = 6;
    for i=1:6
        block.Dwork(i).Name=names{i};
        block.Dwork(i).Dimensions=1;
        block.Dwork(i).DatatypeID=0;
        block.Dwork(i).Complexity='Real';
        block.Dwork(i).UsedAsDiscState=true;
    end
end

function InitConditions(block)
    for i=1:6; block.Dwork(i).Data=0; end
    clear_override();
end

function Outputs(block)
    if block.Dwork(6).Data > 0.5
        block.OutputPort(1).Data = 0;
        block.OutputPort(2).Data = 0;
    else
        block.OutputPort(1).Data = block.Dwork(1).Data;
        block.OutputPort(2).Data = block.Dwork(2).Data;
    end
end

function Update(block)
    rl_active  = block.InputPort(1).Data;
    %In2 instability kept for backward compat but not used directly
    mu_margin  = block.InputPort(3).Data;
    cte_n      = block.InputPort(4).Data;   % cte/6
    he_n       = block.InputPort(5).Data;   % he/pi
    speed_n    = block.InputPort(6).Data;   % speed/25
    a1         = block.InputPort(7).Data;
    a2         = block.InputPort(8).Data;
    a3         = block.InputPort(9).Data;
    a4         = block.InputPort(10).Data;
    grip_usage = block.InputPort(11).Data;  % a_lat/(mu*g) from obs[7]
    v_excess_n = block.InputPort(12).Data;  % (v-v_safe)/8 from obs[8]
    beta_n     = block.InputPort(13).Data;  % beta/0.349 from obs[3]

    % Denormalise
    cte_m    = abs(cte_n) * 6.0;           % [m]
    beta_r   = abs(beta_n) * 0.349;        % [rad]
    v_excess = v_excess_n * 8.0;           % [m/s]
    he_d     = abs(he_n) * 180;            % approx degrees

    prev_cte_n  = block.Dwork(3).Data;
    prev_beta_n = block.Dwork(4).Data;
    in_seg_prev = block.Dwork(5).Data;
    dcte_n      = cte_n  - prev_cte_n;
    dbeta_n     = abs(beta_n) - abs(prev_beta_n);

    % Crash thresholds (physics-based)
    CTE_CRASH  = 1.5;    % [m] = half lane width
    BETA_CRASH = 0.349;  % [rad] = 20 deg

    crash_cte   = cte_m  > CTE_CRASH;
    crash_beta  = beta_r > BETA_CRASH;
    crash_speed = (v_excess > 8.0) && (abs(v_excess_n) > 0.8);

    is_crash = crash_cte || crash_beta || crash_speed;

    % Apply MPC override inside segment
    if rl_active > 0.5
        ov.active    = true;
        ov.dw_cte    = a1 * 12.5 + 2.5;
        ov.dw_he     = a2 *  8.0 + 2.0;
        ov.dw_ddelta = a3 *  6.0 + 2.0;
        ov.dv_target = a4 *  4.0 - 1.0;   % biased: default slows down
        assignin('base','rl_param_override', ov);
    else
        clear_override();
    end

    seg_completed = (in_seg_prev > 0.5) && (rl_active < 0.5) && ~is_crash;
    is_done = double(is_crash || seg_completed);

    % ── REWARD ─────────────────────────────────────────────────────────────
    if rl_active < 0.5
        reward = 0;  % CRITICAL FIX: no free reward outside segment

    elseif is_crash
        if crash_beta
            reward = -100.0 * (beta_r  / BETA_CRASH);
        elseif crash_speed
            reward = -80.0  * min(1, v_excess / 8.0);
        else
            reward = -60.0  * (cte_m / CTE_CRASH);
        end

    else
        % Tire saturation — most important signal
        if grip_usage > 1.0
            R_grip = -8.0 * (grip_usage - 1.0);
        elseif grip_usage > 0.80
            R_grip = -2.0 * (grip_usage - 0.80);
        else
            R_grip = +3.0 * (0.80 - grip_usage);
        end

        % CTE tracking + trend
        LANE_W = 3.5;
        R_cte = -4.0*(cte_m/LANE_W) - 3.0*max(0, dcte_n);

        % Slip angle (oversteer)
        beta_thr = 0.087;   % 5 deg
        if beta_r > beta_thr
            R_beta = -5.0*((beta_r-beta_thr)/(BETA_CRASH-beta_thr));
        else
            R_beta = +1.0;
        end
        R_beta = R_beta - 3.0*max(0, dbeta_n);

        % Speed vs physics limit
        if v_excess > 0
            R_speed = -4.0 * min(1, v_excess / 5.0);
        else
            R_speed = +0.5 * max(0, 1.0 - abs(v_excess)/5.0);
        end

        % Heading + action regularisation
        R_he     = -2.0 * (he_d / 30.0);
        act_mag  = sqrt(a1^2 + a2^2 + a3^2 + a4^2);
        R_action = -0.3 * max(0, act_mag - 1.0);

        % Survival bonus (per step reward for staying on road)
        R_alive  = +2.0;

        reward = R_grip + R_cte + R_beta + R_speed + R_he + R_action + R_alive;
    end

    block.Dwork(1).Data = double(reward);
    block.Dwork(2).Data = double(is_done);
    block.Dwork(3).Data = cte_n;
    block.Dwork(4).Data = beta_n;
    block.Dwork(5).Data = rl_active;
    if is_done > 0.5
        block.Dwork(6).Data = 1;
    end
end

function Terminate(~)
    clear_override();
end

function clear_override()
    ov.active=false; ov.dw_cte=0; ov.dw_he=0; ov.dw_ddelta=0; ov.dv_target=0;
    try; assignin('base','rl_param_override',ov); catch; end
end