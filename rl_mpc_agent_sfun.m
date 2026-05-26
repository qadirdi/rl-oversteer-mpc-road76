function rl_mpc_agent_sfun(block)
% =========================================================================
%  RL MPC AGENT  –  Level-2 MATLAB S-Function  (curriculum version)
%
%  18-dimensional state space.
%  Terminal reward on segment completion or crash.
%  Curriculum-aware: knows how deep into mu reduction it is.
%
%  INPUTS  (11)
%   In1  rl_active     – 0/1 switch         ← sector_switch Out1
%   In2  segment_id    – current segment     ← sector_switch Out2
%   In3  seg_progress  – [0..1]              ← sector_switch Out3
%   In4  mu_challenge  – current mu          ← sector_switch Out4
%   In5  instability   – [0..1]              ← sector_switch Out5
%   In6  mu_margin     – mu_cur - mu_min     ← sector_switch Out6
%   In7  cte           – cross-track error   ← MPC Out4
%   In8  he            – heading error       ← MPC Out5
%   In9  speed         – vehicle speed       ← vehicle Out3
%   In10 vy            – lateral velocity    ← vehicle Out8
%   In11 r             – yaw rate            ← vehicle Out9
%
%  OUTPUTS  (6)
%   Out1  dw_cte       – Δw_cte    → workspace override
%   Out2  dw_he        – Δw_he
%   Out3  dw_ddelta    – Δw_ddelta
%   Out4  dv_target    – Δv_target [m/s]
%   Out5  reward       – step reward (monitor/training)
%   Out6  rl_enabled   – mirrors rl_active
%
%  STATE VECTOR (18 elements, all normalised to [-1,1] or [0,1])
%   1   cte_norm         cross-track error / 6m
%   2   he_norm          heading error / pi
%   3   beta_norm        body side-slip / 0.26 rad (15 deg)
%   4   r_norm           yaw rate / 1.5 rad/s
%   5   speed_norm       speed / 25 m/s
%   6   instability      grip loss index [0,1]
%   7   mu_chal_norm     challenge friction / 0.65
%   8   mu_min_norm      min safe friction / 0.65
%   9   mu_margin_norm   (mu_cur - mu_min) / 0.30
%  10   seg_progress     fraction through segment [0,1]
%  11   dw_cte_norm      current w_cte delta / 15
%  12   dw_he_norm       current w_he delta / 10
%  13   dw_ddelta_norm   current w_ddelta delta / 8
%  14   dv_target_norm   current v_target delta / 5
%  15   instab_trend     instability(t) - instability(t-1)
%  16   curriculum_depth (min_safe_mu - mu_test) / 0.20
%  17   consec_failures  consecutive_failures / 3.0
%  18   dist_to_seg_end  distance to segment end / seg_length
%
%  ACTION VECTOR (4 elements, all in [-1,1])
%   1   a_wcte      → Δw_cte    in [-10, +15]
%   2   a_whe       → Δw_he     in [ -6, +10]
%   3   a_wddelta   → Δw_ddelta in [ -4,  +8]
%   4   a_vtarget   → Δv_target in [ -5,  +3] m/s
% =========================================================================
setup(block);
end


function setup(block)
    block.NumInputPorts  = 11;
    block.NumOutputPorts = 6;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    for k = 1:11
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = false;
    end
    for k = 1:6
        block.OutputPort(k).Dimensions = 1;
        block.OutputPort(k).DatatypeID = 0;
        block.OutputPort(k).Complexity = 'Real';
    end

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
    block.NumDworks = 7;
    names = {'dw_cte','dw_he','dw_ddelta','dv_target','reward','rl_enabled','state_prev'};
    dims  = [1, 1, 1, 1, 1, 1, 18];
    for k = 1:7
        block.Dwork(k).Name            = names{k};
        block.Dwork(k).Dimensions      = dims(k);
        block.Dwork(k).DatatypeID      = 0;
        block.Dwork(k).Complexity      = 'Real';
        block.Dwork(k).UsedAsDiscState = true;
    end
end


function InitConditions(block)
    for k = 1:6; block.Dwork(k).Data = 0; end
    block.Dwork(7).Data = zeros(18, 1);
end


function Outputs(block)
    block.OutputPort(1).Data = block.Dwork(1).Data;
    block.OutputPort(2).Data = block.Dwork(2).Data;
    block.OutputPort(3).Data = block.Dwork(3).Data;
    block.OutputPort(4).Data = block.Dwork(4).Data;
    block.OutputPort(5).Data = block.Dwork(5).Data;
    block.OutputPort(6).Data = block.Dwork(6).Data;
end


function Update(block)
    rl_active = block.InputPort(1).Data;

    if rl_active < 0.5
        clear_override();
        for k = 1:6; block.Dwork(k).Data = 0; end
        return;
    end

    % Read inputs
    seg_id    = block.InputPort(2).Data;
    seg_prog  = block.InputPort(3).Data;
    mu_chal   = block.InputPort(4).Data;
    instab    = block.InputPort(5).Data;
    mu_margin = block.InputPort(6).Data;
    cte       = block.InputPort(7).Data;
    he        = block.InputPort(8).Data;
    speed     = block.InputPort(9).Data;
    vy        = block.InputPort(10).Data;
    r         = block.InputPort(11).Data;

    % Current param deltas
    dw_cte    = block.Dwork(1).Data;
    dw_he     = block.Dwork(2).Data;
    dw_ddelta = block.Dwork(3).Data;
    dv_target = block.Dwork(4).Data;

    mu_min_seg = mu_chal + 0.02;
    vx = max(speed, 1.0);
    beta = atan2(vy, vx);

    % Read curriculum info from workspace
    curriculum_depth  = 0;
    consec_failures   = 0;
    dist_to_end_norm  = 1 - seg_prog;
    try
        ci = evalin('base','rl_curriculum_info');
        curriculum_depth = clamp((ci.mu_min - ci.mu_test) / 0.20, 0, 1);
        consec_failures  = clamp(ci.consecutive_failures / 3.0, 0, 1);
    catch
    end

    % ── BUILD 18-DIM STATE ─────────────────────────────────────────────
    s = zeros(18,1);
    s(1)  = clamp(cte  / 6.0,        -1, 1);
    s(2)  = clamp(he   / pi,          -1, 1);
    s(3)  = clamp(beta / 0.26,        -1, 1);
    s(4)  = clamp(r    / 1.5,         -1, 1);
    s(5)  = clamp(speed / 25.0,        0, 1);
    s(6)  = clamp(instab,              0, 1);
    s(7)  = clamp((mu_chal  - 0.05) / 0.65, 0, 1);
    s(8)  = clamp((mu_min_seg - 0.05) / 0.65, 0, 1);
    s(9)  = clamp(mu_margin / 0.30,   -1, 1);
    s(10) = clamp(seg_prog,            0, 1);
    s(11) = clamp(dw_cte    / 15.0,   -1, 1);
    s(12) = clamp(dw_he     / 10.0,   -1, 1);
    s(13) = clamp(dw_ddelta /  8.0,   -1, 1);
    s(14) = clamp(dv_target /  5.0,   -1, 1);
    s(15) = clamp(instab - block.Dwork(7).Data(6), -1, 1);  % trend
    s(16) = clamp(curriculum_depth,    0, 1);
    s(17) = clamp(consec_failures,     0, 1);
    s(18) = clamp(dist_to_end_norm,    0, 1);
    block.Dwork(7).Data = s;

    % ── SELECT ACTION ──────────────────────────────────────────────────
    mode = get_agent_mode();
    if strcmp(mode, 'trained')
        action = apply_trained_policy(s);
    else
        action = heuristic_policy(s, instab, mu_margin, curriculum_depth);
    end

    % Map [-1,1] → physical deltas
    new_dw_cte    =  action(1) * 12.5 + 2.5;
    new_dw_he     =  action(2) *  8.0 + 2.0;
    new_dw_ddelta =  action(3) *  6.0 + 2.0;
    new_dv_target =  action(4) * (-4.0) + (-1.0);

    % Apply override to workspace
    ov.active    = true;
    ov.dw_cte    = new_dw_cte;
    ov.dw_he     = new_dw_he;
    ov.dw_ddelta = new_dw_ddelta;
    ov.dv_target = new_dv_target;
    assignin('base', 'rl_param_override', ov);

    % ── STEP REWARD ────────────────────────────────────────────────────
    % Ensure action and state are plain double before reward / logging
    action_d = double(action(:));
    s_d      = double(s(:));
    rew = compute_step_reward(s_d, action_d, instab, mu_margin, curriculum_depth);
    rew = double(rew);

    % Log experience
    log_experience(s_d, action_d, rew);

    block.Dwork(1).Data = double(new_dw_cte);
    block.Dwork(2).Data = double(new_dw_he);
    block.Dwork(3).Data = double(new_dw_ddelta);
    block.Dwork(4).Data = double(new_dv_target);
    block.Dwork(5).Data = double(rew);
    block.Dwork(6).Data = 1;
end


function Terminate(block)
    clear_override();
end


% =========================================================================
%  STEP REWARD
%  Called every 0.1s while inside a segment.
%  Terminal reward (+100/-100) is added by rl_segment_trainer
%  AFTER the episode ends, not here.
% =========================================================================
function rew = compute_step_reward(s, action, instab, mu_margin, curr_depth)

    % ── Safety ───────────────────────────────────────────────────────────
    if instab > 0.95
        R_safety = -50.0;                % near-crash
    elseif instab < 0.15
        R_safety = +3.0;                 % fully stable
    else
        R_safety = +1.0 * (1 - instab);  % gradual
    end

    % ── Tracking ─────────────────────────────────────────────────────────
    R_tracking = -2.0 * abs(s(1))   ...   % cte_norm
                 -1.5 * abs(s(2));         % he_norm

    % ── Speed efficiency ─────────────────────────────────────────────────
    R_speed = +0.5 * s(5);

    % ── Action smoothness ────────────────────────────────────────────────
    action_norm = norm(action);
    R_action = -0.3 * action_norm;

    % Extra penalty for large actions when mu is safe (unnecessary tuning)
    if mu_margin >= 0
        R_action = R_action - 0.5 * action_norm;
    end

    % ── Curriculum depth bonus ────────────────────────────────────────────
    % Small positive reward per step for surviving at a lower mu
    % Scales with how deep into curriculum we are: 0 at mu_min, +0.5 at max depth
    R_curriculum = +0.5 * curr_depth * (1 - instab);

    rew = R_safety + R_tracking + R_speed + R_action + R_curriculum;
end


% =========================================================================
%  HEURISTIC POLICY
%  Rule-based fallback. Scales aggressiveness with both instability and
%  curriculum depth (deeper = more aggressive tuning allowed).
% =========================================================================
function action = heuristic_policy(s, instab, mu_margin, curr_depth)
    urgency = instab + max(0, -mu_margin / 0.05) + curr_depth * 0.3;
    urgency = min(urgency, 1.0);

    a_wcte    =  urgency * 0.7;
    a_whe     =  urgency * 0.5;
    a_wddelta =  urgency * 0.6;
    a_vtarget = -urgency * 0.8;

    if instab < 0.15 && mu_margin >= 0
        a_wcte    = -0.10;
        a_whe     = -0.10;
        a_wddelta = -0.05;
        a_vtarget =  0.20;
    end

    action = clamp_vec([a_wcte; a_whe; a_wddelta; a_vtarget], -1, 1);
end


function action = apply_trained_policy(state)
    % state is 18x1 double.
    % dlnetwork predict expects [features x batch] dlarray.
    % Output must be converted back to plain double before assigning to DWork.
    try
        net = evalin('base', 'rl_actor_net');
        % Input: [18 x 1] dlarray with 'CB' (channel x batch) format
        x   = dlarray(double(state(:)), 'CB');
        y   = predict(net, x);
        % Convert dlarray → plain double vector
        action = double(extractdata(y));
        action = clamp_vec(action(:), -1, 1);
    catch
        action = zeros(4,1);
    end
end


% =========================================================================
%  EXPERIENCE LOGGER
% =========================================================================
function log_experience(state, action, reward)
    MAX_N   = 50000;
    N_STATE = 18;     % must match state vector length
    N_ACT   = 4;

    % Load existing buffer or create fresh one
    buf_ok = false;
    try
        buf = evalin('base','rl_experience_buffer');
        % Validate dimensions – reset if state width changed (e.g. 15→18)
        if size(buf.states,2) == N_STATE && size(buf.actions,2) == N_ACT
            buf_ok = true;
        else
            % Old buffer with wrong size – discard silently and start fresh
        end
    catch
    end

    if ~buf_ok
        buf.states  = zeros(MAX_N, N_STATE);
        buf.actions = zeros(MAX_N, N_ACT);
        buf.rewards = zeros(MAX_N, 1);
        buf.head    = 0;
        buf.count   = 0;
    end

    idx                = mod(buf.head, MAX_N) + 1;
    buf.states(idx,:)  = state(:)';       % force row vector
    buf.actions(idx,:) = action(:)';
    buf.rewards(idx)   = reward;
    buf.head           = idx;
    buf.count          = min(buf.count + 1, MAX_N);
    assignin('base','rl_experience_buffer', buf);
end


% =========================================================================
%  HELPERS
% =========================================================================
function clear_override()
    ov.active=false; ov.dw_cte=0; ov.dw_he=0; ov.dw_ddelta=0; ov.dv_target=0;
    assignin('base','rl_param_override',ov);
end

function mode = get_agent_mode()
    try; mode = evalin('base','rl_agent_mode'); catch; mode='heuristic'; end
end

function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end

function y = clamp_vec(x, lo, hi)
    y = max(lo, min(hi, x));
end