function sector_switch_sfun(block)
% =========================================================================
%  SECTOR SWITCH  –  Level-2 MATLAB S-Function
%
%  Monitors vehicle position, detects entry into risky road segments
%  (from vulnerability_results in workspace), and:
%    1. Sets mu for that segment to  min_safe_mu - MU_CHALLENGE_OFFSET
%    2. Activates the RL agent switch (Out1 goes HIGH)
%    3. Packages the RL state vector
%    4. Deactivates when vehicle exits the segment
%
%  ── INPUTS ────────────────────────────────────────────────────────────────
%   In1  X      – vehicle global X         [m]   ← vehicle Out1
%   In2  Y      – vehicle global Y         [m]   ← vehicle Out2
%   In3  speed  – vehicle speed            [m/s] ← vehicle Out3
%   In4  psi    – vehicle heading          [rad] ← vehicle Out4
%   In5  vy     – lateral velocity         [m/s] ← vehicle Out8
%   In6  r      – yaw rate                 [rad/s]← vehicle Out9
%   In7  cte    – cross-track error        [m]   ← MPC Out4
%   In8  he     – heading error            [rad] ← MPC Out5
%
%  ── OUTPUTS ───────────────────────────────────────────────────────────────
%   Out1  rl_active     – 0/1 switch signal    → rl_mpc_agent In1 (enable)
%   Out2  segment_id    – current segment (0=none, 1..N=segment index)
%   Out3  seg_progress  – 0..1 how far through current segment
%   Out4  mu_challenge  – current challenge mu (= min_safe_mu - offset)
%   Out5  instability   – inferred instability index [0..1]
%   Out6  mu_margin     – mu_current - mu_min_seg  (negative = challenged)
%
%  ── WORKSPACE VARIABLES (read at init) ───────────────────────────────────
%   vulnerability_results – saved by run_vulnerability_pipeline
%     .segments(k).indices    – waypoint indices for segment k
%     .segments(k).min_safe_mu – minimum safe friction found for segment k
%   road_ref – [N×3] path reference
%   mu_road_array – [N×1] current friction profile (written by this block)
%
%  ── SIDE EFFECTS ──────────────────────────────────────────────────────────
%   Writes 'mu_road_array' to workspace whenever segment entry/exit occurs.
%   Writes 'rl_active_segment' struct to workspace for RL agent reference.
% =========================================================================
setup(block);
end


% =========================================================================
function setup(block)
    block.NumInputPorts  = 8;
    block.NumOutputPorts = 6;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    for k = 1:8
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = false;  % reads DWork in Outputs
    end
    for k = 1:6
        block.OutputPort(k).Dimensions = 1;
        block.OutputPort(k).DatatypeID = 0;
        block.OutputPort(k).Complexity = 'Real';
    end

    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0.10 0];   % 10 Hz – matches MPC rate
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% =========================================================================
%  DWORK
%   1  prev_wp_idx    [1]   – waypoint search hint
%   2  rl_active      [1]   – 0 or 1
%   3  segment_id     [1]   – active segment index (0 = none)
%   4  seg_progress   [1]   – [0..1]
%   5  mu_challenge   [1]   – friction being applied
%   6  instability    [1]   – instability index
%   7  mu_margin      [1]   – mu_current - mu_min_seg
% =========================================================================
function PostPropSetup(block)
    block.NumDworks = 7;
    names = {'prev_wp','rl_active','segment_id','seg_progress', ...
             'mu_challenge','instability','mu_margin'};
    for k = 1:7
        block.Dwork(k).Name            = names{k};
        block.Dwork(k).Dimensions      = 1;
        block.Dwork(k).DatatypeID      = 0;
        block.Dwork(k).Complexity      = 'Real';
        block.Dwork(k).UsedAsDiscState = true;
    end
end


% =========================================================================
function InitConditions(block)
    % Align search hint to teleport start position
    start_wp = 1;
    try; start_wp = max(1,round(evalin('base','sim_start_wp'))); catch; end
    block.Dwork(1).Data = double(start_wp);  % prev_wp
    block.Dwork(2).Data = 0;      % rl_active
    block.Dwork(3).Data = 0;      % segment_id
    block.Dwork(4).Data = 0;      % seg_progress
    block.Dwork(5).Data = 0.7;    % mu_challenge (nominal until set)
    block.Dwork(6).Data = 0;      % instability
    block.Dwork(7).Data = 0;      % mu_margin

    % Clear RL override so MPC starts with default params
    ov.active    = false;
    ov.dw_cte    = 0;
    ov.dw_he     = 0;
    ov.dw_ddelta = 0;
    ov.dv_target = 0;
    assignin('base', 'rl_param_override', ov);
    assignin('base', 'rl_active_segment', struct('id',0,'mu_min',0.7,'mu_challenge',0.7));
end


% =========================================================================
%  OUTPUTS  –  returns stored DWork values (no feedthrough)
% =========================================================================
function Outputs(block)
    block.OutputPort(1).Data = block.Dwork(2).Data;   % rl_active
    block.OutputPort(2).Data = block.Dwork(3).Data;   % segment_id
    block.OutputPort(3).Data = block.Dwork(4).Data;   % seg_progress
    block.OutputPort(4).Data = block.Dwork(5).Data;   % mu_challenge
    block.OutputPort(5).Data = block.Dwork(6).Data;   % instability
    block.OutputPort(6).Data = block.Dwork(7).Data;   % mu_margin
end


% =========================================================================
%  UPDATE  –  main logic
% =========================================================================
function Update(block)
    MU_CHALLENGE_OFFSET = 0.02;   % challenge = min_safe_mu - this value
    MU_NOMINAL          = 0.70;   % restore to this when outside segments
    BETA_REF            = deg2rad(15.0);  % was 5° — too sensitive, normal cornering exceeded it
    L_WB                = 2.70;   SR = 15.0;

    % ── Read inputs ──────────────────────────────────────────────────────
    X     = block.InputPort(1).Data;
    Y     = block.InputPort(2).Data;
    speed = block.InputPort(3).Data;
    psi   = block.InputPort(4).Data;
    vy    = block.InputPort(5).Data;
    r     = block.InputPort(6).Data;
    cte   = block.InputPort(7).Data;
    he    = block.InputPort(8).Data;

    % ── Load vulnerability data ───────────────────────────────────────────
    try
        vr       = evalin('base', 'vulnerability_results');
        segs     = vr.segments;
        road_ref = evalin('base', 'road_ref');
        mu_arr   = evalin('base', 'mu_road_array');
    catch
        return;
    end

    N_wp = size(road_ref, 1);
    N_seg = length(segs);

    % ── Find closest waypoint (bounded search) ────────────────────────────
    prev = max(1, min(N_wp, round(block.Dwork(1).Data)));
    ilo  = max(1,   prev - 5);
    ihi  = min(N_wp, prev + 80);
    dx   = road_ref(ilo:ihi,1) - X;
    dy   = road_ref(ilo:ihi,2) - Y;
    [~,li] = min(dx.^2 + dy.^2);
    wp_now = ilo + li - 1;
    block.Dwork(1).Data = double(wp_now);

    % ── Check which segment vehicle is in (0 = none) ──────────────────────
    % During RL training, only activate the specific segment being trained.
    % rl_training_segment_id = 0 means normal mode (activate all segments).
    % rl_training_segment_id = k means only activate segment k.
    training_seg_id = 0;
    try; training_seg_id = round(evalin('base','rl_training_segment_id')); catch; end

    in_seg = 0;
    for s = 1:N_seg
        if ~isfield(segs(s),'min_safe_mu') || isnan(segs(s).min_safe_mu)
            continue;
        end
        % Skip segments that are not the training target
        if training_seg_id > 0 && s ~= training_seg_id
            continue;
        end
        if any(segs(s).indices == wp_now)
            in_seg = s;
            break;
        end
    end

    prev_seg = round(block.Dwork(3).Data);

    % ── Compute instability index (same logic as MPC adapt_speed) ─────────
    vx          = max(speed, 1.0);
    beta        = atan2(vy, vx);
    slip_usage  = min(abs(beta) / BETA_REF, 1.0);
    delta_road  = 0;   % prev delta not available here – use beta only
    instability = slip_usage;   % simplified: oversteer indicator only

    % ── Segment entry / exit logic ────────────────────────────────────────
    if in_seg > 0
        seg      = segs(in_seg);
        mu_min   = seg.min_safe_mu;
        mu_chal  = max(0.05, mu_min - MU_CHALLENGE_OFFSET);

        % Progress through segment [0..1]
        idx_list = seg.indices;
        [~, pos] = min(abs(idx_list - wp_now));
        progress = (pos - 1) / max(length(idx_list) - 1, 1);

        % Real margin: how far actual mu is below the known safe threshold
        % Negative = challenged below safe level, more negative = more dangerous
        try
            actual_arr = evalin('base','mu_road_array');
            actual_mu_now = actual_arr(min(wp_now, length(actual_arr)));
        catch
            actual_mu_now = mu_chal;
        end
        mu_margin = actual_mu_now - mu_min;   % negative when challenged

        if in_seg ~= prev_seg
            % ── ENTRY ────────────────────────────────────────────────────
            % In training mode (training_seg_id > 0): the trainer already
            % set mu_road_array to the curriculum level. Do NOT overwrite.
            % In normal mode (training_seg_id = 0): set challenge mu as usual.
            if training_seg_id == 0
                mu_arr(idx_list) = mu_chal;
                assignin('base', 'mu_road_array', mu_arr);
            end

            % For display: read actual current mu from mu_road_array (shows real value)
            try
                actual_mu_arr = evalin('base','mu_road_array');
                actual_mu = actual_mu_arr(idx_list(1));
            catch
                actual_mu = mu_chal;
            end

            seg_info.id            = in_seg;
            seg_info.mu_min        = mu_min;
            seg_info.mu_challenge  = actual_mu;    % real mu, not hardcoded
            seg_info.indices       = idx_list;
            seg_info.name          = seg.name;
            assignin('base', 'rl_active_segment', seg_info);

            fprintf('[sector_switch] ENTRY segment %d (%s)  mu=%.3f  t=%.1fs\n', ...
                    in_seg, seg.name, actual_mu, block.CurrentTime);
        end

        block.Dwork(2).Data = 1;          % rl_active = ON
        block.Dwork(3).Data = double(in_seg);
        block.Dwork(4).Data = progress;
        % Store actual mu from road array (not the hardcoded challenge offset)
        try
            actual_arr = evalin('base','mu_road_array');
            idx_first  = seg.indices(1);
            block.Dwork(5).Data = actual_arr(idx_first);
        catch
            block.Dwork(5).Data = mu_chal;
        end
        block.Dwork(6).Data = instability;
        block.Dwork(7).Data = mu_margin;

    else
        % ── OUTSIDE all segments ──────────────────────────────────────────
        if prev_seg > 0
            % EXIT: only restore nominal mu in normal mode.
            % In training mode the trainer owns mu_road_array.
            if training_seg_id == 0
                seg = segs(prev_seg);
                mu_arr(seg.indices) = MU_NOMINAL;
                assignin('base', 'mu_road_array', mu_arr);
            end

            % Clear RL override so MPC returns to default params
            ov.active    = false;
            ov.dw_cte    = 0;  ov.dw_he = 0;
            ov.dw_ddelta = 0;  ov.dv_target = 0;
            assignin('base', 'rl_param_override', ov);
            assignin('base', 'rl_active_segment', struct('id',0,'mu_min',0.7,'mu_challenge',0.7));

            fprintf('[sector_switch] EXIT  segment %d  t=%.1fs\n', ...
                    prev_seg, block.CurrentTime);
        end

        block.Dwork(2).Data = 0;          % rl_active = OFF
        block.Dwork(3).Data = 0;
        block.Dwork(4).Data = 0;
        block.Dwork(5).Data = MU_NOMINAL;
        block.Dwork(6).Data = instability;
        block.Dwork(7).Data = 0;
    end
end


% =========================================================================
function Terminate(~)
end