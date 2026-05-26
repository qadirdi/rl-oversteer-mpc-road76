% =========================================================================
%  RL_FULLROAD_TRAINER.m   —   Curriculum Learning for MPC Stability
%
%  RESEARCH BASIS:
%   Taherian et al. (2021): RL-based parameter tuning for vehicle stability
%   control under low friction — same architecture, DDPG→SAC upgrade.
%   MathWorks Curriculum LKA example: train() in batches, ResetFcn controls
%   difficulty, EpisodeInfo.CumulativeReward evaluates success per level.
%
%  DESIGN:
%   • Full road (wp1 to end) every episode — vehicle sees the whole track
%   • sector_switch dynamically drops mu on segment entry, restores on exit
%   • One segment trained completely (all mu levels) before moving to next
%   • Segments extended ±SEG_EXTEND waypoints for more challenge exposure
%   • Uses RL Toolbox train() — sac_agent updated correctly every level
%   • Training Monitor shows live reward curve per level
%
%  PREREQUISITES:
%   >> sim_setup
%   >> rl_toolbox_sac_setup     (creates sac_agent, env, sac_opts)
%   >> rl_fullroad_trainer
% =========================================================================
clc;
fprintf('=====================================================\n');
fprintf('  RL FULL-ROAD CURRICULUM TRAINER\n');
fprintf('  Based on MathWorks Curriculum LKA pattern\n');
fprintf('=====================================================\n\n');

%% CONFIG
MODEL_NAME             = 'MAIN_VEHICLE_MODEL';
T_EPISODE              = 60;       % [s]  covers all 5 segments (last exits ~50s)
MU_STEP                = 0.02;     % mu drop per level
N_EPISODES_PER_LEVEL   = 8;        % episodes per train() call
SUCCESS_THRESHOLD      = 0.60;     % fraction of episodes that must succeed
REWARD_SUCCESS_THR     = 30;       % episode reward above this = success
MAX_CONSEC_FAIL        = 3;
SEG_EXTEND             = 30;       % ±30 waypoints around each segment
MAX_STEPS_PER_EP       = round(T_EPISODE / 0.10);  % 900 steps at 0.1s
SAVE_DIR               = 'rl_fullroad_results';

%% PREREQUISITES
fprintf('Checking prerequisites...\n');
required = {'road_ref','vulnerability_results','sac_agent','env'};
for k = 1:length(required)
    if ~evalin('base',sprintf('exist(''%s'',''var'')',required{k}))
        error('%s not found. Run sim_setup then rl_toolbox_sac_setup.',required{k});
    end
end

road_ref = evalin('base','road_ref');
vr       = evalin('base','vulnerability_results');
segs     = vr.segments;
N_wp     = size(road_ref,1);
sac_agent= evalin('base','sac_agent');
env      = evalin('base','env');

valid_segs = [];
for s = 1:length(segs)
    if isfield(segs(s),'min_safe_mu') && ~isnan(segs(s).min_safe_mu) ...
       && segs(s).min_safe_mu > MU_STEP + 0.05
        valid_segs(end+1) = s; %#ok<AGROW>
    end
end
fprintf('  %d segments to train\n\n', length(valid_segs));

if ~exist(SAVE_DIR,'dir'); mkdir(SAVE_DIR); end
if ~bdIsLoaded(MODEL_NAME); load_system(MODEL_NAME); end

warning('off','Simulink:Engine:AlgLoopWithStaticAnalysis');
warning('off','Simulink:Engine:AlgebraicLoopSolver');
set_param(MODEL_NAME,'StopTime',num2str(T_EPISODE));

% Disable visualiser during training (prevents 600s freeze)
vis_block = find_system(MODEL_NAME,'MFunctionName','vehicle_visualizer_2d');
if isempty(vis_block)
    vis_block = find_system(MODEL_NAME,'FunctionName','vehicle_visualizer_2d');
end
vis_was = 'off';
if ~isempty(vis_block)
    vis_was = get_param(vis_block{1},'Commented');
    set_param(vis_block{1},'Commented','on');
    fprintf('Visualiser disabled (re-enabled after training)\n\n');
end

%% BUILD TRAINING OPTIONS FOR ONE CURRICULUM LEVEL
% Key insight from MathWorks curriculum example:
% - MaxEpisodes = N per level, StopTrainingCriteria = 'none'
%   → train() runs exactly N episodes and returns
% - Plots = 'training-progress' → shows RL Training Monitor window
% - Between train() calls: update workspace vars for next difficulty level
level_opts = rlTrainingOptions(...
    'MaxEpisodes',              N_EPISODES_PER_LEVEL, ...
    'MaxStepsPerEpisode',       MAX_STEPS_PER_EP, ...
    'ScoreAveragingWindowLength', N_EPISODES_PER_LEVEL, ...
    'StopTrainingCriteria',     'none', ...
    'Verbose',                  true, ...
    'Plots',                    'none');

%% RESULTS
all_results = [];   % [] init lets MATLAB assign first struct without type conflict

%% MAIN CURRICULUM LOOP
for sn = 1:length(valid_segs)
    s_idx = valid_segs(sn);
    seg   = segs(s_idx);

    % Extended segment (more exposure time in challenge zone)
    ext_start   = max(1,    seg.start - SEG_EXTEND);
    ext_end     = min(N_wp, seg.end   + SEG_EXTEND);
    ext_indices = (ext_start:ext_end)';
    ext_m       = length(ext_indices);

    fprintf('============================================\n');
    fprintf('SEGMENT %d/%d: %s\n', sn, length(valid_segs), seg.name);
    fprintf('  Original:  wp%d-%d  (%dm)\n', seg.start, seg.end, seg.end-seg.start);
    fprintf('  Extended:  wp%d-%d  (%dm)  [±%dwp = %.1f sec at 50kph]\n', ...
            ext_start, ext_end, ext_m, SEG_EXTEND, ext_m/(50/3.6));
    fprintf('  min_safe_mu: %.3f\n', seg.min_safe_mu);
    fprintf('============================================\n\n');

    % Update vulnerability_results with extended boundaries for sector_switch
    vr_ext = vr;
    vr_ext.segments(s_idx).indices = ext_indices;
    vr_ext.segments(s_idx).start   = ext_start;
    vr_ext.segments(s_idx).end     = ext_end;
    assignin('base','vulnerability_results', vr_ext);
    assignin('base','rl_training_segment_id', s_idx);

    % Curriculum state
    mu_test     = seg.min_safe_mu;
    consec_fail = 0;
    level       = 0;
    best_mu     = seg.min_safe_mu;
    level_history = struct('level',{},'mu',{},'successes',{},...
                           'rate',{},'rewards',{});

    %% CURRICULUM LEVELS
    while consec_fail < MAX_CONSEC_FAIL

        level   = level + 1;
        mu_test = seg.min_safe_mu - level * MU_STEP;
        mu_test = max(0.05, mu_test);

        fprintf('  --- Level %d | mu=%.3f (%.0f%% below safe) ---\n', ...
                level, mu_test, (seg.min_safe_mu-mu_test)/seg.min_safe_mu*100);

        % Store curriculum state for ResetFcn
        assignin('base','rl_curriculum_mu_test',   mu_test);
        assignin('base','rl_curriculum_ext_idx',   ext_indices);
        assignin('base','rl_curriculum_ext_start', ext_start);
        assignin('base','rl_curriculum_ext_end',   ext_end);
        assignin('base','rl_curriculum_seg_idx',   s_idx);
        ci.mu_min             = seg.min_safe_mu;
        ci.mu_test            = mu_test;
        ci.consecutive_failures = consec_fail;
        assignin('base','rl_curriculum_info', ci);

        % Point env to the full-road reset function
        env.ResetFcn = @fullroad_reset_fcn;

        %% RUN N_EPISODES_PER_LEVEL via train()
        % Correct API (MathWorks doc): trainStats = train(agent, env, opts)
        % agent is modified IN-PLACE, trainStats.EpisodeInfo.CumulativeReward
        % contains per-episode rewards
        fprintf('  Running %d full-road episodes...\n', N_EPISODES_PER_LEVEL);
        rewards = zeros(N_EPISODES_PER_LEVEL,1);
        try
            trainStats = train(sac_agent, env, level_opts);
            assignin('base','sac_agent', sac_agent);

            % ── Robust reward extraction ─────────────────────────────────
            % Try every known field name/path across MATLAB versions
            r = [];

            % Path 1: EpisodeInfo.CumulativeReward (newer toolbox)
            if isfield(trainStats,'EpisodeInfo')
                ei = trainStats.EpisodeInfo;
                if isstruct(ei) && isfield(ei,'CumulativeReward')
                    r = double(ei.CumulativeReward(:));
                end
            end

            % Path 2: Direct EpisodeReward vector
            if isempty(r) && isfield(trainStats,'EpisodeReward')
                v = trainStats.EpisodeReward;
                if isnumeric(v) && numel(v) > 1
                    r = double(v(:));
                elseif isstruct(v) && isfield(v,'Value')
                    r = double(v.Value(:));
                end
            end

            % Path 3: Scan all numeric arrays of correct length
            if isempty(r)
                fn = fieldnames(trainStats);
                for fi = 1:length(fn)
                    val = trainStats.(fn{fi});
                    if isnumeric(val) && numel(val) == N_EPISODES_PER_LEVEL ...
                       && ~strcmpi(fn{fi},'EpisodeIndex') ...
                       && ~strcmpi(fn{fi},'TotalAgentSteps')
                        r = double(val(:));
                        fprintf('  [reward extracted from field: %s]\n',fn{fi});
                        break;
                    end
                end
            end

            % Path 4: EpisodeIndex exists → rewards ARE in trainStats
            % but stored differently; print struct for debug
            if isempty(r)
                fprintf('  [DEBUG] trainStats fields:\n');
                fn = fieldnames(trainStats);
                for fi = 1:length(fn)
                    val = trainStats.(fn{fi});
                    fprintf('    .%s  size=%s\n', fn{fi}, mat2str(size(val)));
                end
            end

            if ~isempty(r)
                n = min(length(r), N_EPISODES_PER_LEVEL);
                rewards(1:n) = r(1:n);
            end

        catch ME
            fprintf('  [ERROR] train(): %s\n', ME.message);
            % Print full cause chain so we can diagnose
            cause = ME;
            depth = 0;
            while ~isempty(cause.cause) && depth < 5
                cause = cause.cause{1};
                depth = depth + 1;
                fprintf('    caused by: %s\n', cause.message);
            end
            if ~isempty(ME.stack)
                fprintf('    at: %s line %d\n', ME.stack(1).name, ME.stack(1).line);
            end
        end

        %% EVALUATE LEVEL SUCCESS
        successes = sum(rewards > REWARD_SUCCESS_THR);
        rate      = successes / N_EPISODES_PER_LEVEL;

        fprintf('\n  Level %d:  %d/%d success (%.0f%%)  rewards: min=%.0f  mean=%.0f  max=%.0f\n\n',...
                level, successes, N_EPISODES_PER_LEVEL, rate*100, ...
                min(rewards), mean(rewards), max(rewards));

        lh.level    = level;
        lh.mu       = mu_test;
        lh.successes = successes;
        lh.rate     = rate;
        lh.rewards  = rewards;
        if isempty(fieldnames(level_history))
            level_history = lh;
        else
            level_history(end+1) = lh; %#ok<AGROW>
        end

        if rate >= SUCCESS_THRESHOLD
            consec_fail = 0;
            best_mu     = mu_test;
            fprintf('  PASSED → advancing to mu=%.3f\n\n', mu_test-MU_STEP);
        else
            consec_fail = consec_fail + 1;
            fprintf('  FAILED (%d/%d consecutive failures)\n\n', ...
                    consec_fail, MAX_CONSEC_FAIL);
        end

        if mu_test <= 0.07; fprintf('  Reached lower bound.\n\n'); break; end

    end  % curriculum

    %% SEGMENT REPORT
    fprintf('--- %s CURRICULUM DONE ---\n', seg.name);
    fprintf('  min_safe_mu (pipeline): %.3f\n', seg.min_safe_mu);
    fprintf('  min achievable mu (RL): %.3f  (improved %.3f)\n', ...
            best_mu, seg.min_safe_mu-best_mu);
    fprintf('\n');

    res.name        = seg.name;
    res.original_mu = seg.min_safe_mu;
    res.best_mu     = best_mu;
    res.level_history = level_history;
    res.ext_m       = ext_m;
    if isempty(all_results)
        all_results = res;
    else
        try
            all_results(end+1) = res;
        catch
            % fieldnames mismatch — convert to cell and back
            tmp = num2cell(all_results);
            tmp{end+1} = res;
            all_results = [tmp{:}];
        end
    end

    % Save after each segment (crash-safe)
    save(fullfile(SAVE_DIR,'fullroad_results.mat'),'all_results','sac_agent');

    % Restore original segment info before moving to next
    assignin('base','vulnerability_results', vr);

end  % segment loop

%% RESTORE MODEL STATE
assignin('base','rl_training_segment_id', 0);
assignin('base','vulnerability_results', vr);
assignin('base','mu_road_array', 0.7*ones(N_wp,1));
set_param(MODEL_NAME,'StopTime','60');
if ~isempty(vis_block)
    set_param(vis_block{1},'Commented', vis_was);
    fprintf('Visualiser restored.\n');
end

%% FINAL SUMMARY
fprintf('\n=====================================================\n');
fprintf('  TRAINING COMPLETE\n');
fprintf('=====================================================\n\n');
fprintf('%-20s  %-10s  %-10s  %-10s  %s\n', ...
        'Segment','Orig mu','Best mu','Improve','Ext(m)');
fprintf('%s\n',repmat('-',60,1));
for k = 1:length(all_results)
    r = all_results(k);
    if ~isfield(r,'name')||isempty(r.name); continue; end
    fprintf('%-20s  %-10.3f  %-10.3f  %-10.3f  %dm\n', ...
            r.name, r.original_mu, r.best_mu, ...
            r.original_mu-r.best_mu, r.ext_m);
end

% Save trained agent
if ~exist('rl_toolbox_agents','dir'); mkdir('rl_toolbox_agents'); end
save(fullfile('rl_toolbox_agents','sac_fullroad.mat'),'sac_agent');
assignin('base','fullroad_results', all_results);
fprintf('\nAgent saved: rl_toolbox_agents/sac_fullroad.mat\n');
fprintf('Results in workspace as ''fullroad_results''\n\n');


% =========================================================================
%  RESET FUNCTION — called by train() before EVERY episode
%  Sets: full road start (wp1), challenge mu for current level
% =========================================================================
function in = fullroad_reset_fcn(in)
    assignin('base','sim_start_wp',   1);       % full road always
    assignin('base','sim_crashed',    false);
    assignin('base','sim_crash_time', 0);
    assignin('base','sim_crash_wp',   0);

    % Clear MPC override so each episode starts clean
    ov.active=false; ov.dw_cte=0; ov.dw_he=0; ov.dw_ddelta=0; ov.dv_target=0;
    assignin('base','rl_param_override', ov);

    try
        mu_test     = evalin('base','rl_curriculum_mu_test');
        ext_indices = evalin('base','rl_curriculum_ext_idx');
        N_wp        = size(evalin('base','road_ref'), 1);
        mu_arr      = 0.70 * ones(N_wp, 1);
        mu_arr(ext_indices) = mu_test;
        assignin('base','mu_road_array', mu_arr);

        ext_s = evalin('base','rl_curriculum_ext_start');
        ext_e = evalin('base','rl_curriculum_ext_end');
        s_idx = evalin('base','rl_curriculum_seg_idx');
        fprintf('[reset] Full road | seg%d wp%d-%d | mu_challenge=%.3f\n', ...
                s_idx, ext_s, ext_e, mu_test);
    catch ME
        fprintf('[reset] Warning: %s\n', ME.message);
    end
end