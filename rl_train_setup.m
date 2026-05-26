% =========================================================================
%  RL_SEGMENT_TRAINER.m
%  Curriculum-based episodic training for RL MPC parameter tuner.
%
%  ALGORITHM
%   For each segment from vulnerability_results:
%     Start at mu = min_safe_mu  (known safe from pipeline)
%     Each curriculum level drops mu by MU_STEP = 0.02
%     At each level run N_EPISODES_PER_LEVEL episodes
%     Episode = teleport vehicle 50wp before segment → run → success/crash
%     success_rate >= 0.60 → go deeper (record best params)
%     success_rate < 0.60  → consecutive_failures++
%     consecutive_failures == 3 → stop, report min achievable mu
%
%  RUN
%   >> sim_setup
%   >> rl_segment_trainer
% =========================================================================
clc;
fprintf('=====================================================\n');
fprintf('  RL SEGMENT TRAINER  - Curriculum Learning\n');
fprintf('=====================================================\n\n');

%% CONFIG
MODEL_NAME           = 'MAIN_VEHICLE_MODEL';
MU_STEP              = 0.02;
N_EPISODES_PER_LEVEL = 10;
SUCCESS_THRESHOLD    = 0.60;
MAX_CONSEC_FAIL      = 3;
WP_APPROACH          = 50;
SAC_UPDATES_PER_EP   = 30;
MIN_BUFFER_FOR_SAC   = 128;
SAVE_DIR             = 'rl_training_results';

%% PREREQUISITES
required = {'road_ref','mu_road_array','vulnerability_results'};
for k = 1:length(required)
    if ~evalin('base',sprintf('exist(''%s'',''var'')',required{k}))
        error('%s missing. Run sim_setup first.',required{k});
    end
end

road_ref = evalin('base','road_ref');
vr       = evalin('base','vulnerability_results');
segs     = vr.segments;
N_wp     = size(road_ref,1);

valid_segs = [];
for s = 1:length(segs)
    if isfield(segs(s),'min_safe_mu') && ~isnan(segs(s).min_safe_mu) ...
       && segs(s).min_safe_mu > 0.05
        valid_segs(end+1) = s; %#ok<AGROW>
    end
end
fprintf('Found %d valid segments.\n\n', length(valid_segs));

if ~exist(SAVE_DIR,'dir'); mkdir(SAVE_DIR); end
if ~bdIsLoaded(MODEL_NAME); load_system(MODEL_NAME); end
warning('off','Simulink:Engine:AlgLoopWithStaticAnalysis');
warning('off','Simulink:Engine:AlgebraicLoopSolver');
assignin('base','rl_agent_mode','heuristic');

training_results.timestamp = datestr(now);
training_results.segments  = struct('name',{},'original_mu',{},'best_mu',{}, ...
                                    'best_params',{},'level_history',{});

%% MAIN LOOP
for seg_num = 1:length(valid_segs)
    s_idx = valid_segs(seg_num);
    seg   = segs(s_idx);

    fprintf('============================================\n');
    fprintf('SEGMENT %d/%d: %s  wp%d--%d\n', ...
            seg_num,length(valid_segs),seg.name,seg.start,seg.end);
    fprintf('  Original min_safe_mu: %.3f\n',seg.min_safe_mu);
    fprintf('============================================\n\n');

    seg_len_m  = length(seg.indices);
    T_episode  = ceil((seg_len_m + WP_APPROACH) / (60/3.6)) + 10;
    start_wp   = max(1, seg.start - WP_APPROACH);
    seg_end_wp = seg.end;

    mu_test        = seg.min_safe_mu;
    consec_fail    = 0;
    level          = 0;
    best_mu        = seg.min_safe_mu;
    best_params    = zeros(1,4);
    level_history  = struct('level',{},'mu_test',{},'successes',{},...
                            'success_rate',{},'mean_reward',{});

    while consec_fail < MAX_CONSEC_FAIL

        level   = level + 1;
        mu_test = seg.min_safe_mu - level * MU_STEP;
        mu_test = max(0.05, mu_test);

        fprintf('  Level %d  mu=%.3f\n', level, mu_test);

        ci.mu_min            = seg.min_safe_mu;
        ci.mu_test           = mu_test;
        ci.consecutive_failures = consec_fail;
        assignin('base','rl_curriculum_info', ci);

        mu_arr              = 0.7 * ones(N_wp,1);
        mu_arr(seg.indices) = mu_test;
        assignin('base','mu_road_array', mu_arr);

        successes      = 0;
        ep_rewards     = zeros(N_EPISODES_PER_LEVEL,1);
        ep_params_list = cell(N_EPISODES_PER_LEVEL,1);

        for ep = 1:N_EPISODES_PER_LEVEL
            assignin('base','sim_crashed',    false);
            assignin('base','sim_crash_time', 0);
            assignin('base','sim_crash_wp',   0);
            assignin('base','sim_start_wp',   start_wp);
            clear_rl_override();

            t0 = tic;
            sim_ok = true;
            try
                sim(MODEL_NAME, T_episode);
            catch ME
                sim_ok = false;
                fprintf('    Ep%2d sim error: %s\n',ep,ME.message);
            end
            wall = toc(t0);

            crashed      = evalin('base','sim_crashed');
            crash_wp     = evalin('base','sim_crash_wp');
            seg_done     = false;
            ep_rew       = 0;
            ep_par       = zeros(1,4);

            if sim_ok && evalin('base','exist(''sim_log'',''var'')')
                lg          = evalin('base','sim_log');
                max_wp      = get_max_wp(lg, road_ref);
                seg_done    = (~crashed) && (max_wp >= seg_end_wp);
                ep_rew      = get_recent_mean_reward();
                try
                    ov = evalin('base','rl_param_override');
                    ep_par = [ov.dw_cte, ov.dw_he, ov.dw_ddelta, ov.dv_target];
                catch; end
            end

            if seg_done
                successes    = successes + 1;
                R_term       = 100 + 50*level;
                tag          = sprintf('OK  R+%d', R_term);
            else
                R_term       = -100;
                tag          = sprintf('FAIL@wp%d', crash_wp);
            end

            add_terminal_reward(R_term);
            ep_rewards(ep)     = ep_rew;
            ep_params_list{ep} = ep_par;

            fprintf('    Ep%2d/%d  %s  rew=%+6.1f  wall=%3.0fs\n', ...
                    ep, N_EPISODES_PER_LEVEL, tag, ep_rew, wall);

            % SAC update after each episode
            if get_buf_count() >= MIN_BUFFER_FOR_SAC
                do_sac_update(SAC_UPDATES_PER_EP);
                if get_buf_count() >= MIN_BUFFER_FOR_SAC * 4
                    assignin('base','rl_agent_mode','trained');
                end
            end
        end

        sr = successes / N_EPISODES_PER_LEVEL;
        fprintf('  Level %d: %d/%d (%.0f%%)  mean_rew=%.1f\n\n', ...
                level, successes, N_EPISODES_PER_LEVEL, sr*100, mean(ep_rewards));

        lh.level        = level;
        lh.mu_test      = mu_test;
        lh.successes    = successes;
        lh.success_rate = sr;
        lh.mean_reward  = mean(ep_rewards);
        if isempty(fieldnames(level_history))
            level_history = lh;
        else
            level_history(end+1) = lh; %#ok<AGROW>
        end

        if sr >= SUCCESS_THRESHOLD
            consec_fail = 0;
            best_mu     = mu_test;
            [~,bi]      = max(ep_rewards);
            best_params = ep_params_list{bi};
            fprintf('  PASSED -> next level mu=%.3f\n\n', mu_test-MU_STEP);
        else
            consec_fail = consec_fail + 1;
            fprintf('  FAILED (%d/%d consecutive)\n\n', consec_fail, MAX_CONSEC_FAIL);
        end

        if mu_test <= 0.05; break; end
    end

    % Report
    fprintf('--- SEGMENT %s DONE ---\n', seg.name);
    fprintf('  Original : %.3f\n', seg.min_safe_mu);
    fprintf('  RL best  : %.3f  (improved by %.3f)\n', ...
            best_mu, seg.min_safe_mu - best_mu);
    if ~isempty(best_params) && any(best_params ~= 0)
        fprintf('  Best MPC params: dw_cte=%+.2f  dw_he=%+.2f  dw_ddelta=%+.2f  dv=%+.2f\n\n', ...
                best_params(1), best_params(2), best_params(3), best_params(4));
    end

    entry.name        = seg.name;
    entry.original_mu = seg.min_safe_mu;
    entry.best_mu     = best_mu;
    entry.best_params = best_params;
    entry.level_history = level_history;
    if isempty(fieldnames(training_results.segments))
        training_results.segments = entry;
    else
        training_results.segments(end+1) = entry;
    end

    save(fullfile(SAVE_DIR,'training_results.mat'),'training_results');
    fprintf('Checkpoint saved.\n\n');
end

%% FINAL SUMMARY
fprintf('=====================================================\n');
fprintf('  TRAINING COMPLETE\n');
fprintf('=====================================================\n');
fprintf('%-20s  %8s  %8s  %10s\n','Segment','Orig mu','Best mu','Improvement');
fprintf('%s\n',repmat('-',52,1));
for k = 1:length(training_results.segments)
    sr = training_results.segments(k);
    if ~isfield(sr,'name') || isempty(sr.name); continue; end
    d = sr.original_mu - sr.best_mu;
    fprintf('%-20s  %8.3f  %8.3f  %+.3f (%.0f%%)\n', ...
            sr.name, sr.original_mu, sr.best_mu, d, d/sr.original_mu*100);
end

try
    an = evalin('base','rl_actor_net');
    save(fullfile(SAVE_DIR,'rl_actor_net.mat'),'an');
    fprintf('\nActor network saved.\n');
catch; end

assignin('base','training_results',training_results);
fprintf('Results in workspace as ''training_results''\n\n');


%% =========================================================================
%  HELPERS
%% =========================================================================

function clear_rl_override()
    ov.active=false;ov.dw_cte=0;ov.dw_he=0;ov.dw_ddelta=0;ov.dv_target=0;
    assignin('base','rl_param_override',ov);
end

function max_wp = get_max_wp(lg, road_ref)
    N = size(road_ref,1); max_wp = 1;
    for k = 1:10:length(lg.X)
        ilo=max(1,max_wp-5); ihi=min(N,max_wp+80);
        dx=road_ref(ilo:ihi,1)-lg.X(k); dy=road_ref(ilo:ihi,2)-lg.Y(k);
        [~,li]=min(dx.^2+dy.^2); wp=ilo+li-1;
        if wp>max_wp; max_wp=wp; end
    end
end

function rew = get_recent_mean_reward()
    try
        buf=evalin('base','rl_experience_buffer');
        n=min(100,buf.count); if n<1; rew=0; return; end
        idxs=mod((buf.head-n:buf.head-1),max(buf.count,1))+1;
        rew=mean(buf.rewards(idxs));
    catch; rew=0; end
end

function add_terminal_reward(R)
    try
        buf=evalin('base','rl_experience_buffer');
        if buf.count<1; return; end
        buf.rewards(buf.head)=buf.rewards(buf.head)+R;
        assignin('base','rl_experience_buffer',buf);
    catch; end
end

function n = get_buf_count()
    try; buf=evalin('base','rl_experience_buffer'); n=buf.count; catch; n=0; end
end

function do_sac_update(n_upd)
    try
        buf=evalin('base','rl_experience_buffer');
        if buf.count < 128; return; end
        try
            net=evalin('base','rl_actor_net');
        catch
            layers=[featureInputLayer(18,'Normalization','none','Name','in')
                    fullyConnectedLayer(256,'Name','fc1')
                    reluLayer('Name','r1')
                    fullyConnectedLayer(256,'Name','fc2')
                    reluLayer('Name','r2')
                    fullyConnectedLayer(4,'Name','out')
                    tanhLayer('Name','tanh')];
            net=dlnetwork(layerGraph(layers));
            assignin('base','rl_actor_net',net);
            fprintf('    [RL] Actor network built (18->256->256->4)\n');
        end
        lr=3e-4; bs=min(128,buf.count);
        for it=1:n_upd
            idx=randperm(buf.count,bs);
            S=dlarray(buf.states(idx,:)','CB');
            A=dlarray(buf.actions(idx,:)','CB');
            rw=buf.rewards(idx);
            rw=(rw-mean(rw))/(std(rw)+1e-8);
            R=dlarray(rw','CB');
            [gr,~]=dlfeval(@pg_step,net,S,A,R);
            net=dlupdate(@(p,g)p-lr*g,net,gr);
        end
        assignin('base','rl_actor_net',net);
    catch ME
        if ~(contains(ME.message,'dlnetwork')||contains(ME.message,'dlarray'))
            fprintf('    [RL] Update note: %s\n',ME.message);
        end
    end
end

function [gr,loss]=pg_step(net,S,A,R)
    Ap=forward(net,S);
    lp=-0.5*sum((A-Ap).^2,1);
    loss=-mean(lp.*R);
    gr=dlgradient(loss,net.Learnables);
end