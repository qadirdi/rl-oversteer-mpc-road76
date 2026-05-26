% =========================================================================
%  RL_TOOLBOX_SAC_TRAINING.m
%  Runs training for the SAC agent using MATLAB RL Toolbox.
%
%  Run rl_toolbox_sac_setup first to create agent and environment.
%
%  >> sim_setup
%  >> rl_toolbox_sac_setup
%  >> rl_toolbox_sac_training
% =========================================================================

fprintf('=== SAC TRAINING ===\n\n');

% Check prerequisites
if ~exist('sac_agent','var') || ~exist('sac_env','var')
    error('Run rl_toolbox_sac_setup first.');
end

% ── Disable road_crash_detector during RL training ───────────────────────
% crash_detector calls set_param(stop) which fights the RL Toolbox episode
% runner. During training, is_done in rl_reward_sfun handles crash detection.
crash_block = find_system(MODEL_NAME,'MFunctionName','road_crash_detector_sfun');
if isempty(crash_block)
    crash_block = find_system(MODEL_NAME,'FunctionName','road_crash_detector_sfun');
end
crash_was_commented = 'off';
if ~isempty(crash_block)
    crash_was_commented = get_param(crash_block{1},'Commented');
    set_param(crash_block{1},'Commented','on');
    fprintf('road_crash_detector DISABLED for RL training.\n');
end

% Disable visualiser during training (prevents graphics freezes)
MODEL_NAME = 'MAIN_VEHICLE_MODEL';
vis_block = find_system(MODEL_NAME,'MFunctionName','vehicle_visualizer_2d');
if isempty(vis_block)
    vis_block = find_system(MODEL_NAME,'FunctionName','vehicle_visualizer_2d');
end
vis_commented = 'on';
if ~isempty(vis_block)
    vis_commented = get_param(vis_block{1},'Commented');
    set_param(vis_block{1},'Commented','on');
    fprintf('Visualiser disabled for training.\n');
end

warning('off','Simulink:Engine:AlgLoopWithStaticAnalysis');
warning('off','Simulink:Engine:AlgebraicLoopSolver');

% ── Delete incompatible slprj folder if it exists ────────────────────────
slprj_path = fullfile(userpath, '..', 'MATLAB Drive', 'slprj');
if ~exist(slprj_path,'dir')
    slprj_path = '/MATLAB Drive/slprj';   % MATLAB Online path
end
if exist(slprj_path,'dir')
    rmdir(slprj_path,'s');
    fprintf('Deleted old slprj folder (incompatible release).\n');
end

% ── Suppress algebraic loop warning (RL Agent block has inherent loop) ───
warning('off','Simulink:Engine:AlgLoopWithStaticAnalysis');
warning('off','Simulink:Engine:AlgebraicLoopSolver');

% ── Suppress algebraic loop at model level ───────────────────────────────
warning('off','Simulink:Engine:AlgLoopWithStaticAnalysis');
warning('off','Simulink:Engine:AlgebraicLoopSolver');
set_param(MODEL_NAME, 'AlgebraicLoopSolver', 'LineSearch');
set_param(MODEL_NAME, 'AlgebraicLoopMsg', 'none');
fprintf('Algebraic loop warnings suppressed.\n');

fprintf('Starting training...\n');
fprintf('  Max episodes:  %d\n',  sac_opts.MaxEpisodes);
fprintf('  Steps/episode: %d\n',  sac_opts.MaxStepsPerEpisode);
fprintf('  Stop at avg reward > %.0f\n\n', sac_opts.StopTrainingValue);

% Run training
training_stats = train(sac_agent, sac_env, sac_opts);

% Save results
if ~exist('rl_toolbox_agents','dir'); mkdir('rl_toolbox_agents'); end
save('rl_toolbox_agents/sac_trained.mat', 'sac_agent', 'training_stats');
fprintf('\nTrained agent saved to rl_toolbox_agents/sac_trained.mat\n');

% Restore crash_detector and visualiser
if ~isempty(crash_block)
    set_param(crash_block{1},'Commented', crash_was_commented);
end
if ~isempty(vis_block)
    set_param(vis_block{1},'Commented', vis_commented);
    fprintf('Visualiser and crash detector restored.\n');
end

% Print summary
fprintf('\n=== TRAINING COMPLETE ===\n');
fprintf('  Episodes run:     %d\n', length(training_stats.EpisodeReward));
fprintf('  Final avg reward: %.2f\n', mean(training_stats.EpisodeReward(end-19:end)));
fprintf('  Best episode:     %.2f\n', max(training_stats.EpisodeReward));
fprintf('\nTo simulate with trained agent:\n');
fprintf('  >> sim_setup\n');
fprintf('  >> load(''rl_toolbox_agents/sac_trained.mat'')\n');
fprintf('  >> sim(''%s'')\n', MODEL_NAME);