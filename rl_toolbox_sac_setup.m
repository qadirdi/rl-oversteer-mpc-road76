% =========================================================================
%  RL_TOOLBOX_SAC_SETUP.m
%  Written against confirmed rl.option.rlSACAgentOptions properties:
%    EntropyWeightOptions, ActorOptimizerOptions, CriticOptimizerOptions,
%    TargetSmoothFactor, TargetUpdateFrequency, MiniBatchSize,
%    NumStepsToLookAhead, ExperienceBufferLength, SampleTime,
%    DiscountFactor, NumWarmStartSteps, LearningFrequency,
%    PolicyUpdateFrequency
% =========================================================================

MODEL_NAME = 'MAIN_VEHICLE_MODEL';

%% ── Observation / Action spaces ─────────────────────────────────────────
obs_info = rlNumericSpec([16 1], ...
    'LowerLimit', -ones(16,1), ...
    'UpperLimit',  ones(16,1));
obs_info.Name = 'observations';

act_info = rlNumericSpec([4 1], ...
    'LowerLimit', -ones(4,1), ...
    'UpperLimit',  ones(4,1));
act_info.Name = 'actions';

%% ── Environment ──────────────────────────────────────────────────────────
env = rlSimulinkEnv(MODEL_NAME, [MODEL_NAME '/RL Agent'], obs_info, act_info);
env.ResetFcn = @rl_reset_fn;

%% ── Actor network ────────────────────────────────────────────────────────
obs_in = featureInputLayer(16, 'Name', 'obs_in');

trunk = [
    fullyConnectedLayer(256, 'Name', 'fc1')
    layerNormalizationLayer('Name', 'ln1')
    reluLayer('Name', 'rel1')
    fullyConnectedLayer(256, 'Name', 'fc2')
    layerNormalizationLayer('Name', 'ln2')
    reluLayer('Name', 'rel2')
    fullyConnectedLayer(128, 'Name', 'fc3')
    reluLayer('Name', 'rel3')
];

mean_branch = [
    fullyConnectedLayer(4, 'Name', 'mean_fc')
    tanhLayer('Name', 'mean_tanh')
];

std_branch = [
    fullyConnectedLayer(4, 'Name', 'std_fc')
    softplusLayer('Name', 'std_sp')
];

actor_net = layerGraph(obs_in);
actor_net = addLayers(actor_net, trunk);
actor_net = addLayers(actor_net, mean_branch);
actor_net = addLayers(actor_net, std_branch);
actor_net = connectLayers(actor_net, 'obs_in',  'fc1');
actor_net = connectLayers(actor_net, 'rel3',    'mean_fc');
actor_net = connectLayers(actor_net, 'rel3',    'std_fc');

actor = rlContinuousGaussianActor(actor_net, obs_info, act_info, ...
    'ActionMeanOutputNames',              {'mean_tanh'}, ...
    'ActionStandardDeviationOutputNames', {'std_sp'}, ...
    'ObservationInputNames',              {'obs_in'});

%% ── Critic networks ──────────────────────────────────────────────────────
critic1 = buildCritic(obs_info, act_info, 1);
critic2 = buildCritic(obs_info, act_info, 2);

%% ── Agent options (using only confirmed properties) ──────────────────────
agent_opts = rlSACAgentOptions;

% Entropy weight
agent_opts.EntropyWeightOptions.EntropyWeight = 0.2;

% Learning rates — on agent_opts, not on actor/critic objects
agent_opts.ActorOptimizerOptions.LearnRate             = 3e-4;
agent_opts.ActorOptimizerOptions.GradientThreshold     = 1;
agent_opts.CriticOptimizerOptions(1).LearnRate         = 3e-4;
agent_opts.CriticOptimizerOptions(1).GradientThreshold = 1;
agent_opts.CriticOptimizerOptions(2).LearnRate         = 3e-4;
agent_opts.CriticOptimizerOptions(2).GradientThreshold = 1;

% Replay buffer
agent_opts.ExperienceBufferLength = 200000;
agent_opts.MiniBatchSize          = 256;

% Warm-up: collect this many steps before first network update
agent_opts.NumWarmStartSteps      = 1000;

% Core RL parameters
agent_opts.DiscountFactor         = 0.99;
agent_opts.TargetSmoothFactor     = 5e-3;
agent_opts.TargetUpdateFrequency  = 1;
agent_opts.SampleTime             = 0.10;
agent_opts.NumStepsToLookAhead    = 1;

%% ── Create agent ─────────────────────────────────────────────────────────
sac_agent = rlSACAgent(actor, [critic1, critic2], agent_opts);

%% ── Training options ─────────────────────────────────────────────────────
sac_opts = rlTrainingOptions( ...
    'MaxEpisodes',                5000, ...
    'MaxStepsPerEpisode',         900,  ...
    'ScoreAveragingWindowLength', 50,   ...
    'StopTrainingCriteria',       'none', ...
    'Verbose',                    true, ...
    'Plots',                      'training-progress');

assignin('base', 'sac_agent', sac_agent);
assignin('base', 'sac_opts',  sac_opts);
assignin('base', 'env',       env);

fprintf('\nSAC agent ready:\n');
fprintf('  Obs: 16-dim | Act: 4-dim\n');
fprintf('  Actor:   256-256-128 + LayerNorm\n');
fprintf('  Critics: twin Q-functions\n');
fprintf('  Buffer:  200k | Batch: 256 | Warmup: 1000 steps\n\n');


%% ── Reset function ───────────────────────────────────────────────────────
function in = rl_reset_fn(in)
    N_wp = length(evalin('base', 'road_ref'));
    ci   = evalin('base', 'rl_curriculum_info');

    mu_arr            = 0.7 * ones(N_wp, 1);
    s1                = max(1,    ci.ext_start);
    s2                = min(N_wp, ci.ext_end);
    mu_arr(s1:s2)     = ci.mu_test;

    assignin('base', 'mu_road_array',             mu_arr);
    assignin('base', 'sim_start_wp',              1);
    assignin('base', 'rl_training_segment_id',    ci.seg_id);

    ov.active=false; ov.dw_cte=0; ov.dw_he=0; ov.dw_ddelta=0; ov.dv_target=0;
    assignin('base', 'rl_param_override', ov);
end


%% ── Critic builder ───────────────────────────────────────────────────────
function critic = buildCritic(obs_info, act_info, idx)
    pfx = ['q' num2str(idx) '_'];

    obs_in = featureInputLayer(16, 'Name', 'obs_in');
    act_in = featureInputLayer(4,  'Name', 'act_in');

    obs_enc = [
        fullyConnectedLayer(256, 'Name', [pfx 'ofc'])
        reluLayer(               'Name', [pfx 'orl'])
    ];
    act_enc = [
        fullyConnectedLayer(64,  'Name', [pfx 'afc'])
        reluLayer(               'Name', [pfx 'arl'])
    ];
    combined = [
        concatenationLayer(1, 2, 'Name', [pfx 'cat'])
        fullyConnectedLayer(256, 'Name', [pfx 'fc2'])
        layerNormalizationLayer( 'Name', [pfx 'ln2'])
        reluLayer(               'Name', [pfx 'rl2'])
        fullyConnectedLayer(128, 'Name', [pfx 'fc3'])
        reluLayer(               'Name', [pfx 'rl3'])
        fullyConnectedLayer(1,   'Name', [pfx 'out'])
    ];

    net = layerGraph(obs_in);
    net = addLayers(net, act_in);
    net = addLayers(net, obs_enc);
    net = addLayers(net, act_enc);
    net = addLayers(net, combined);
    net = connectLayers(net, 'obs_in',     [pfx 'ofc']);
    net = connectLayers(net, 'act_in',     [pfx 'afc']);
    net = connectLayers(net, [pfx 'orl'],  [pfx 'cat/in1']);
    net = connectLayers(net, [pfx 'arl'],  [pfx 'cat/in2']);

    critic = rlQValueFunction(net, obs_info, act_info, ...
        'ObservationInputNames', {'obs_in'}, ...
        'ActionInputNames',      {'act_in'});
end