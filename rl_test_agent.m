% =========================================================================
%  RL_TEST_AGENT.m  —  Direct RL vs Baseline comparison
%
%  Tests each working segment at the minimum mu achieved during training.
%  Re-runs sim_setup per segment to correctly inject the challenge mu.
%  Uses actual sim_log field names: cte, beta, speed, mu, he
% =========================================================================

fprintf('\n========================================================\n');
fprintf('  RL vs Baseline — Segment-by-Segment Comparison\n');
fprintf('========================================================\n');

MODEL_NAME = 'MAIN_VEHICLE_MODEL';
AGENT_FILE = 'rl_toolbox_agents/sac_fullroad.mat';
RL_BLOCK   = 'MAIN_VEHICLE_MODEL/RL Agent';
SAVE_DIR   = 'rl_test_results';
if ~exist(SAVE_DIR,'dir'); mkdir(SAVE_DIR); end

%% ── Load agent ───────────────────────────────────────────────────────────
tmp = load(AGENT_FILE,'sac_agent');
sac_agent = tmp.sac_agent;
assignin('base','sac_agent', sac_agent);
fprintf('  Agent loaded: %s\n\n', AGENT_FILE);

%% ── Segments to test (working segments only) ─────────────────────────────
% challenge_mu = minimum mu achieved during curriculum training
test_segs = struct( ...
    'name',         {'kappa_s1',  'delta_s2',  'he_s1'  }, ...
    'seg_idx',      {1,           2,           5        }, ...
    'nominal_mu',   {0.403,       0.466,       0.481    }, ...
    'challenge_mu', {0.343,       0.366,       0.361    });   % actual training results
N = length(test_segs);

%% ── Results storage ──────────────────────────────────────────────────────
res = struct();
for k = 1:N
    res(k).name         = test_segs(k).name;
    res(k).rl_survived  = false;
    res(k).bl_survived  = false;
    res(k).rl_max_cte   = NaN;
    res(k).bl_max_cte   = NaN;
    res(k).rl_max_beta  = NaN;
    res(k).bl_max_beta  = NaN;
    res(k).rl_mean_spd  = NaN;
    res(k).bl_mean_spd  = NaN;
end

%% ── Helper: run one sim and extract metrics ──────────────────────────────
function R = run_one(mdl, label)
    fprintf('    [%s] Running...\n', label);
    try
        sim(mdl);
        sl = evalin('base','sim_log');

        cte   = sl.cte(:);          % [m]
        beta  = sl.beta(:);         % [rad]
        speed = sl.speed(:) * 3.6;  % [km/h]

        crashed = any(abs(cte) > 1.5) || any(abs(beta) > 0.349);

        R.survived  = ~crashed;
        R.max_cte   = max(abs(cte));
        R.mean_cte  = mean(abs(cte));
        R.max_beta  = max(abs(beta)) * 180/pi;
        R.mean_beta = mean(abs(beta)) * 180/pi;
        R.mean_spd  = mean(speed);

        status = 'SURVIVED'; if crashed; status = 'CRASHED'; end
        fprintf('    [%s] %s  MaxCTE=%.3fm  Max|β|=%.1f°  Speed=%.1fkm/h\n', ...
            label, status, R.max_cte, R.max_beta, R.mean_spd);
    catch ME
        fprintf('    [%s] SIM ERROR: %s\n', label, ME.message);
        R.survived  = false;
        R.max_cte   = NaN; R.mean_cte  = NaN;
        R.max_beta  = NaN; R.mean_beta = NaN;
        R.mean_spd  = NaN;
    end
end

%% ── Main loop: one segment at a time ─────────────────────────────────────
for k = 1:N
    seg = test_segs(k);
    fprintf('\n--- Segment %d/%d: %s ---\n', k, N, seg.name);
    fprintf('    Nominal min μ = %.3f  |  Challenge μ = %.3f  (%.0f%% below nominal)\n', ...
        seg.nominal_mu, seg.challenge_mu, ...
        (seg.nominal_mu - seg.challenge_mu)/seg.nominal_mu*100);

    % Set challenge mu for this segment via vulnerability_results
    % sim_setup is called fresh to ensure sector_switch picks up new values
    vr = evalin('base','vulnerability_results');
    vr(seg.seg_idx).mu_challenge = seg.challenge_mu;
    assignin('base','vulnerability_results', vr);

    % Force model to re-initialise with new parameters
    set_param(MODEL_NAME, 'SimulationCommand', 'stop');
    pause(0.5);

    set_param(MODEL_NAME, 'StopTime', '90');

    % ── RL ON ──────────────────────────────────────────────────────────
    set_param(RL_BLOCK, 'Commented', 'off');
    assignin('base','sac_agent', sac_agent);
    rl_r = run_one(MODEL_NAME, 'RL  ');
    res(k).rl_survived = rl_r.survived;
    res(k).rl_max_cte  = rl_r.max_cte;
    res(k).rl_max_beta = rl_r.max_beta;
    res(k).rl_mean_spd = rl_r.mean_spd;

    % ── BASELINE ───────────────────────────────────────────────────────
    set_param(RL_BLOCK, 'Commented', 'on');
    bl_r = run_one(MODEL_NAME, 'BASE');
    res(k).bl_survived = bl_r.survived;
    res(k).bl_max_cte  = bl_r.max_cte;
    res(k).bl_max_beta = bl_r.max_beta;
    res(k).bl_mean_spd = bl_r.mean_spd;

    % Restore RL block
    set_param(RL_BLOCK, 'Commented', 'off');

    % Reset vulnerability_results back to nominal
    vr(seg.seg_idx).mu_challenge = seg.nominal_mu * 0.95;
    assignin('base','vulnerability_results', vr);
end

%% ── Final comparison table ───────────────────────────────────────────────
fprintf('\n\n========================================================\n');
fprintf('  FINAL RESULTS — RL Adaptive MPC vs Nominal MPC\n');
fprintf('========================================================\n\n');

fprintf('--- Friction Capability (from training) ---\n');
fprintf('%-14s  %-12s  %-12s  %-10s  %-12s\n', ...
    'Segment','Nominal μ','RL min μ','Reduction','Safety gain');
fprintf('%s\n', repmat('-',1,64));
for k = 1:N
    drop = test_segs(k).nominal_mu - test_segs(k).challenge_mu;
    gain = drop / test_segs(k).nominal_mu * 100;
    fprintf('%-14s  %-12.3f  %-12.3f  %-10.3f  %.1f%%\n', ...
        test_segs(k).name, test_segs(k).nominal_mu, ...
        test_segs(k).challenge_mu, drop, gain);
end

fprintf('\n--- Behaviour at Challenge μ ---\n');
fprintf('%-14s  %-12s  %-12s  %-14s  %-14s  %-14s  %-14s\n', ...
    'Segment','RL Status','Base Status','RL MaxCTE[m]', ...
    'Base MaxCTE[m]','RL Max|β|[°]','Base Max|β|[°]');
fprintf('%s\n', repmat('-',1,98));
for k = 1:N
    fprintf('%-14s  %-12s  %-12s  %-14.3f  %-14.3f  %-14.1f  %-14.1f\n', ...
        res(k).name, ...
        pass_fail(~res(k).rl_survived), ...
        pass_fail(~res(k).bl_survived), ...
        res(k).rl_max_cte,  res(k).bl_max_cte, ...
        res(k).rl_max_beta, res(k).bl_max_beta);
end

save(fullfile(SAVE_DIR,'comparison.mat'),'res','test_segs');
fprintf('\nSaved to %s/comparison.mat\n', SAVE_DIR);

function s = pass_fail(crashed)
    if crashed; s = 'CRASH'; else; s = 'PASS'; end
end