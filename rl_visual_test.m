% =========================================================================
%  RL_VISUAL_TEST.m  —  Watch the trained agent drive in real time
%
%  Enables the visualiser block, runs a single simulation with RL ON,
%  then runs again with RL OFF so you can compare visually.
% =========================================================================

MODEL_NAME = 'MAIN_VEHICLE_MODEL';
AGENT_FILE = 'rl_toolbox_agents/sac_fullroad.mat';
RL_BLOCK   = 'MAIN_VEHICLE_MODEL/RL Agent';

%% ── Setup ────────────────────────────────────────────────────────────────
sim_setup;

tmp = load(AGENT_FILE, 'sac_agent');
assignin('base', 'sac_agent', tmp.sac_agent);
fprintf('Agent loaded.\n');

%% ── Find and ENABLE the visualiser block ────────────────────────────────
% Search for any block with 'visual' or 'vis' or 'display' in its name
all_blks = find_system(MODEL_NAME, 'SearchDepth', 2);
vis_blk = '';
for k = 1:length(all_blks)
    n = lower(get_param(all_blks{k}, 'Name'));
    if contains(n, 'visual') || contains(n, 'display') || contains(n, 'anim') || contains(n, 'plot')
        vis_blk = all_blks{k};
        fprintf('Found visualiser: %s\n', vis_blk);
        break;
    end
end

if ~isempty(vis_blk)
    set_param(vis_blk, 'Commented', 'off');
    fprintf('Visualiser enabled.\n');
else
    fprintf('[INFO] No visualiser block found by name search.\n');
    fprintf('       List all blocks:\n');
    for k = 1:length(all_blks)
        n = get_param(all_blks{k}, 'Name');
        fprintf('         %s\n', n);
    end
end

%% ── Model settings ───────────────────────────────────────────────────────
set_param(MODEL_NAME, 'StopTime',            '90');
set_param(MODEL_NAME, 'SimulationMode',      'normal');    % normal = real-time visual
set_param(MODEL_NAME, 'AlgebraicLoopMsg',    'none');

%% ── Challenge mu: kappa_s1 at training minimum ───────────────────────────
vr = evalin('base', 'vulnerability_results');
vr(1).mu_challenge = 0.343;   % kappa_s1 at RL minimum
assignin('base', 'vulnerability_results', vr);
assignin('base', 'sim_start_wp', 1);
assignin('base', 'rl_training_segment_id', 0);

%% ── RUN 1: RL Agent ON ───────────────────────────────────────────────────
fprintf('\n========================================\n');
fprintf('  RUN 1: RL AGENT ACTIVE\n');
fprintf('  Watch the vehicle navigate the road.\n');
fprintf('  kappa_s1 corner at t~37s with mu=0.343\n');
fprintf('========================================\n');

set_param(RL_BLOCK, 'Commented', 'off');
assignin('base', 'sac_agent', tmp.sac_agent);

input('Press Enter to start RUN 1 (RL ON)...');
sim(MODEL_NAME);

sl = evalin('base', 'sim_log');
fprintf('\nRUN 1 complete:\n');
fprintf('  Max |CTE|:   %.3f m\n', max(abs(sl.cte)));
fprintf('  Max |beta|:  %.1f deg\n', max(abs(sl.beta)) * 180/pi);
fprintf('  Duration:    %.1f s\n', sl.t(end));

%% ── RUN 2: Baseline (RL OFF) ─────────────────────────────────────────────
fprintf('\n========================================\n');
fprintf('  RUN 2: NOMINAL MPC (RL disabled)\n');
fprintf('  Same road, same mu — no RL adaptation.\n');
fprintf('========================================\n');

set_param(RL_BLOCK, 'Commented', 'on');

input('Press Enter to start RUN 2 (Baseline)...');
sim(MODEL_NAME);

sl_bl = evalin('base', 'sim_log');
fprintf('\nRUN 2 complete:\n');
fprintf('  Max |CTE|:   %.3f m\n', max(abs(sl_bl.cte)));
fprintf('  Max |beta|:  %.1f deg\n', max(abs(sl_bl.beta)) * 180/pi);
fprintf('  Duration:    %.1f s\n', sl_bl.t(end));

set_param(RL_BLOCK, 'Commented', 'off');

%% ── Side-by-side plot ────────────────────────────────────────────────────
figure('Name', 'RL vs Baseline — Vehicle Trajectory', 'Position', [100 100 1200 500]);

subplot(1,2,1);
plot(sl.X, sl.Y, 'b-', 'LineWidth', 2);
title('RL Agent');
xlabel('X [m]'); ylabel('Y [m]');
axis equal; grid on;
text(sl.X(1), sl.Y(1), '  start', 'Color', 'g', 'FontWeight', 'bold');

subplot(1,2,2);
plot(sl_bl.X, sl_bl.Y, 'r-', 'LineWidth', 2);
title('Nominal MPC (no RL)');
xlabel('X [m]'); ylabel('Y [m]');
axis equal; grid on;
text(sl_bl.X(1), sl_bl.Y(1), '  start', 'Color', 'g', 'FontWeight', 'bold');

figure('Name', 'RL vs Baseline — CTE and Slip', 'Position', [100 620 1200 400]);

subplot(1,2,1);
hold on;
plot(sl.t,    abs(sl.cte),    'b-',  'LineWidth', 1.5, 'DisplayName', 'RL');
plot(sl_bl.t, abs(sl_bl.cte), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
yline(1.5, 'k:', 'LineWidth', 1.2, 'DisplayName', 'road edge');
ylabel('|CTE| [m]'); xlabel('Time [s]');
title('Cross-track error'); legend; grid on;

subplot(1,2,2);
hold on;
plot(sl.t,    abs(sl.beta)    * 180/pi, 'b-',  'LineWidth', 1.5, 'DisplayName', 'RL');
plot(sl_bl.t, abs(sl_bl.beta) * 180/pi, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Baseline');
yline(20, 'k:', 'LineWidth', 1.2, 'DisplayName', 'crash threshold');
ylabel('|β| [deg]'); xlabel('Time [s]');
title('Body slip angle'); legend; grid on;

fprintf('\nDone. Figures show trajectory and stability comparison.\n');