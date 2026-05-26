% =========================================================================
%  SIM_SETUP.m
%  Run this ONCE before every simulation session.
%  Takes about 2 seconds.
%
%  >> sim_setup
%  >> sim('MAIN_VEHICLE_MODEL')
% =========================================================================
fprintf('Setting up workspace for simulation...\n');

%% Road reference
if ~exist('road_ref','var') || size(road_ref,2) ~= 3
    fprintf('  Building road_ref...\n');
    road_ref = build_road_ref();
else
    fprintf('  road_ref: %d waypoints [OK]\n', size(road_ref,1));
end
N = size(road_ref,1);

%% Friction array (nominal everywhere)
mu_road_array = 0.7 * ones(N, 1);
fprintf('  mu_road_array: %d values at 0.7 [OK]\n', N);

%% Vulnerability results
if ~exist('vulnerability_results','var')
    if exist('vulnerability_map.mat','file')
        s = load('vulnerability_map.mat');
        vulnerability_results = s.results;
        fprintf('  vulnerability_results: loaded from file [OK]\n');
    else
        error('vulnerability_map.mat not found. Run run_vulnerability_pipeline first.');
    end
else
    fprintf('  vulnerability_results: %d segments [OK]\n', ...
            length(vulnerability_results));
end

%% RL variables
rl_param_override.active    = false;
rl_param_override.dw_cte    = 0;
rl_param_override.dw_he     = 0;
rl_param_override.dw_ddelta = 0;
rl_param_override.dv_target = 0;

rl_active_segment.id            = 0;
rl_active_segment.mu_min        = 0.7;
rl_active_segment.mu_challenge  = 0.7;

if ~exist('rl_agent_mode','var')
    rl_agent_mode = 'heuristic';
rl_training_segment_id = 0;  % 0 = activate all segments (normal mode)
end

%% Crash flags
sim_crashed    = false;
sim_crash_time = 0;
sim_crash_wp   = 0;

%% Clear stale experience buffer if it has wrong state dimensions
% (happens when upgrading from 15-dim to 18-dim state)
if exist('rl_experience_buffer','var')
    if size(rl_experience_buffer.states,2) ~= 18
        clear rl_experience_buffer;
        fprintf('  Cleared stale experience buffer (wrong state size)\n');
    end
end

%% MPC mode (make sure fast mode is OFF for real runs)
mpc_fast_mode  = false;

%% Print segment summary so you know what to expect
fprintf('\n  Risky segments that will be challenged:\n');
segs = vulnerability_results.segments;
for k = 1:length(segs)
    if isfield(segs(k),'min_safe_mu') && ~isnan(segs(k).min_safe_mu)
        fprintf('    Seg %d  %-12s  wp %4d–%-4d  mu_challenge=%.3f\n', ...
                k, segs(k).name, segs(k).start, segs(k).end, ...
                segs(k).min_safe_mu - 0.02);
    end
end

fprintf('\n  rl_agent_mode = ''%s''\n', rl_agent_mode);
fprintf('\nWorkspace ready. Run simulation:\n');
fprintf('  >> sim(''MAIN_VEHICLE_MODEL'')\n\n');