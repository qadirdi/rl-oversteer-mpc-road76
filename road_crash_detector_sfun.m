function road_crash_detector_sfun(block)
% =========================================================================
%  ROAD CRASH DETECTOR  –  Level-2 MATLAB S-Function
%
%  Checks all four vehicle corners against the road boundary every 20 ms.
%  When any corner exits the road, the simulation is stopped cleanly
%  (Terminate callbacks run, sim_log is saved normally).
%
%  A grace period of WARMUP_S seconds ignores departures at the start
%  while the MPC is still aligning with the path.
%  A confirmation count requires OFF_COUNT consecutive off-road samples
%  before triggering – avoids false positives from single noisy frames.
%
%  ── INPUTS ────────────────────────────────────────────────────────────────
%   In1  X    – vehicle global X   [m]  ← vehicle Out1
%   In2  Y    – vehicle global Y   [m]  ← vehicle Out2
%   In3  psi  – vehicle heading   [rad] ← vehicle Out4
%
%  ── OUTPUTS ───────────────────────────────────────────────────────────────
%   Out1  crash_flag  –  0 = on road,  1 = off road / crashed
%
%  ── SIDE EFFECT ───────────────────────────────────────────────────────────
%  When crash is confirmed:
%    1. assignin('base', 'sim_crashed', true)  → pipeline reads this
%    2. set_param(bdroot, 'SimulationCommand', 'stop')  → stops simulation
%
%  ── SIMULINK WIRING ───────────────────────────────────────────────────────
%   vehicle Out1 (X)   → In1
%   vehicle Out2 (Y)   → In2
%   vehicle Out4 (psi) → In3
%   Out1 → Scope or Terminator (not needed for operation, just for monitoring)
%
%  ── WORKSPACE REQUIREMENT ────────────────────────────────────────────────
%   road_ref  [N×3]  – built by build_road_ref()
% =========================================================================
setup(block);
end


% =========================================================================
function setup(block)
    block.NumInputPorts  = 3;
    block.NumOutputPorts = 1;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    for k = 1:3
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = false;  % reads DWork in Outputs
    end

    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0;
    block.OutputPort(1).Complexity = 'Real';

    block.NumDialogPrms  = 0;
    block.NumContStates  = 0;
    block.SampleTimes    = [0.02 0];     % 50 Hz – fast boundary check
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('PostPropagationSetup', @PostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% =========================================================================
%  DWORK
%   1  prev_idx      [1]  – waypoint search hint (Update)
%   2  off_count     [1]  – consecutive off-road sample counter
%   3  has_stopped   [1]  – flag: 1 = stop already requested (no repeat)
%   4  crash_out     [1]  – value returned by Outputs (0 or 1)
% =========================================================================
function PostPropSetup(block)
    block.NumDworks = 4;
    specs = {'prev_idx',1; 'off_count',1; 'has_stopped',1; 'crash_out',1};
    for k = 1:4
        block.Dwork(k).Name            = specs{k,1};
        block.Dwork(k).Dimensions      = specs{k,2};
        block.Dwork(k).DatatypeID      = 0;
        block.Dwork(k).Complexity      = 'Real';
        block.Dwork(k).UsedAsDiscState = true;
    end
end


% =========================================================================
function InitConditions(block)
    % Initialise prev_idx from sim_start_wp so bounded search
    % starts near the teleport position, not at wp 1
    start_wp = 1;
    try
        start_wp = max(1, round(evalin('base','sim_start_wp')));
    catch
    end
    block.Dwork(1).Data = double(start_wp);  % prev_idx
    block.Dwork(2).Data = 0;    % off_count
    block.Dwork(3).Data = 0;    % has_stopped
    block.Dwork(4).Data = 0;    % crash_out
    assignin('base', 'sim_crashed', false);
end


% =========================================================================
%  OUTPUTS  –  returns stored crash flag (no feedthrough)
% =========================================================================
function Outputs(block)
    block.OutputPort(1).Data = block.Dwork(4).Data;
end


% =========================================================================
%  UPDATE  –  check boundaries, stop simulation if off-road
% =========================================================================
function Update(block)

    % ── Tuning constants ─────────────────────────────────────────────────
    ROAD_HW   = 3.3;    % [m]  road half-width crash threshold
                        %       (road centre ±3.3 m; visualiser draws ±3.0 m)
    CAR_HW    = 0.95;   % [m]  car half-width  (1.8 m body + small buffer)
    CAR_HL    = 2.40;   % [m]  car half-length (4.6 m body + small buffer)
    WARMUP_S  = 8.0;    % [s]  ignore crash detection during warm-up
    OFF_COUNT = 3;      % consecutive off-road samples before triggering

    % ── Already stopped? ─────────────────────────────────────────────────
    if block.Dwork(3).Data > 0;  return;  end

    % ── Warm-up grace period ─────────────────────────────────────────────
    if block.CurrentTime < WARMUP_S;  return;  end

    % ── Read inputs ───────────────────────────────────────────────────────
    X   = block.InputPort(1).Data;
    Y   = block.InputPort(2).Data;
    psi = block.InputPort(3).Data;

    % ── Load road_ref ─────────────────────────────────────────────────────
    try
        road_ref = evalin('base', 'road_ref');
    catch
        return;   % no road_ref yet – skip
    end
    N    = size(road_ref, 1);
    prev = max(1, min(N, round(block.Dwork(1).Data)));

    % ── Find closest waypoint (bounded search) ────────────────────────────
    ilo = max(1,   prev - 5);
    ihi = min(N,   prev + 80);
    dx  = road_ref(ilo:ihi, 1) - X;
    dy  = road_ref(ilo:ihi, 2) - Y;
    [~, li] = min(dx.^2 + dy.^2);
    idx = ilo + li - 1;
    block.Dwork(1).Data = double(idx);

    xr  = road_ref(idx, 1);
    yr  = road_ref(idx, 2);
    hdr = road_ref(idx, 3);

    % ── Compute 4 corner positions in global frame ─────────────────────────
    % Corner offsets in vehicle body frame:  (±HL, ±HW)
    corners_local = [ CAR_HL,  CAR_HW;   % front-left
                      CAR_HL, -CAR_HW;   % front-right
                     -CAR_HL,  CAR_HW;   % rear-left
                     -CAR_HL, -CAR_HW];  % rear-right

    cos_p = cos(psi);  sin_p = sin(psi);
    off_road = false;

    for c = 1:4
        cx = X + cos_p*corners_local(c,1) - sin_p*corners_local(c,2);
        cy = Y + sin_p*corners_local(c,1) + cos_p*corners_local(c,2);

        % Signed lateral distance from closest road centreline point
        % positive = left of road heading, negative = right
        dcx = cx - xr;
        dcy = cy - yr;
        lat_dist = -sin(hdr)*dcx + cos(hdr)*dcy;

        if abs(lat_dist) > ROAD_HW
            off_road = true;
            break;
        end
    end

    % ── Update consecutive counter ────────────────────────────────────────
    if off_road
        cnt = block.Dwork(2).Data + 1;
        block.Dwork(2).Data = cnt;
    else
        block.Dwork(2).Data = 0;   % reset on any on-road sample
    end

    % ── Trigger crash if confirmed ────────────────────────────────────────
    if block.Dwork(2).Data >= OFF_COUNT
        block.Dwork(4).Data = 1;    % set output flag
        block.Dwork(3).Data = 1;    % mark as stopped (prevent repeat)

        % Save crash flag to workspace so pipeline crash detection sees it
        assignin('base', 'sim_crashed', true);
        assignin('base', 'sim_crash_time', block.CurrentTime);
        assignin('base', 'sim_crash_wp',   idx);

        fprintf('[crash_detector] Off-road at t=%.2f s  (wp %d,  %.0f m from start)\n', ...
                block.CurrentTime, idx, idx * 1.0);

        % Stop the simulation cleanly (Terminate callbacks still run)
        set_param(bdroot, 'SimulationCommand', 'stop');
    end
end


% =========================================================================
function Terminate(block)
    if block.Dwork(3).Data > 0
        fprintf('[crash_detector] Simulation stopped due to road departure.\n');
    end
end