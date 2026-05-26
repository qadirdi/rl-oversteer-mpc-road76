function vehicle_dynamics_sfun(block)
% =========================================================================
%  VEHICLE DYNAMICS MODEL  –  Level-2 MATLAB S-Function  (v3 – stiffness fixed)
%
%  KEY CHANGES vs v2 (solver crash fix):
%   1. sgn(w) → smooth tanh(w*30)  : eliminates discontinuous derivative
%   2. Jw  2.5 → 8.0  kg·m²        : reduces wheel ODE stiffness 3.2×
%   3. T_brake_max 1600 → 600 N·m  : reduces peak wheel deceleration 2.7×
%   4. VX_MIN 0.5 → 2.0 m/s        : prevents kappa blow-up at 60 km/h
%   5. Bx 10→8, By 8→7             : smoother Pacejka gradients
%   6. Derivative clamping          : safety net against any remaining spikes
%   7. vx0 = 60/3.6 m/s            : start at 60 km/h, aligned to road_ref
%   8. NumOutputPorts = 9           : Out8=vy, Out9=r for MPC adaptation
%
%  INPUTS
%   1  delta_sw   Steering wheel angle [rad]
%   2  accel_cmd  Throttle 0-1
%   3  brake_cmd  Brake    0-1
%   4  mu         Road friction (1.0=dry, 0.5=wet, 0.2=ice)
%
%  OUTPUTS
%   1  X          Global X       [m]
%   2  Y          Global Y       [m]
%   3  speed      |v|            [m/s]
%   4  psi        Heading        [rad] wrapped [-pi,pi]
%   5  pose_X     = X
%   6  pose_Y     = Y
%   7  pose_psi   = psi
%   8  vy         Lateral vel    [m/s]  → MPC In5
%   9  r          Yaw rate       [rad/s] → MPC In6
%
%  SOLVER SETTING (IMPORTANT)
%   Simulink → Model Settings → Solver:
%     Type         : Fixed-step
%     Solver       : ode4  (Runge-Kutta)
%     Step size    : 0.005  (5 ms)  ← changed from 0.001 for speed
%   This avoids the adaptive-step stiffness crash entirely.
% =========================================================================
setup(block);
end


% =========================================================================
function setup(block)
    block.NumInputPorts  = 4;
    block.NumOutputPorts = 9;
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    for k = 1:4
        block.InputPort(k).Dimensions        = 1;
        block.InputPort(k).DatatypeID        = 0;
        block.InputPort(k).Complexity        = 'Real';
        block.InputPort(k).DirectFeedthrough = false;  % Outputs() reads only states, not inputs
    end
    for k = 1:9
        block.OutputPort(k).Dimensions = 1;
        block.OutputPort(k).DatatypeID = 0;
        block.OutputPort(k).Complexity = 'Real';
    end

    block.NumContStates  = 10;
    block.NumDialogPrms  = 0;
    block.SampleTimes    = [0 0];   % continuous
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Derivatives',          @Derivatives);
    block.RegBlockMethod('Terminate',            @Terminate);
end


% =========================================================================
function InitConditions(block)
    Rw  = 0.320;
    vx0 = 50 / 3.6;     % [m/s] = 50 km/h start — realistic entrance speed

    X0 = 0;  Y0 = 0;  psi0 = 0;
    try
        rr = evalin('base', 'road_ref');
        if size(rr,1) >= 1 && size(rr,2) >= 3
            % sim_start_wp: set by rl_segment_trainer to teleport vehicle
            % to a specific waypoint (e.g. 50m before a risky segment).
            % Default = 1 (start of road) for normal simulation.
            start_wp = 1;
            try
                start_wp = max(1, min(size(rr,1), ...
                               round(evalin('base','sim_start_wp'))));
            catch
            end
            X0   = rr(start_wp, 1);
            Y0   = rr(start_wp, 2);
            psi0 = rr(start_wp, 3);
        end
    catch
    end

    w0 = vx0 / Rw;   % pre-spin wheels so kappa(0) = 0
    block.ContStates.Data = [vx0; 0; 0; X0; Y0; psi0; w0; w0; w0; w0];
end


% =========================================================================
function Outputs(block)
    xs  = block.ContStates.Data;
    vx  = xs(1);  vy = xs(2);  r = xs(3);
    X   = xs(4);  Y  = xs(5);  psi = xs(6);

    psi_w = mod(psi + pi, 2*pi) - pi;
    speed = sqrt(vx^2 + vy^2);

    block.OutputPort(1).Data = X;
    block.OutputPort(2).Data = Y;
    block.OutputPort(3).Data = speed;
    block.OutputPort(4).Data = psi_w;
    block.OutputPort(5).Data = X;
    block.OutputPort(6).Data = Y;
    block.OutputPort(7).Data = psi_w;
    block.OutputPort(8).Data = vy;
    block.OutputPort(9).Data = r;
end


% =========================================================================
function Derivatives(block)

    % ---- PARAMETERS ----
    m   = 1250;     Izz = 1500;
    a   = 1.08;     b   = 1.62;
    tf  = 1.55;     tr  = 1.56;
    Rw  = 0.320;    g   = 9.81;

    % Wheel inertia RAISED (2.5→8): max dw = 600/8 = 75 rad/s² (was 640)
    Jw  = 8.0;

    SR          = 15.0;
    T_eng_max   = 200;   % [N·m] per front wheel
    % Brake torque REDUCED (1600→600): still gives ~0.5g, far less stiff
    T_brake_max = 600;

    rho_air = 1.225;  Cd = 0.28;  A_f = 2.20;

    % Pacejka — slightly lower B for smoother gradients
    Bx = 8.0;  Cx = 1.90;  Dx_c = 1.05;  Ex = 0.97;
    By = 7.0;  Cy = 1.30;  Dy_c = 1.00;  Ey = -1.60;

    % VX_MIN RAISED (0.5→2.0): at 60 km/h hub vx never approaches zero
    VX_MIN = 2.0;

    % ---- STATES ----
    xs   = block.ContStates.Data;
    vx   = xs(1);  vy = xs(2);  r = xs(3);  psi = xs(6);
    w_fl = xs(7);  w_fr = xs(8);  w_rl = xs(9);  w_rr = xs(10);

    % ---- INPUTS ----
    delta_sw  = block.InputPort(1).Data;
    accel_cmd = max(0, min(1, block.InputPort(2).Data));
    brake_cmd = max(0, min(1, block.InputPort(3).Data));
    mu        = max(0.05, min(1.5, block.InputPort(4).Data));

    delta = delta_sw / SR;
    cd = cos(delta);  sd = sin(delta);

    % ---- NORMAL LOADS ----
    Fzf = m*g*b/(2*(a+b));  Fzr = m*g*a/(2*(a+b));
    Fz_fl = max(500,Fzf);  Fz_fr = max(500,Fzf);
    Fz_rl = max(500,Fzr);  Fz_rr = max(500,Fzr);

    % ---- HUB VELOCITIES (body frame) ----
    vx_fl=vx-r*tf/2;  vy_fl=vy+r*a;
    vx_fr=vx+r*tf/2;  vy_fr=vy+r*a;
    vx_rl=vx-r*tr/2;  vy_rl=vy-r*b;
    vx_rr=vx+r*tr/2;  vy_rr=vy-r*b;

    % ---- WHEEL FRAME VELOCITIES ----
    vx_fl_w= vx_fl*cd+vy_fl*sd;  vy_fl_w=-vx_fl*sd+vy_fl*cd;
    vx_fr_w= vx_fr*cd+vy_fr*sd;  vy_fr_w=-vx_fr*sd+vy_fr*cd;
    vx_rl_w=vx_rl;  vy_rl_w=vy_rl;
    vx_rr_w=vx_rr;  vy_rr_w=vy_rr;

    % ---- SLIP ANGLES (SAE J670: alpha = -atan(vy_w/vx_w)) ----
    vxfe = @(v) max(v, VX_MIN);
    alpha_fl=-atan(vy_fl_w/vxfe(vx_fl_w));
    alpha_fr=-atan(vy_fr_w/vxfe(vx_fr_w));
    alpha_rl=-atan(vy_rl_w/vxfe(vx_rl_w));
    alpha_rr=-atan(vy_rr_w/vxfe(vx_rr_w));

    % ---- SLIP RATIOS ----
    cl=@(x)max(-1,min(1,x));
    kappa_fl=cl((w_fl*Rw-vx_fl_w)/max(abs(vx_fl_w),VX_MIN));
    kappa_fr=cl((w_fr*Rw-vx_fr_w)/max(abs(vx_fr_w),VX_MIN));
    kappa_rl=cl((w_rl*Rw-vx_rl_w)/max(abs(vx_rl_w),VX_MIN));
    kappa_rr=cl((w_rr*Rw-vx_rr_w)/max(abs(vx_rr_w),VX_MIN));

    % ---- TIRE FORCES ----
    [Fx_fl_w,Fy_fl_w]=mf_tire(kappa_fl,alpha_fl,mu,Fz_fl,Bx,Cx,Dx_c,Ex,By,Cy,Dy_c,Ey);
    [Fx_fr_w,Fy_fr_w]=mf_tire(kappa_fr,alpha_fr,mu,Fz_fr,Bx,Cx,Dx_c,Ex,By,Cy,Dy_c,Ey);
    [Fx_rl_w,Fy_rl_w]=mf_tire(kappa_rl,alpha_rl,mu,Fz_rl,Bx,Cx,Dx_c,Ex,By,Cy,Dy_c,Ey);
    [Fx_rr_w,Fy_rr_w]=mf_tire(kappa_rr,alpha_rr,mu,Fz_rr,Bx,Cx,Dx_c,Ex,By,Cy,Dy_c,Ey);

    % ---- BODY FRAME FORCES ----
    Fx_fl_v= Fx_fl_w*cd-Fy_fl_w*sd;  Fy_fl_v= Fx_fl_w*sd+Fy_fl_w*cd;
    Fx_fr_v= Fx_fr_w*cd-Fy_fr_w*sd;  Fy_fr_v= Fx_fr_w*sd+Fy_fr_w*cd;
    Fx_rl_v=Fx_rl_w;  Fy_rl_v=Fy_rl_w;
    Fx_rr_v=Fx_rr_w;  Fy_rr_v=Fy_rr_w;

    % ---- AERO DRAG ----
    F_drag=0.5*rho_air*Cd*A_f*vx*abs(vx);

    % ---- EQUATIONS OF MOTION ----
    Fx_sum=Fx_fl_v+Fx_fr_v+Fx_rl_v+Fx_rr_v-F_drag;
    Fy_sum=Fy_fl_v+Fy_fr_v+Fy_rl_v+Fy_rr_v;
    Mz = a*(Fy_fl_v+Fy_fr_v) - b*(Fy_rl_v+Fy_rr_v) ...
       + (tf/2)*(-Fx_fl_v+Fx_fr_v) + (tr/2)*(-Fx_rl_v+Fx_rr_v);

    dvx = Fx_sum/m + vy*r;
    dvy = Fy_sum/m - vx*r;
    dr  = Mz/Izz - 3.0*r*exp(-vx^2/8.0);   % mild low-speed damping

    dX   = vx*cos(psi)-vy*sin(psi);
    dY   = vx*sin(psi)+vy*cos(psi);
    dpsi = r;

    % ---- WHEEL SPIN ----
    % SMOOTH sign: tanh(w*30) ≈ sign(w) but C-infinity → no solver discontinuity
    k_s = 30.0;
    T_eng_fl = accel_cmd*T_eng_max/2;
    T_eng_fr = accel_cmd*T_eng_max/2;

    dw_fl=(T_eng_fl - tanh(w_fl*k_s)*brake_cmd*T_brake_max - Fx_fl_w*Rw)/Jw;
    dw_fr=(T_eng_fr - tanh(w_fr*k_s)*brake_cmd*T_brake_max - Fx_fr_w*Rw)/Jw;
    dw_rl=(         - tanh(w_rl*k_s)*brake_cmd*T_brake_max - Fx_rl_w*Rw)/Jw;
    dw_rr=(         - tanh(w_rr*k_s)*brake_cmd*T_brake_max - Fx_rr_w*Rw)/Jw;

    % ---- CLAMP DERIVATIVES (catches any remaining numerical spikes) ----
    dvx  = clamp(dvx,  -50,  50);
    dvy  = clamp(dvy,  -50,  50);
    dr   = clamp(dr,   -30,  30);
    dX   = clamp(dX,  -100, 100);
    dY   = clamp(dY,  -100, 100);
    dw_fl= clamp(dw_fl,-200, 200);
    dw_fr= clamp(dw_fr,-200, 200);
    dw_rl= clamp(dw_rl,-200, 200);
    dw_rr= clamp(dw_rr,-200, 200);

    block.Derivatives.Data = [dvx;dvy;dr;dX;dY;dpsi;dw_fl;dw_fr;dw_rl;dw_rr];
end


% =========================================================================
function Terminate(~)
end


% =========================================================================
%  Pacejka Magic Formula + Friction Ellipse
% =========================================================================
function [Fx,Fy] = mf_tire(kappa,alpha,mu,Fz,Bx,Cx,Dxc,Ex,By,Cy,Dyc,Ey)
    Dx=mu*Fz*Dxc;  Dy=mu*Fz*Dyc;
    px=Bx*kappa;   Fx0=Dx*sin(Cx*atan(px-Ex*(px-atan(px))));
    ta=tan(alpha);  py=By*ta;  Fy0=Dy*sin(Cy*atan(py-Ey*(py-atan(py))));
    sx=abs(kappa);  sy=abs(ta);  s=sqrt(sx^2+sy^2);
    if s>1e-9;  Fx=(sx/s)*Fx0;  Fy=(sy/s)*Fy0;
    else;       Fx=Fx0;          Fy=Fy0;
    end
end


% =========================================================================
function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end