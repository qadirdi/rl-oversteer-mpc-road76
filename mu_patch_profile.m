function mu_val = mu_patch_profile(s_query, s0, mu_nominal, mu_low, L_patch, transition_len)
% =========================================================================
%  MU_PATCH_PROFILE   –   road friction profile with sigmoid transitions
%
%  Models a localised low-friction patch on an otherwise uniform road.
%  The mu value transitions from mu_nominal → mu_low at the patch onset
%  (s0) and back from mu_low → mu_nominal at the patch end (s0+L_patch),
%  each transition occurring smoothly over `transition_len` metres using
%  a logistic (sigmoid) function.
%
%  INPUTS
%    s_query        scalar or vector of arc-length positions to evaluate [m]
%    s0             patch onset arc-length  [m]
%    mu_nominal     baseline friction (outside patch)          default 0.7
%    mu_low         minimum friction (centre of patch)         [0.2 – 0.7]
%    L_patch        total patch length [m]  (includes transitions)
%    transition_len transition zone length at each end [m]     default 2.0
%                   Must be < L_patch/2
%
%  OUTPUT
%    mu_val         friction coefficient at each query position
%
%  EXAMPLE
%    s  = 0:0.1:100;
%    mu = mu_patch_profile(s, 30, 0.7, 0.3, 20, 2);
%    plot(s, mu);
%
%  SIGMOID SHAPE
%    The logistic function  σ(x) = 1/(1+exp(-k*x))  is used.
%    The steepness k is chosen so that 95% of the transition is completed
%    within `transition_len` metres:
%      k = 2*log(19) / transition_len   ≈ 5.89 / transition_len
%
%  COMBINED PROFILE
%    mu(s) = mu_nominal - (mu_nominal - mu_low) *
%            sigmoid_rise(s - s0) * sigmoid_fall(s - (s0+L_patch))
%
%    where:
%      sigmoid_rise(x)  = σ(k*x)          goes 0→1 at x=0
%      sigmoid_fall(x)  = 1 - σ(k*x)      goes 1→0 at x=0
% =========================================================================

if nargin < 6 || isempty(transition_len)
    transition_len = 2.0;   % [m]  default 2-metre transition zone
end

% Clamp transition to half the patch (can't have overlapping ramps)
transition_len = min(transition_len, L_patch/2 - 0.1);
transition_len = max(transition_len, 0.1);

% Sigmoid steepness: 95% transition in transition_len metres
k = 2 * log(19) / transition_len;   % ≈ 5.89 / transition_len

% Depth of friction reduction
delta_mu = mu_nominal - mu_low;

% Positions of onset and end of patch
s_start = s0;
s_end   = s0 + L_patch;

% Sigmoid rise at patch onset:  0 → 1 over [s_start]
rise  = 1 ./ (1 + exp(-k * (s_query - s_start)));

% Sigmoid fall at patch end:    1 → 0 over [s_end]
fall  = 1 - 1 ./ (1 + exp(-k * (s_query - s_end)));

% Combined envelope (product gives the hat shape)
envelope = rise .* fall;

% Final friction profile
mu_val = mu_nominal - delta_mu * envelope;

% Hard clamp for safety
mu_val = max(0.05, min(1.5, mu_val));

end