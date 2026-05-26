# RL-Based Oversteer Control Under Dynamically Changing Adhesion

**MSc Thesis — Budapest University of Technology and Economics (BME)**
**Department of Automotive Technologies**
**Author:** Abdulgadir (Qadir) Ahmadov
**Supervisor:** Tóth Szilárd Hunor
**Year:** 2026

---

## Overview

This repository contains the complete MATLAB/Simulink simulation environment developed for the MSc thesis:

> *"Reinforcement Learning Based Oversteer Control Under Dynamically Changing Adhesion"*

The system trains a Soft Actor-Critic (SAC) reinforcement learning agent to adaptively tune the cost function weights of a Model Predictive Control (MPC) path-tracking controller in real time, improving vehicle stability on low-friction road sections compared to a nominal baseline controller.

The simulation environment is built around a real Hungarian road segment — **Road 76 near Nagykapornak, Zala County** — a notoriously dangerous stretch with a documented history of stability-loss accidents in adverse weather conditions.

---

## Key Results

| Segment | Nominal min μ | RL min μ | Safety gain |
|---|---|---|---|
| kappa_s1 | 0.403 | 0.343 | **14.9%** |
| delta_s2 | 0.466 | 0.366 | **21.5%** |
| he_s1 | 0.481 | 0.361 | **24.9%** |

At challenge friction levels, the RL-adaptive controller reduces:
- Peak cross-track error by up to **81.9%** (kappa_s1)
- Peak body slip angle by up to **65.3%** (kappa_s1, 13.3° vs 38.3° baseline)

---

## System Architecture

```
Vehicle State ──► Observation Vector (16D) ──► SAC Agent ──► Action (4D)
                                                                    │
                                                          ┌─────────▼──────────┐
                                                          │  MPC Parameter     │
                                                          │  Adjustment        │
                                                          │  Δw_cte, Δw_he,   │
                                                          │  Δw_δ, Δv_target  │
                                                          └─────────┬──────────┘
                                                                    │
Road Reference ──► MPC Controller ◄──────────────────────────────────
                        │
                        ▼
                   Vehicle Dynamics (Pacejka Magic Formula, 4-wheel, 10 states)
```

---

## Repository Structure

```
rl-oversteer-mpc-road76/
│
├── simulation/
│   ├── MAIN_VEHICLE_MODEL.slx          # Main Simulink model
│   ├── sim_setup.m                     # Workspace initialiser
│   ├── vehicle_dynamics_sfun.m         # 4-wheel dynamics S-Function
│   ├── mpc_vehicle_controller.m        # MPC path-tracking S-Function
│   ├── sector_switch_sfun.m            # Challenge zone manager
│   ├── rl_observation_sfun.m           # 16-dim observation builder
│   ├── rl_reward_sfun.m                # Physics-based reward function
│   ├── rl_toolbox_sac_setup.m          # SAC agent configuration
│   ├── rl_fullroad_trainer.m           # Curriculum training script
│   └── run_vulnerability_pipeline.m   # Road vulnerability scanner
│
├── evaluation/
│   ├── rl_test_agent.m                 # RL vs baseline comparison
│   ├── rl_visual_test.m                # Visual simulation test
│   └── generate_thesis_figures.m      # Generates all thesis figures
│
├── road_data/
│   └── road76_waypoints.mat            # 1276 GPS-derived waypoints
│
├── results/
│   ├── rl_toolbox_agents/
│   │   └── sac_fullroad.mat            # Trained SAC agent
│   └── vulnerability_results.mat      # Road vulnerability analysis
│
├── thesis_figures/                     # All generated PNG figures
│
└── README.md
```

---

## Requirements

- MATLAB R2023b or later
- Simulink
- MATLAB Reinforcement Learning Toolbox
- MATLAB Optimization Toolbox (for MPC)

---

## Quick Start

```matlab
% 1. Clone or download this repository and open MATLAB
% 2. Navigate to the repository root folder
% 3. Run setup:
sim_setup

% 4. Run a simulation with the trained RL agent:
sim('MAIN_VEHICLE_MODEL')

% 5. Compare RL agent vs baseline:
rl_test_agent

% 6. Generate all thesis figures:
generate_thesis_figures
```

---

## Training From Scratch

```matlab
% 1. Set up workspace
sim_setup

% 2. Configure and initialise SAC agent
rl_toolbox_sac_setup

% 3. Run curriculum training on all segments (~8 hours)
rl_fullroad_trainer

% 4. Trained agent saved to:
% rl_toolbox_agents/sac_fullroad.mat
```

---

## Vehicle Model Parameters

| Parameter | Value | Unit |
|---|---|---|
| Vehicle mass m | 1650 | kg |
| Yaw inertia Iz | 2900 | kg·m² |
| Wheelbase L | 2.8 | m |
| CG height hCG | 0.54 | m |
| Pacejka B/C/E | 8.0 / 1.6 / −0.5 | — |
| Background friction | 0.70 | — |

---

## Road Environment

- **Road:** Route 76, Zala County, Hungary (near Nagykapornak)
- **Length:** ~2.5 km
- **Waypoints:** 1276 GPS-derived centreline points
- **Vulnerable segments:** 5 identified (kappa_s1, delta_s2, cte_s2, cte_s3, he_s1)
- **Data source:** OpenStreetMap (OSM)

---

## Citation

If you use this code or methodology in your research, please cite:

```
A. Ahmadov, "Reinforcement Learning Based Oversteer Control Under 
Dynamically Changing Adhesion," MSc thesis, Budapest University of 
Technology and Economics, Budapest, Hungary, 2026.
```

---

## Supervisor

**Tóth Szilárd Hunor**
Department of Automotive Technologies
Budapest University of Technology and Economics

---

## License

This repository is made available for academic and research purposes.
For commercial use, please contact the author.
