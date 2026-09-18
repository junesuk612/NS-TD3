# NS-TD3: Null-Space TD3 for Free-Floating Space Manipulators

> **Learning-Enhanced Trajectory Planning for Free-Floating Space Manipulators under Inertia Changes**
> Junesuk Kim — *IFAC World Congress 2026 (submitted)*

A **TD3 reinforcement learning** agent that learns the **1-D null-space parameter α(t)** of a 7-DOF free-floating space manipulator, coordinating end-effector tracking with base attitude preservation under **payload-induced inertia changes**.

---

## Demo

![NS-TD3 demo](media/NS-TD3.mp4)

*Full-length video: [`media/NS-TD3.mp4`](media/NS-TD3.mp4). The arm docks a payload to a station port and returns home while the base attitude is regulated by the learned null-space policy.*

---

## Highlights

- **Hierarchical control** — TD3 (1 Hz) sets the null-space scalar Δα; a differential IK layer (10 Hz) enforces joint limits.
- **Handles inertia changes** — Payload docking shifts the inertia coupling matrix `B = Hbb⁻¹·Hba`. The learned policy adapts to the post-docking dynamics without retuning.
- **1-D action, high leverage** — 7-DOF arm tracking a 6-DOF EE ⇒ null-space dimension = 1. The learned α(t) trades off arm-energy vs. base-recoil objectives that would otherwise conflict.
- **Analytical baselines built in** — `α*_A` (energy-optimal) and `α*_B` (recoil-optimal) are computed in closed form each step and included in the observation vector.

---

## Method

```
TD3 agent  ── Δα ─▶  Null-space IK ── q̇ ─▶  Free-floating dynamics
    ▲                                              │
    └────────  29-D observation, reward  ◀─────────┘
```

**Action** — scalar Δα ∈ [-1, 1] (residual on null-space parameter)
**Observation (29-D)** — EE error (3) + base quat (4) + base ω (3) + base v (3) + joint q (7) + joint q̇ (7) + prev action (1) + docked flag (1)
**Reward** — `r_track + r_recoil + r_base_ramp(t) + r_smooth + r_alive`
Base-attitude penalty ramps up during the homing phase (t = 160–260 s) to prioritize tracking early and base stability late.

See the paper for the full formulation: [`docs/IFAC_Junesuk_Kim_Final.pdf`](docs/IFAC_Junesuk_Kim_Final.pdf).

---

## Requirements

- **MATLAB R2024b or newer**
- Reinforcement Learning Toolbox
- Robotics System Toolbox
- Optimization Toolbox
- Simulink

---

## Quick Start

```matlab
% Add all subfolders to the path
addpath(genpath(pwd));

% (A) Baseline — pure IK, no RL
>> baseline_sim

% (B) Train the TD3 agent from scratch (~hours)
>> RL_main

% (C) Load the pre-trained agent and evaluate
>> load("agents/good_agent.mat", "agent")
>> % then run the evaluation block at the bottom of RL_main.m

% (D) Visualize results
>> plot_results(out)
```

Set the target docking port and initial payload location at the top of [`matlab/RL_main.m`](matlab/RL_main.m):

```matlab
port_num       = 10;  % target station port (1–10)
sat_docked_num = 6;   % initial satellite / payload port
```

---

## Repository Structure

```
.
├── matlab/
│   ├── RL_main.m                 % TD3 training entry point
│   ├── baseline_sim.m            % pure-IK baseline (no RL)
│   ├── compare_baselines.m       % α*_A / α*_B / RL comparison
│   ├── plot_results.m            % Figures 1–6
│   ├── concept3_simulink_RL.slx  % Simulink plant + reward
│   ├── concept3_for_sim.mat      % rigidBodyTree (Concept3)
│   ├── Concept3.urdf             % URDF definition
│   ├── IK_sub/                   % proposed null-space IK (System objects)
│   ├── IK_baseline/              % RH-QP / offline / iter α baselines
│   ├── RL_sub/                   % Simulink MATLAB Function blocks (obs, reward)
│   └── meshes/                   % URDF visual meshes
├── agents/
│   └── good_agent.mat            % best trained TD3 agent
├── media/
│   └── NS-TD3.mp4                % demo video (proposed method)
├── docs/
│   └── IFAC_Junesuk_Kim_Final.pdf
├── LICENSE
└── README.md
```

---

## Robot Model — Concept3

A **7-DOF revolute arm** mounted on a **free-floating satellite base**. The base has no ground reaction; arm motion transfers linear and angular momentum to the base (dynamic coupling). Ten docking ports are distributed around the station; `AB2port()` maps port index → 6-DOF pose. Payload inertia enters `Hbb` after docking, shifting the coupling matrix `B` that the learned policy must adapt to.

---

## Citation

```bibtex
@inproceedings{kim2026nstd3,
  title  = {Learning-Enhanced Trajectory Planning for Free-Floating Space Manipulators under Inertia Changes},
  author = {Kim, Junesuk},
  booktitle = {Proc. IFAC World Congress},
  year   = {2026},
  note   = {Submission 2667}
}
```

---

## Author

**Junesuk Kim**
M.S. Candidate, Aerospace Engineering, Seoul National University
✉️ junesuk61212@gmail.com

## License

MIT — see [`LICENSE`](LICENSE).
