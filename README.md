# Global HELP — UAV Model & Communication (Focus on `Main.m`)

_Last updated: 2025-10-08 14:46_

This document covers the **UAV model**, **trajectory utilities**, **RIS-assisted communication & rate computation**, and the **main simulation flow**.  
Solver internals are omitted **except** the two entry points used by `Main.m`: `InitData_Main.m` and `Simulation_Main.m`.

---

## 1) Fast Run — Executing the Full Scenario

```matlab
% In MATLAB, set folder to: ARIS_Main/ARIS_Main
addpath(genpath('Functions'));   % utilities for path, channel, rates, plotting
Main                            % runs the scenario, saves results, and plots
```

**Outputs**
- Results file: `HoT_NoT_uMRAV_Both.mat`
- Figures: generated automatically by `Draw_Main.m`

---

## 2) Project Arborescence & One‑Line Explanations (UAV + Communication)

```
ARIS_Main/  — Project root (UAV + Communication focused)
  CMoN_Init.m                               — General script/function
  Draw.m                                   — Plot the results of the Simulation file 
  Draw_Main.m                              — Compares NoT vs HoT runs; main plotting entry
  InitData.m                               — Initializes simulation input/data from settings
  InitData_ngrid.m                         — Grid-based data initialization
  InitMemory.m                             — Prepares workspace memory structures
  Initialization_Simulink.m                — Simulink environment initialization
  Main.m                                   — Central scenario script (defines, runs, plots)
  Model_Generation.m                       — Creates environment and geometric setup
  Simulation.m                             — High-level UAV + communication simulation wrapper
  qqs.m                                   — Auxiliary computation script
  video_Maker.m                            — Generates animation/video of UAV trajectory
  Functions/                               — Reusable components
    Communication_Functions/               — Communication & RIS utilities
      Rates_No_Complex_phase.m             — Per-step user rate computation
      Setup_env.m                          — Defines frequency, power, bandwidth, path-loss
      array_response_phases_BS.m           — BS steering vector generation
      computeTotalRateAlongPath.m          — Integrates rate along UAV path
      compute_aperture_gain_from_rotation.m— Computes gain from UAV orientation
      phase_array_response_RIS.m           — RIS per-element phase response (Rx/Tx)
    MATMPC_Functions/                      — Entry points used by Main.m
      InitData_Main.m                      — Builds MATMPC-ready input/data from settings, path & weights
      Simulation_Main.m                    — Runs the closed-loop simulation with horizon/weights/path
    Robotic_Functions/                     — UAV trajectory & kinematic utilities
      densifyToNs.m                        — Resamples path to `Ns` points (matches simulation steps)
      smoothPath.m                         — Smooths UAV path to eliminate sharp turns
      optimalTrajectoryPU.m                — Path planning maximizing proxy utility (rate proxy)
      optimalTrajectoryRandomKNN.m         — Randomized KNN path planning variant
      quaternion_geodesic_distance.m       — Attitude distance on SO(3)
  data/                                   — Input and result data files
  doc/                                    — Documentation
  examples/                               — Usage examples / reference models
    GTMR_4_com_soft.m                      — Example UAV+Comm model
  mex_core/                               — MEX compilation utilities
    Compile_Mex.m                          — Example compilation helper
  model_src/                              — Model source definitions
```

---

## 3) Main Workflow — Where Each Piece Is Used (`Main.m`)

1. **Initialization & Path**
   - Adds `Functions/` to MATLAB path.
   - Sets timing: `Ts_st`, `Tf_init`, `T_stabilization` and `Ns = Tf_init/Ts_st`.
   - Builds UAV **proxy‑utility path**, then:
     ```matlab
     pathXY_ProxyUtility = densifyToNs(pathXY_ProxyUtility, Ns);
     pathXY_ProxyUtility = smoothPath(pathXY_ProxyUtility, 150);
     ```

2. **Prepare Input/Data (MATMPC_Functions)**
   - `InitData_Main.m` constructs MATMPC‑ready I/O from current settings, weights & path:
     ```matlab
     [input, data] = InitData_Main(settings, Ns, ...
         q_sv, q_p, q_v, q_eul, q_omega, h_UAV, pathXY_ProxyUtility);
     ```

3. **Closed‑Loop Simulation (MATMPC_Functions)**
   - `Simulation_Main.m` executes simulation for each mode (NoT / HoT) and returns trajectories:
     ```matlab
     [controls_*, state_*, time_*, data_*] = Simulation_Main( ...
         settings, opt, N, Ns, q_sv, q_p, q_v, q_eul, q_omega, ...
         h_UAV, pathXY_ProxyUtility, Tf_init, T_stabilization);
     ```

4. **Save & Plot**
   - Saves to `.mat` and calls `Draw_Main` to produce figures.

---

## 4) Function Cards — Concise Purpose & Signature

### A) **Functions/MATMPC_Functions/**
- **InitData_Main.m** — _Builds MATMPC‑ready input/data from settings, weights & path_  
  **Signature:**  
  ```matlab
  [input, data] = InitData_Main(settings, Ns, q_sv, q_p, q_v, q_eul, q_omega, h_UAV, pathXY)
  ```

- **Simulation_Main.m** — _Runs closed‑loop simulation (given horizon, weights, and path)_  
  **Signature:**  
  ```matlab
  [controls_MPC, state_sim, time, data, ...] = Simulation_Main( ...
      settings, opt, N, Ns, q_sv, q_p, q_v, q_eul, q_omega, ...
      h_UAV, pathXY_ProxyUtility, Tf_init, T_stabilization)
  ```

### B) **Functions/Robotic_Functions/**

- **optimalTrajectoryPU.m** — _Path planning maximizing a proxy‑utility (rate proxy)_  
- **optimalTrajectoryRandomKNN.m** — _Randomized KNN path planning variant_  
- **quaternion_geodesic_distance.m** — _Attitude distance on SO(3)_

### C) **Functions/Communication_Functions/**
- **Setup_env.m** — _Defines `freq`, `Power`, `Bandwidth`, `beta_B`, `beta_R`_  
- **array_response_phases_BS.m** — _BS phase‑only steering vector_  
- **phase_array_response_RIS.m** — _RIS per‑element phase response (Rx/Tx)_  
- **Rates_No_Complex_phase.m** — _Per‑step user rate per UAV pose_  
- **computeTotalRateAlongPath.m** — _Integrates rates along a full path_

**Core evaluation example (without the MPC loop):**
```matlab
[totalRate, ratesVec] = computeTotalRateAlongPath(pathXY, h_UAV, ...
    pA, pR, pK, R, Power, Bandwidth, freq, beta_B, beta_R);
```

---

## 5) Key Parameters in `Main.m` (for quick edits)

- **Timing:** `Tf_init`, `T_stabilization`, `Ts_st`  ⇒ `Ns = Tf_init/Ts_st`  
- **Geometry:** `h_UAV`, path generator (PU vs KNN), smoothing window (e.g., 150)  
- **Comms:** `freq`, `beta_B`, `beta_R`, `Power`, `Bandwidth`  
- **RIS/BS:** size of `pR`/`pA`, RIS rotation `R`  
- **Weights:** `q_eul`, `q_omega`, `q_p`, `q_v`, `q_sv`  (NoT vs HoT modes)

---

This HELP file intentionally stays focused on the **UAV + communication** logic and the **exact functions** `Main.m` relies on — including the two `MATMPC_Functions` you requested.


---

## 6) Additional Notes on Model Generation and Simulation

- **examples/GTMR_4_com_soft.m** — contains the **quadrotor model** used in this project.  
  It defines the UAV states, control inputs, dynamics, and output equations.

- **Model_Generation.m** — must be **executed before running `Main.m` or `Simulation.m`**.  
  It loads the chosen UAV model (e.g., GTMR_4_com_soft) and **generates the symbolic model** required for the simulations.

- **Simulation.m** — corresponds to the **original MATMPC Simulation script**, with only a **reference trajectory section added** for **tracking experiments**.  
  It has been modified to integrate your **UAV trajectory and RIS-assisted communication** logic.

- **video_Maker.m**  **Generates animation/video of UAV trajectory of the UAV** simulated in **Simulation.m**

---
