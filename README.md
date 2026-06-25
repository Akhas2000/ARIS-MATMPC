# Orientation-Aware Control and Trajectory Design for Aerial RIS-Assisted Wireless Communications

This document describes the installation procedure, project organization, and execution of the main simulation scripts.

---

# 0. Installation

This project is built on top of **MATMPC**.

## Step 1 — Install MATMPC

Download and install **MATMPC** by following the official instructions:

<https://github.com/chenyutao36/MATMPC>

Verify that MATMPC runs correctly before proceeding.

---

## Step 2 — Install CasADi

MATMPC requires **CasADi**.

Install a CasADi version compatible with your MATMPC release and add it to the MATLAB path.

You can verify the installation by running:

```matlab
import casadi.*

x = MX.sym('x');
disp(x)
```

If MATLAB reports an error, manually add CasADi to the MATLAB path:

```matlab
addpath('/path/to/casadi')
```

---

## Step 3 — Copy the Project

Download this repository and copy all files and folders from the main folder into the MATMPC directory. This will overwrite the existing "examples folder", as well as other files and folders such as "InitData, Simulation, Draw", and others in the MATMPC directory.



Open MATLAB inside the project directory and add the project functions:

```matlab
addpath(genpath('Functions'))
```

The project is now ready to run.

---

# 1. Quick Launch

## Generate the UAV Model (First Execution Only)

Before running any simulation, generate the symbolic UAV model by executing:

```matlab
Model_Generation
```

During the generation process, MATLAB will ask for confirmation twice.

Answer:

```text
Yes
Yes
```

This step compiles the symbolic dynamics into the MEX functions used by MATMPC.

Model generation only needs to be executed once, or whenever the UAV model contained in

```text
examples/GTMR_4_com_soft.m
```

is modified.

---

## Run a Single HoT Simulation

To execute only the **Horizontal Orientation Tracking (HoT)** benchmark:

```matlab
Simulation
```

The script automatically:

- initializes the simulation,
- runs the closed-loop controller,
- generates the corresponding plots through `Draw.m`.

---

## Run All Benchmark Simulations

Execute:

```matlab
Main
```

This script sequentially runs the benchmark scenarios:

- **NoT** (No Tracking)
- **HoT** (Horizontal Orientation Tracking)
- **SL** (Straight-Line)

For each benchmark, the required initialization is performed automatically before launching the simulation.

At the end of execution,

```matlab
Draw_Main
```

is called automatically to generate all comparison figures.

---

## Scaling Analysis

The scalability studies are executed independently through:

```matlab
Main_Scale_K_User
```

or

```matlab
Main_Scale_M
```

Unlike the benchmark simulations, these scripts automatically perform model generation internally whenever required.

No manual execution of

```matlab
Model_Generation
```

is necessary.

### Main_Scale_K_User

Evaluates the influence of the number of users (**K**) by:

- regenerating the communication model for different values of **K**,
- executing the corresponding simulations,
- averaging the obtained results.

### Main_Scale_M

Evaluates the influence of the number of RIS elements (**M**) using the same procedure while varying the RIS array size.

---

## Reference Trajectory Evaluation

### `Main_Ref_Traj`

This script evaluates the reference trajectory generation algorithm proposed in **Algorithm 1** of the paper.

It compares two utility functions used during graph search:

- **Proposed Proxy Utility** for several values of:
  - \(p = 2\)
  - \(p = 5\)
  - \(p = 8\)

  using the closed-form beamformer and RIS phase shifts derived in the paper.

- **Communication Rate** used directly as the utility function.

The generated trajectories and their corresponding communication performance are displayed for comparison.

---

### `Main_Ref_Djikstra_Vs_PSCA`

This script compares:

- the reference trajectory obtained using the proposed **Proxy Utility** (with \(p = 8\)), using the closed-form beamformer and RIS phase shifts derived in the paper,

against

- a trajectory generated using the **PSCA** optimization algorithm jointly optimizing:
  - trajectory,
  - beamforming,
  - RIS phase shifts.

The communication performance of both reference trajectories is then compared.

---

# 2. Project Structure

```text
ARIS_Main/
│
├── Main.m
├── Simulation.m
├── Main_Ref_Traj.m
├── Main_Ref_Djikstra_Vs_PSCA.m
├── Main_Scale_K_User.m
├── Main_Scale_M.m
│
├── Draw.m
├── Draw_Main.m
│
├── Model_Generation.m
│
├── examples/
├── Functions/
```

---

## examples/

```text
examples/
│
├── GTMR_4_com_soft.m
├── GTMR_4_com_soft_Scale.m
```

The **examples** folder contains the symbolic UAV model used by MATMPC.

The main model file is:

```text
GTMR_4_com_soft.m
```

It defines:

- system states,
- control inputs,
- nonlinear UAV dynamics,
- quaternion kinematics,
- communication-related dynamics.

Any modification of this file requires running:

```matlab
Model_Generation
```

before launching a simulation.

The folder additionally contains:

```text
GTMR_4_com_soft_Scale.m
```

which is the model used to conduct the scalability analysis.

---

## Functions/

This directory contains all project-specific functions organized into four main modules.

### Communication_Functions

Implements the RIS-assisted communication model.

Main routines include:

- communication environment generation (`Setup.m`),
- base station array response (`array_response_phases_BS.m`),
- RIS phase response (`phase_array_response_RIS.m`),
- communication rate evaluation (`Rates_No_Complex_phase.m`),
- sum-rate computation along a path (`computeTotalRateAlongPath.m`),
- aperture gain computation (`compute_aperture_gain_from_rotation.m`).

---

### MATMPC_Functions

Contains the interface between the project and MATMPC.

These functions:

- initialize the NMPC problem and configure constraints and cost functions (`InitData_Main.m`, `InitData_Scale.m`),
- generate the model for scalability analysis (`Model_Generation_Scale.m`),
- execute the closed-loop NMPC simulations (`Simulation_Main.m`, `Simulation_Scale.m`, `Main_Scale.m`).

---

### Ref_Generation_PSCA

Contains the PSCA reference generation functions.

These functions implement:

- PSCA to determine the optimal hovering position, RIS phase shift matrix, and beamformer (`PSCA_Optimal_Position.m`),
- PSCA to determine the reference trajectory, beamformer, and RIS phase shifts (`PSCA_REF_NoReg.m`),
- densification of the PSCA trajectory to match the NMPC time grid (`PSCA_Interpolate.m`).

---

### Robotic_Functions

Contains trajectory planning and geometric utilities.

These functions implement:

- Algorithm 1 reference trajectory generation (`optimalTrajectoryPU.m`, `optimalTrajectoryPU_Astar.m`, or `optimalTrajectoryRandomKNN.m`),
- quaternion geodesic distance computation (`quaternion_geodesic_distance.m`).

Together, these functions generate the reference trajectories used by the NMPC controller and by the trajectory comparison scripts.
