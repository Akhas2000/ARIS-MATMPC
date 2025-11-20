# ARIS_Main Integrated With MATMPC

> UAV-RIS Simulation Framework using CasADi + MATMPC + ARIS

---

## Overview

This documentation explains:

- How **CasADi**, **MATMPC**, and **ARIS_Main** work together
- The **correct installation procedure**
- What files must be **replaced vs kept**
- How **Simulation_Main**, **Simulation_Scale**, and their Init functions connect
- Which scripts require **Model_Generation** and which do **not**

---

## Correct Folder Architecture

> **Very Important**: MATMPC must contain **all executable components**

```
MATMPC/
├── casadi/                    # Place CasADi here
├── core/
├── examples/                  # Replace with ARIS examples only
├── Functions/                 # Move ARIS Functions folder here
├── InitData_Main.m           # ARIS version replaces built-in one
├── Simulation_Main.m         # ARIS version replaces built-in one
├── Model_Generation.m        # ARIS version replaces built-in one
├── Draw.m                    # ARIS version replaces built-in one
└── [ARIS extra files...]
```

### Critical Integration Rules

Your **ARIS_Main folder MUST NOT be copied inside MATMPC as a folder.**

Instead:

- The **Functions** folder must be placed inside `MATMPC/Functions/`

- The **examples** inside MATMPC should be replaced by the ARIS project **examples** folder

- All other ARIS `.m` files must be placed directly in the MATMPC root

If any file already exists (Simulation, Model_Generation, Init, Draw):
→ **Replace it with the ARIS version**

---

## How CasADi + MATMPC + ARIS Interact

There are **three simulation engines**, each with its own Init + Model + Simulation logic.

### Main Simulation Engine (Full MPC Simulation)

**Files involved:**
- `Main.m`
- `Simulation_Main.m`
- `InitData_Main.m`
- `Model_Generation.m`

#### 1 Main.m
Top-level script. Calls `Simulation_Main.m`.

#### Simulation_Main.m
Runs the **MPC loop**.

Calls:
- `InitData_Main` ← Initialization
- CasADi ← Symbolic functions
- MATMPC solver ← Optimization at each step

#### InitData_Main.m
Used **only** in `Simulation_Main`.

Creates:
- MPC weights
- UAV initial state 
- Reference path
- All solver options


#### Model_Generation.m
Used **before Main.m**

Creates:
- UAV CasADi SX dynamics
- Jacobians
- Integrators
- Predictor functions

### 2 Scaling Simulation Engine

They allow for the selection of the number of RIS elements, the number of antenna elements, and the number of Users:
- `Main_Scale_M.m`: Scaling simulation on the number of RIS elements
- `Main_Scale_K_User.m`: Scaling simulation on the number of users
- `Main_Scale_Main.m` call both `Main_Scale_M.m` and `Main_Scale_K_User.m` 

**Files involved:**
- `Simulation_Scale.m`
- `InitData_Scale.m`
- `Model_Generation_Scale.m`

#### Model_Generation_Scale.m
Runs first. Creates a **CasADi model**.

#### InitData_Scale.m
Initializes for scaling cases (variable M or K).

#### Simulation_Scale.m
Executes simulation but without full MPC complexity.

>  **Important**: Scaling simulations **do NOT require** the full `Model_Generation.m`. They use the embedded model created by `Model_Generation_Scale`.

---

## Communication Functions

Located in `Functions/Communication_Functions/`

| Function | Description |
|----------|-------------|
| `Setup_env.m` | Creates the environments RIS, users and antenna coordinates |
| `array_response_phases_BS.m` | Compute the transmit array response of the BS |
| `phase_array_response_RIS.m` | Compute the phase of the array response of RIS (transmit or received) |
| `compute_aperture_gain_from_rotation.m` | Compute the aperture of RIS |
| `Rates_No_Complex_phase.m` | Compute the rate of all users in the network |
| `computeTotalRateAlongPath.m` | Compute the sumrate across a path |

These handle:
- RIS phase modeling
- BS steering vectors
- Channel/RIS gain modeling
- Rate computation
- Environment parameters

---

## Correct Installation Procedure

### STEP 1 — Install CasADi inside MATMPC

1. Download CasADi for MATLAB from: https://web.casadi.org/get/
2. Extract it
3. Place the extracted folder in: `MATMPC/casadi/`
4. Add to MATLAB path:
   ```matlab
   addpath(genpath('MATMPC/casadi'));
   ```

### STEP 2 — Replace the MATMPC example folder

1. Delete: `MATMPC/examples/`
2. Replace it by: `ARIS_Main/examples/`
   → Rename ARIS example folder to **examples** and copy it into MATMPC

### STEP 3 — Replace the MATMPC Functions folder

1. Delete: `MATMPC/Functions/`
2. Replace with ARIS version: `ARIS_Main/Functions/`

### STEP 4 — Copy ARIS core files to the MATMPC root folder

These files must overwrite any MATMPC versions:

- `Simulation_Main.m`
- `Simulation_Scale.m`
- `Model_Generation.m`
- `Model_Generation_Scale.m`
- `InitData_Main.m`
- `InitData_Scale.m`
- `Draw.m`
- `Draw_Main.m`
- Any other ARIS `.m` file

Every file with the same name should be **replaced**.

---

## Running Simulations

### 1 Full MPC Simulation (Main Engine)

Run model generation:
```matlab
Model_Generation
```

Then run simulation:
```matlab
Main
```

This performs:
- UAV model initialization
- MPC loop
- HoT/NoT switching
- Rate computation
- Plotting

### 2 Scaling Simulations

**No model generation needed**, because model is already embedded.

Run:
```matlab
Main_Scale_M
Main_Scale_K_User
Main_Scale_Main
```

These scripts:
- Call `Model_Generation_Scale` internally
- Then run `Simulation_Scale`
- Do not require full dynamics
- Are optimized for performance

---

## Summary Table

| Engine | Needs Model Generation? | Simulation Function | Init Function | Model |
|--------|------------------------|-------------------|---------------|-------|
| **Main Simulation** | YES (`Model_Generation`) | `Simulation_Main` | `InitData_Main` | Full dynamics |
| **Scale vs M/K** | NO | `Simulation_Scale` | `InitData_Scale` | Embedded model |

---

