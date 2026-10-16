# PSO-Based Fractional-Order PID Control of a DC Motor

## Overview
This project models and simulates DC motor speed regulation using a conventional PID controller, a manually tuned Fractional-Order PID (FOPID) controller, and a Particle Swarm Optimization (PSO)-tuned FOPID controller.

The system is implemented using MATLAB and Simulink. Oustaloup recursive approximation is used to represent fractional-order operators in a realizable rational form.

## Objectives
- Develop a DC motor transfer-function model.
- Compare PID and FOPID speed-control performance.
- Optimize FOPID parameters using Particle Swarm Optimization.
- Evaluate closed-loop stability and load-torque disturbance response.
- Validate the optimized controller in Simulink.

## DC Motor Model
The motor transfer function is:

\[
G(s)=\frac{0.1}{0.01s^2+0.14s+0.41}
\]

The FOPID controller is represented as:

\[
C(s)=K_p+\frac{K_i}{s^\lambda}+K_d s^\mu
\]

## Tools and Methods
- MATLAB
- Simulink
- Particle Swarm Optimization (PSO)
- Oustaloup recursive approximation
- Step-response and disturbance-response analysis

## Simulation Results

| Controller | Rise Time (s) | Settling Time (s) | Overshoot (%) |
|---|---:|---:|---:|
| Conventional PID | 0.38051 | 1.3397 | Approximately 0 |
| Manual FOPID | 0.20175 | 4.0676 | 0 |
| PSO-FOPID | 0.04270 | 0.38355 | 0.19371 |

For the optimized FOPID controller, the final tracking error in the reported simulation was approximately 0.0001462.

## Repository Contents
- `main_project.m` — motor model and baseline controller analysis
- `oustaloup.m` — fractional-order approximation
- `optimize_fopid.m` — PSO optimization and performance analysis
- `build_fopid_model.m` — Simulink model generation
- `analyze_fopid_model.m` — model analysis
- `.slx` files — Simulink models
- `.csv` files — numerical comparison and disturbance results
- `.png` files — generated response plots

## How to Run
1. Download or clone this repository.
2. Open MATLAB and set the repository folder as the current folder.
3. Ensure the required Control System Toolbox and Global Optimization Toolbox functionality is available.
4. Run `main_project.m` for baseline analysis.
5. Run `optimize_fopid.m` for PSO-based tuning.
6. Run `build_fopid_model.m` to generate the optimized Simulink model.

**Note:** Running the model-generation script recreates the `DC_Motor_FOPID_Auto.slx` file. Save a backup first if you have made manual changes to the model.

## Scope and Limitations
The results are based on a simulated motor model and a finite-order approximation of fractional operators. Hardware implementation and experimental validation have not been performed.

## Author
Shobhit Kumar,
Electrical Engineering Undergraduate
Sant Longowal Institute of Engineering and Technology (SLIET), Longowal, Punjab
