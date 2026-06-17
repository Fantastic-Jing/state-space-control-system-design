# State-Space Control Design

This repository contains the implementation, theoretical analysis, and Simulink simulations for advanced control systems design, focusing on hydraulic and mechanical multi-mass plants.

## Project Structure

* **3-Tank Hydraulic System**: State-space representation, multi-input multi-output (MIMO) configuration, linearized system analysis, and disturbance/leakage simulation.
* **Torsional Spring-Mass-Damper System**: Mechanical modeling of three rotating masses, controllability/observability analysis, and full-state feedback controller design via pole placement with prefilters.

## Technical Stack
* MATLAB (State-Space modeling, Matrix Operations, Eigenvalue Analysis)
* Simulink (Dynamic Simulation, Custom Integrator-Gain Topology for State-Space Structures)

## Key Concepts Covered
* 99% and 2% Settling Time ($T_s$) estimation based on dominant poles
* Structural properties analysis (`ctrb`, `obsv`)
* Multi-mass torsional dynamic differential equations
* Closed-loop pole placement with zero steady-state error (Prefilter $V$ design)