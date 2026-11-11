# DC Motor Dynamic Simulation and Steady-State Validation (C++)

## Overview
A self-contained C++17 simulation of a separately excited DC motor using a
two-state electromechanical model. The program applies a 24 V input and then
introduces a step change in load torque. It numerically solves the motor
differential equations with the classical fourth-order Runge-Kutta (RK4)
method and exports the time history to CSV.

## Engineering problem
How do armature current and rotor speed evolve after a voltage is applied,
and how does a change in mechanical load affect the motor's response?

## Mathematical model
The armature circuit is represented by:

`L (di/dt) = Va - R i - Ke omega`

The rotor dynamics are represented by:

`J (domega/dt) = Kt i - b omega - Tload`

Where:
- `Va`: armature voltage (V)
- `i`: armature current (A)
- `R`: armature resistance (ohm)
- `L`: armature inductance (H)
- `Ke`: back-EMF constant (V/(rad/s))
- `Kt`: torque constant (N.m/A)
- `omega`: angular speed (rad/s)
- `J`: rotor inertia (kg.m^2)
- `b`: viscous friction coefficient (N.m.s)
- `Tload`: external load torque (N.m)

For the final constant voltage and load, the analytical steady-state speed is
calculated from the steady-state form of the same equations. This provides an
independent reference for checking the numerical result.

## Numerical method
The program uses fixed-step classical RK4 integration with a time step of
0.001 s over a 20 s simulation. RK4 evaluates the state derivatives four times
per step to obtain a higher-order estimate than forward Euler integration.

## Scenario and default parameters
- Armature voltage: 24 V
- Load torque: 0.05 N.m before 1.5 s
- Load torque: 0.08 N.m from 1.5 s onward
- Armature resistance: 2 ohm
- Armature inductance: 0.5 H
- Torque constant: 0.1 N.m/A
- Back-EMF constant: 0.1 V/(rad/s)
- Rotor inertia: 0.02 kg.m^2
- Viscous friction: 0.002 N.m.s
- Initial current and speed: zero
- Simulation duration: 20 s

These are illustrative parameters, not specifications for a particular commercial
motor. Update them when modelling a real motor with datasheet values.

## Build and run
Requires a C++17-compatible compiler such as GCC or Clang.

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic dc_motor_simulation.cpp -o dc_motor_simulation
./dc_motor_simulation
```

On Windows with MinGW, run the resulting executable as:

```powershell
.\dc_motor_simulation.exe
```

## Output
The program creates `dc_motor_results.csv` with:
- time
- applied voltage
- load torque
- armature current
- angular speed (rad/s)
- angular speed (rpm)
- electromagnetic torque

The CSV can be opened in Excel or plotted using Python, MATLAB, or another
data-analysis tool.

## Validation approach
At the end of the simulation, the program compares the simulated speed with
the analytical steady-state speed for the final voltage and load torque.
The absolute and relative differences are printed to the terminal.

For a stronger validation study:
1. Increase the simulation duration and confirm the final speed converges.
2. Repeat with smaller time steps (for example, 0.0005 s and 0.00025 s).
3. Compare the final values and quantify the change as the step size decreases.
4. Check current and speed against independently calculated reference cases.

## Limitations and future improvements
This first version assumes constant motor parameters and linear viscous friction.
It does not model magnetic saturation, brush voltage drop, temperature effects,
Coulomb friction, or a speed controller. Useful extensions include:
- a PI speed controller with actuator saturation and anti-windup;
- parameter sweeps for inertia, load torque, and supply voltage;
- automated regression tests;
- convergence analysis across multiple integration step sizes;
- plots of speed, current, and torque;
- comparison with measured motor data or a trusted simulation package.

## Reproducibility and integrity
All equations, default parameters, integration settings, and output columns are
documented here and in the source file. The model is an educational simulation;
do not describe it as validated against a physical motor until it has been
compared with measured data or a trusted reference model.
