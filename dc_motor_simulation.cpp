/*
 * DC Motor Dynamic Simulation and Steady-State Validation
 *
 * Engineering model:
 *   L * di/dt     = Va - R*i - Ke*omega
 *   J * domega/dt = Kt*i - b*omega - Tload
 *
 * State variables:
 *   i      : armature current [A]
 *   omega  : angular velocity [rad/s]
 *
 * Numerical method:
 *   Classical fourth-order Runge-Kutta (RK4), fixed time step.
 *
 * Build:
 *   g++ -std=c++17 -O2 -Wall -Wextra -pedantic dc_motor_simulation.cpp -o dc_motor_simulation
 *
 * Run:
 *   ./dc_motor_simulation
 *
 * Output:
 *   dc_motor_results.csv
 *
 * This is an educational, idealised model. It neglects magnetic saturation,
 * brush voltage drop, temperature dependence, and nonlinear friction.
 */

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

struct MotorParameters {
    double armature_resistance_ohm = 2.0;       // R
    double armature_inductance_h = 0.5;         // L
    double torque_constant_nm_per_a = 0.1;      // Kt
    double back_emf_constant_v_per_rad_s = 0.1; // Ke
    double rotor_inertia_kg_m2 = 0.02;          // J
    double viscous_friction_nm_s = 0.002;       // b
};

struct State {
    double current_a;
    double speed_rad_s;
};

struct Derivative {
    double d_current_dt;
    double d_speed_dt;
};

constexpr double PI = 3.14159265358979323846;

double appliedVoltage(double /*time_s*/) {
    // A 24 V step is applied at t = 0.
    return 24.0;
}

double loadTorque(double time_s) {
    // A load disturbance is applied at t = 1.5 s.
    return (time_s < 1.5) ? 0.05 : 0.08; // N.m
}

Derivative motorDerivative(
    double time_s,
    const State& state,
    const MotorParameters& p
) {
    const double voltage = appliedVoltage(time_s);
    const double load = loadTorque(time_s);

    const double di_dt =
        (voltage
         - p.armature_resistance_ohm * state.current_a
         - p.back_emf_constant_v_per_rad_s * state.speed_rad_s)
        / p.armature_inductance_h;

    const double domega_dt =
        (p.torque_constant_nm_per_a * state.current_a
         - p.viscous_friction_nm_s * state.speed_rad_s
         - load)
        / p.rotor_inertia_kg_m2;

    return {di_dt, domega_dt};
}

State addScaled(const State& state, const Derivative& derivative, double scale) {
    return {
        state.current_a + scale * derivative.d_current_dt,
        state.speed_rad_s + scale * derivative.d_speed_dt
    };
}

State rk4Step(
    double time_s,
    double dt,
    const State& state,
    const MotorParameters& p
) {
    const Derivative k1 = motorDerivative(time_s, state, p);
    const Derivative k2 = motorDerivative(
        time_s + 0.5 * dt, addScaled(state, k1, 0.5 * dt), p
    );
    const Derivative k3 = motorDerivative(
        time_s + 0.5 * dt, addScaled(state, k2, 0.5 * dt), p
    );
    const Derivative k4 = motorDerivative(
        time_s + dt, addScaled(state, k3, dt), p
    );

    State next{
        state.current_a + (dt / 6.0) *
            (k1.d_current_dt + 2.0 * k2.d_current_dt
             + 2.0 * k3.d_current_dt + k4.d_current_dt),
        state.speed_rad_s + (dt / 6.0) *
            (k1.d_speed_dt + 2.0 * k2.d_speed_dt
             + 2.0 * k3.d_speed_dt + k4.d_speed_dt)
    };

    if (!std::isfinite(next.current_a) || !std::isfinite(next.speed_rad_s)) {
        throw std::runtime_error("Simulation became numerically unstable.");
    }

    return next;
}

double analyticalSteadyStateSpeed(
    double voltage,
    double load_torque_nm,
    const MotorParameters& p
) {
    // At steady state:
    // i = (Va - Ke*omega)/R
    // 0 = Kt*i - b*omega - Tload
    const double numerator =
        (p.torque_constant_nm_per_a * voltage
         / p.armature_resistance_ohm) - load_torque_nm;

    const double denominator =
        p.viscous_friction_nm_s
        + (p.torque_constant_nm_per_a
           * p.back_emf_constant_v_per_rad_s
           / p.armature_resistance_ohm);

    return numerator / denominator;
}

int main() {
    try {
        const MotorParameters parameters{};

        constexpr double time_step_s = 0.001;
        constexpr double simulation_duration_s = 20.0;
        constexpr int number_of_steps =
            static_cast<int>(simulation_duration_s / time_step_s);

        State state{0.0, 0.0}; // Motor starts from rest.

        std::ofstream csv("dc_motor_results.csv");
        if (!csv) {
            std::cerr << "Error: could not create dc_motor_results.csv\n";
            return 1;
        }

        csv << "time_s,voltage_v,load_torque_nm,current_a,"
               "speed_rad_s,speed_rpm,electromagnetic_torque_nm\n";
        csv << std::fixed << std::setprecision(8);

        for (int step = 0; step <= number_of_steps; ++step) {
            const double time_s = step * time_step_s;
            const double voltage = appliedVoltage(time_s);
            const double load = loadTorque(time_s);
            const double electromagnetic_torque =
                parameters.torque_constant_nm_per_a * state.current_a;
            const double speed_rpm = state.speed_rad_s * 60.0 / (2.0 * PI);

            csv << time_s << ','
                << voltage << ','
                << load << ','
                << state.current_a << ','
                << state.speed_rad_s << ','
                << speed_rpm << ','
                << electromagnetic_torque << '\n';

            if (step < number_of_steps) {
                state = rk4Step(time_s, time_step_s, state, parameters);
            }
        }

        csv.close();

        const double final_load = loadTorque(simulation_duration_s);
        const double theoretical_speed = analyticalSteadyStateSpeed(
            appliedVoltage(simulation_duration_s), final_load, parameters
        );
        const double simulated_speed = state.speed_rad_s;
        const double absolute_error = std::abs(
            simulated_speed - theoretical_speed
        );
        const double percentage_error =
            (std::abs(theoretical_speed) > 1e-12)
            ? 100.0 * absolute_error / std::abs(theoretical_speed)
            : 0.0;

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "DC Motor Dynamic Simulation completed.\n";
        std::cout << "Results written to: dc_motor_results.csv\n\n";
        std::cout << "Final simulated speed: "
                  << simulated_speed << " rad/s ("
                  << simulated_speed * 60.0 / (2.0 * PI) << " rpm)\n";
        std::cout << "Analytical steady-state speed at final load: "
                  << theoretical_speed << " rad/s\n";
        std::cout << "Absolute speed difference: "
                  << absolute_error << " rad/s\n";
        std::cout << "Relative difference: "
                  << percentage_error << " %\n";

        if (percentage_error > 2.0) {
            std::cout
                << "\nNote: the simulation may not have fully settled by "
                   "the selected end time. Increase simulation_duration_s "
                   "to check convergence.\n";
        } else {
            std::cout
                << "\nThe final simulated speed is close to the analytical "
                   "steady-state reference.\n";
        }

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Simulation error: " << error.what() << '\n';
        return 1;
    }
}
