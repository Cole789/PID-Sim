#pragma once
#ifndef PID_HPP
#define PID_HPP
class PID {
public:
    // Proportional, Integral, and Derivative gains
    double kp, ki, kd;

    // state accumulated across updates
    double integral = 0;    // accumulates past error (I term)
    double last_error = 0;  // error from previous update (for D term)

    // initialize controller with chosen gains
    PID(double p, double i, double d) : kp(p), ki(i), kd(d) {}

    // computes the control output for one PID iteration
    double update(double setpoint, double current, double dt) {
        // error between desired target and current system state
        double error = setpoint - current;

        // integral term - sums error over time to eliminate steady-state offset
        integral += error * dt;

        // derivative term - predicts future trend by measuring error change rate
        double derivative = (error - last_error) / dt;

        // store error for next iteration
        last_error = error;

        // combined PID control output
        return (kp * error) + (ki * integral) + (kd * derivative);
    }
};

#endif