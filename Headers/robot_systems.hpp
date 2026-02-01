#pragma once
#ifndef ROBOT_SYSTEMS_HPP
#define ROBOT_SYSTEMS_HPP
#include "systems.hpp"
#include <cmath>

// can simulate linear forward/backward movement and point rotation.
class LinearRobot : public BaseSystem {
        double pos = 0, vel = 0;
    public:
        double update(double force, double dt) override {
            double acc = force; // simple mass=1
            vel += acc * dt;
            pos += vel * dt;
            return pos;
        }
        std::string getName() const override { return "Linear Robot"; }
};

// simple 1D quadrotor altitude model with linear drag and ground clamp
class QuadrotorAltitude : public BaseSystem {
        double altitude = 0.0;
        double verticalVelocity = 0.0;

        const double g = 9.81;
        const double damping = 0.2;
    public:
        double update(double thrust, double dt) override {
            double acc = thrust - g - damping * verticalVelocity; // thrust counteracts gravity and drag
            verticalVelocity += acc * dt;
            altitude += verticalVelocity * dt;

            if (altitude < 0.0) { // prevent sinking below ground
                altitude = 0.0;
                verticalVelocity = 0.0;
            }

            return altitude;
        }
        std::string getName() const override { return "Quadrotor Altitude"; }
};

class Pendulum : public BaseSystem {
        double angle = 10.0; // start tilted
        double ang_vel = 0;

        const double g = 9.81;
        const double length = 1.0;
        const double damping = 0.15; // higher = thicker air/more friction
    public:
        double update(double torque, double dt) override {
            // gravity component
            double gravity_torque = -(g / length) * std::sin(angle);
            // air resistance
            double damping_torque = -damping * ang_vel;
            // total acceleration
            double acc = torque + gravity_torque + damping_torque;
            // integration
            ang_vel += acc * dt;
            angle += ang_vel * dt;

            return angle;
        }
        std::string getName() const override { return "Pendulum"; }
};

// More systems will be created later
#endif