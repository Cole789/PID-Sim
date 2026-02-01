#pragma once
#ifndef SYSTEMS_HPP
#define SYSTEMS_HPP
#include <string>

class BaseSystem {
public:
    virtual ~BaseSystem() {}
    // all systems must implement these two:
    virtual double update(double control_signal, double dt) = 0;
    virtual std::string getName() const = 0;
};
#endif