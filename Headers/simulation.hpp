#pragma once
#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <vector>
#include <cstdio>
#include "pid.hpp"
#include "Headers/robot_systems.hpp"
#include "Headers/terminal.hpp"

void SetTerminalToRawNonBlocking(termios& orig, termios& raw);
void RestoreTerminal(const termios& orig);

int SelectSystem(const std::vector<std::string>& options);
BaseSystem* MakeSystem(int choice);

void AdjustPID(PID& controller, double& kP, double& kI, double& kD, BaseSystem*& system, int choice, std::vector<double>& history, double startVal, double& currentVal, termios& origTerm, termios& rawTerm);

void SwitchSystem(BaseSystem*& system, int& choice, const std::vector<std::string>& options, std::vector<double>& history, double& startVal, double& targetVal, double& currentVal, termios& origTerm, termios& rawTerm, PID& controller, double& kP, double& kI, double& kD);

bool PlotHistory(FILE* gp, const std::vector<double>& history, const std::string& name, double setpoint);

enum class SimResult { Completed, Stopped, AdjustPID, SwitchSystem };

SimResult runSimulation(FILE* gp, BaseSystem*& system, PID& controller, double& currentVal, double setpoint, double dt, double& yMin, double& yMax, std::vector<double>& history, int rangeOfGraph);
#endif