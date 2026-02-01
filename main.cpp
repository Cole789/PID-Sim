#define _XOPEN_SOURCE 500
#include "pid.hpp"
#include "Headers/robot_systems.hpp"
#include "Headers/terminal.hpp"
#include "Headers/simulation.hpp"
#include <iostream>
#include <unistd.h>
#include <vector>

using namespace std;

int main() {
    system("clear");

    // launch gnuplot
    FILE* gp = popen("gnuplot", "w");
    if (!gp) {
        cerr << "Gnuplot not found!\n";
        return 1;
    }

    cout << "\n=== Select System ===\n";
    vector<string> options = {
        "Linear Robot",
        "Quadrotor Altitude",
        "Pendulum"
    };
    int choice = SelectSystem(options);

    double startVal, targetVal;

    cout << "=== Enter Points ===\n";
    cout << "Enter starting position: ";
    cin >> startVal;
    cout << "Enter target position: ";
    cin >> targetVal;

    double kP, kI, kD;

    cout << "\n=== Enter PID Constants ===\n";
    cout << "Enter new kP: "; cin >> kP;
    cout << "Enter new kI: "; cin >> kI;
    cout << "Enter new kD: "; cin >> kD;

    cout << "\nStop simulation with X\n"
        << "Adjust PID constants with A\n"
        << "Switch simulation model with S\n\n";

    // values for simulation
    double currentVal = startVal;
    double setpoint   = targetVal;
    double dt         = 0.1;
    const int rangeOfGraph = 300;
    double yMin = min(startVal, targetVal) - 10.0;
    double yMax = max(startVal, targetVal) + 10.0;

    // gnuplot config
    fprintf(gp, "set term wxt noraise close\n");
    fprintf(gp, "set term wxt noraise title 'PID-SIM'\n");
    fprintf(gp, "set grid\n");
    fprintf(gp, "set xrange [0:%d]\n", rangeOfGraph);
    fprintf(gp, "set yrange [%f:%f]\n", yMin, yMax);

    // create sim objects
    PID controller(kP, kI, kD);
    BaseSystem* system = MakeSystem(choice);
    vector<double> history;

    // two terminal modes [arrow navigation, key input]
    termios origTerm{}, rawTerm{};
    SetTerminalToRawNonBlocking(origTerm, rawTerm);

    // main loop
    bool running = true;
    while (running) {
        SimResult result = runSimulation(
            gp,
            system,
            controller,
            currentVal,
            setpoint,
            dt,
            yMin,
            yMax,
            history,
            rangeOfGraph
        );

        switch (result) {
            case SimResult::Stopped:
                running = false;
                break;

            case SimResult::AdjustPID:
                std::system("clear");
                AdjustPID(
                    controller,
                    kP, kI, kD,
                    system,
                    choice,
                    history,
                    startVal,
                    currentVal,
                    origTerm,
                    rawTerm
                );

                cout << "\nStop simulation with X\n"
                    << "Adjust PID constants with A\n"
                    << "Switch simulation model with S\n\n";

                break;

            case SimResult::SwitchSystem:
                std::system("clear");
                SwitchSystem(
                    system, 
                    choice, 
                    options, 
                    history, 
                    startVal, 
                    targetVal, 
                    currentVal, 
                    origTerm, 
                    rawTerm, 
                    controller, 
                    kP, kI, kD);

                cout << "\nStop simulation with X\n"
                    << "Adjust PID constants with A\n"
                    << "Switch simulation model with S\n\n";

                break;

            case SimResult::Completed:
                cout << "\nSimulation complete.\n";
                running = false;
                break;
        }
    }

    // cleanup
    RestoreTerminal(origTerm);
    delete system;
    fprintf(gp, "exit\n");
    pclose(gp);

    return 0;
}
