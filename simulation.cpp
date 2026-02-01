#include "Headers/simulation.hpp"
#include "Headers/terminal.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <limits>

using namespace std;

BaseSystem* MakeSystem(int choice) {
    switch (choice) {
        case 0: return new LinearRobot();
        case 1: return new QuadrotorAltitude();
        case 2: return new Pendulum();
        default: return new QuadrotorAltitude();
    }
}

int SelectSystem(const vector<string>& options) {
    TerminalRawGuard guard;
    size_t idx = 0;

    cout << "\nUse Up/Down arrows (or W/S) and Enter to choose:\n";
    while (true) {
        for (size_t i = 0; i < options.size(); i++)
            cout << (i == idx ? "> " : "  ") << options[i] << "\n";

        Key k = ReadKey();
        if (k == Key::Enter) break;
        if (k == Key::Up && idx > 0) idx--;
        else if (k == Key::Down && idx + 1 < options.size()) idx++;
        cout << "\033[" << options.size() << "A";
    }
    cout << "\nSelected: " << options[idx] << "\n\n";
    return static_cast<int>(idx);
}

void AdjustPID(PID& controller, double& kP, double& kI, double& kD, BaseSystem*& system, int choice, vector<double>& history, double startVal, double& currentVal, termios& origTerm, termios& rawTerm) {
    RestoreTerminal(origTerm);

    cout << "\n=== Adjust PID Constants ===\n";
    cout << "Current: kP=" << controller.kp << ", kI=" << controller.ki << ", kD=" << controller.kd << '\n';
    cout << "Enter new kP: "; cin >> kP;
    cout << "Enter new kI: "; cin >> kI;
    cout << "Enter new kD: "; cin >> kD;
    controller = PID(kP, kI, kD);

    delete system;
    system = MakeSystem(choice);
    history.clear();
    currentVal = startVal;

    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    SetTerminalToRawNonBlocking(origTerm, rawTerm);
}

void SwitchSystem(BaseSystem*& system, int& choice, const vector<string>& options, vector<double>& history, double& startVal, double& targetVal, double& currentVal, termios& origTerm, termios& rawTerm, PID& controller, double& kP, double& kI, double& kD) {
    RestoreTerminal(origTerm);
    delete system;

    cout << "\n=== Switch System ===\n";
    choice = SelectSystem(options);
    system = MakeSystem(choice);

    cout << "=== Enter Points ===\n";
    cout << "Enter starting position: "; cin >> startVal;
    cout << "Enter target position: "; cin >> targetVal;
    currentVal = startVal;

    cout << "\n=== Enter PID Constants ===\n";
    cout << "Enter new kP: "; cin >> kP;
    cout << "Enter new kI: "; cin >> kI;
    cout << "Enter new kD: "; cin >> kD;
    controller = PID(kP, kI, kD);

    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    history.clear();
    SetTerminalToRawNonBlocking(origTerm, rawTerm);
}

bool PlotHistory(FILE* gp, const vector<double>& history, const string& name, double setpoint) {
    if (fprintf(gp, "plot '-' with lines lw 2 title '%s', %f title 'Setpoint' lt 2\n", name.c_str(), setpoint) < 0)
        return false;
    for (size_t i = 0; i < history.size(); ++i)
        fprintf(gp, "%zu %f\n", i, history[i]);
    fprintf(gp, "e\n");
    return fflush(gp) != EOF;
}

SimResult runSimulation(FILE* gp, BaseSystem*& system, PID& controller, double& currentVal, double setpoint, double dt, double& yMin, double& yMax, vector<double>& history, int rangeOfGraph) {
    for (int i = 0; i < rangeOfGraph; i++) {
        Key k = ReadKey();
        if (k == Key::X) return SimResult::Stopped;
        if (k == Key::A) return SimResult::AdjustPID;
        if (k == Key::S) return SimResult::SwitchSystem;

        double control = controller.update(setpoint, currentVal, dt);
        currentVal = system->update(control, dt);

        bool rangeChanged = false;
        if (currentVal > yMax) { yMax = currentVal + 5.0; rangeChanged = true; }
        if (currentVal < yMin) { yMin = currentVal - 5.0; rangeChanged = true; }
        if (rangeChanged) fprintf(gp, "set yrange [%f:%f]\n", yMin, yMax);

        history.push_back(currentVal);
        if (!PlotHistory(gp, history, system->getName(), setpoint)) return SimResult::Stopped;

        this_thread::sleep_for(chrono::milliseconds(40));
    }
    return SimResult::Completed;
}
