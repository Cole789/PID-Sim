# PID-Sim

PID-Sim is a real-time, terminal-based PID controller simulator written in C++.
It allows interactive experimentation with closed-loop control systems by tuning PID gains live, switching physical models during execution, and visualizing system responses using Gnuplot.

The project is intentionally low-level and lightweight, focusing on control theory fundamentals, system modeling, and direct terminal I/O rather than heavy frameworks or GUI libraries.

---

## Features

- Interactive terminal UI
- Live PID gain adjustment during simulation
- Runtime switching between different physical systems
- Real-time plotting via Gnuplot
- Clean separation between controller logic and system physics

---

## Simulated Systems

- Linear Robot — Simple 1D position control
- Quadrotor Altitude — Vertical motion under thrust and gravity
- Pendulum — Motor-driven pendulum with gravity and damping

Each system shares a common interface and can be swapped without restarting the program.

---

## Controls

### During Simulation
- X — Stop simulation
- A — Adjust PID constants
- S — Switch simulation model

### Menu Navigation
- Up / Down arrows or W / S — Navigate
- Enter — Select

---

## Build Instructions
### Requirements
- C++17-compatible compiler
- POSIX-compatible system (Linux / WSL / macOS)
- Gnuplot installed and accessible from the command line

### Compile
```g++ -std=c++17 -Wall -Wextra -O2 *.cpp -o pidsim```
### Run
```./pidsim```


## Project Structure
```.
├── main.cpp # Program entry point
├── simulation.cpp/hpp # Simulation loop and system management
├── terminal.cpp/hpp # Raw terminal input and key handling
├── pid.hpp # PID controller implementation
├── robot_systems.hpp # Physical system models```


---

## Educational Goals

This project is designed to:
- Demonstrate practical PID control behavior
- Show how control systems interact with physical dynamics
- Encourage experimentation through live parameter tuning
- Reinforce low-level C++ and systems programming concepts


License

MIT License (or replace with your preferred license)
