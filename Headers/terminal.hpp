#pragma once
#ifndef TERMINAL_HPP
#define TERMINAL_HPP
#include <termios.h>
#include <unistd.h>
#include <iostream>

enum class Key { Up, Down, Enter, Other, X, A, S };

struct TerminalRawGuard {
    termios orig{};
    bool active = false;
    TerminalRawGuard() {
        if (tcgetattr(STDIN_FILENO, &orig) == 0) {
            termios raw = orig;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 1;
            raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                active = true;
            }
        }
    }
    ~TerminalRawGuard() {
        if (active) {
            tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        }
    }
};

Key ReadKey();
void SetTerminalToRawNonBlocking(termios& orig, termios& raw);
void RestoreTerminal(const termios& orig);
#endif