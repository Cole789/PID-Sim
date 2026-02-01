#include "terminal.hpp"
#include <unistd.h>

Key ReadKey() {
    char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return Key::Other;
    if (c == '\n' || c == '\r') return Key::Enter;
    if (c == '\033') {
        char seq[2];
        if (read(STDIN_FILENO, &seq[0], 1) != 1) return Key::Other;
        if (read(STDIN_FILENO, &seq[1], 1) != 1) return Key::Other;
        if (seq[0] == '[') {
            if (seq[1] == 'A') return Key::Up;
            if (seq[1] == 'B') return Key::Down;
        }
        return Key::Other;
    }
    if (c == 'x' || c == 'X') return Key::X;
    if (c == 'a' || c == 'A') return Key::A;
    if (c == 's' || c == 'S') return Key::S;
    if (c == 'w' || c == 'W') return Key::Up;
    if (c == 's' || c == 'S') return Key::Down;
    return Key::Other;
}

void SetTerminalToRawNonBlocking(termios& orig, termios& raw) {
    tcgetattr(STDIN_FILENO, &orig);
    raw = orig;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

void RestoreTerminal(const termios& orig) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig);
}
