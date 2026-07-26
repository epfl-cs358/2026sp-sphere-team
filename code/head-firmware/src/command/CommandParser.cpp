#include "CommandParser.h"
#include <cstring>
#include <cstdlib>
#include <cerrno>

int parseServoCommand(const char* msg) {
    if (!msg || msg[0] == '\0') return -1;

    static const char PREFIX[] = "tilt:";
    static const size_t PREFIX_LEN = sizeof(PREFIX) - 1;

    if (strncmp(msg, PREFIX, PREFIX_LEN) != 0) return -1;

    const char* numStart = msg + PREFIX_LEN;
    if (*numStart == '\0') return -1;

    char* end;
    errno = 0;
    long val = strtol(numStart, &end, 10);
    if (end == numStart || errno != 0) return -1;

    return static_cast<int>(val);
}
