#pragma once

#include <stdint.h>

enum { WHEEL_COUNT = 4, CONTROL_PERIOD_MS = 20, COMMAND_TIMEOUT_MS = 500 };

typedef struct {
    uint16_t counts[WHEEL_COUNT];
    int32_t target[WHEEL_COUNT];
    int32_t integral[WHEEL_COUNT];
    int duty[WHEEL_COUNT];
    int drive_sign[WHEEL_COUNT];
    uint32_t last_command;
} Collector;

void collector_tick(Collector *controller, uint32_t now, const uint16_t counts[WHEEL_COUNT]);
void collector_line(Collector *controller, uint32_t now, const char *line,
                    int motion_port, void (*send)(char));
