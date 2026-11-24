#include "collector_control.h"

enum { MAX_TARGET = 1000000, MAX_DUTY = 450, START_DUTY = 100 };
static const int encoder_sign[WHEEL_COUNT] = {-1, 1, -1, 1};

static int clamp(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void stop(Collector *controller)
{
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        controller->target[i] = 0;
        controller->integral[i] = 0;
        controller->duty[i] = 0;
    }
}

void collector_tick(Collector *controller, uint32_t now, const uint16_t counts[WHEEL_COUNT])
{
    if (now - controller->last_command >= COMMAND_TIMEOUT_MS) stop(controller);
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        int delta = (uint16_t)(counts[i] - controller->counts[i]);
        if (delta >= 32768) delta -= 65536;
        controller->counts[i] = counts[i];
        int target = controller->target[i];
        if (!target) continue;
        int direction = target > 0 ? 1 : -1;
        int error = direction * (target - 1000 * encoder_sign[i] * delta);
        int integral = clamp(controller->integral[i] + error / 3, -450000, 450000);
        int output = START_DUTY + error / 45 + integral / 1000;
        // Hold the integral when it would drive the output further into saturation.
        if ((output < MAX_DUTY || error < 0) && (output > 0 || error > 0)) {
            controller->integral[i] = integral;
        }
        controller->duty[i] = direction * controller->drive_sign[i] * clamp(output, 0, MAX_DUTY);
    }
}

static int equals(const char *a, const char *b)
{
    while (*a && *a == *b) { ++a; ++b; }
    return *a == *b;
}

static void text(void (*send)(char), const char *message)
{
    while (*message) send(*message++);
}

static void number(void (*send)(char), unsigned value)
{
    char digits[5];
    unsigned count = 0;
    do {
        digits[count++] = '0' + value % 10;
        value /= 10;
    } while (value);
    while (count) send(digits[--count]);
}

static int integer(const char **cursor, int *value)
{
    const char *p = *cursor;
    if (*p++ != ' ') return 0;
    int sign = 1;
    if (*p == '-') { sign = -1; ++p; }
    if (*p < '0' || *p > '9') return 0;
    int magnitude = 0;
    while (*p >= '0' && *p <= '9') {
        magnitude = magnitude * 10 + (*p++ - '0');
        if (magnitude > MAX_TARGET) return 0;
    }
    *cursor = p;
    *value = sign * magnitude;
    return 1;
}

static int checksum_ok(const char *start, const char *end)
{
    uint16_t crc = 0xffff;
    for (const char *p = start; p < end; ++p) {
        crc ^= (uint16_t)(uint8_t)*p << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc & 0x8000u) ? ((unsigned)crc << 1) ^ 0x1021u : (unsigned)crc << 1);
        }
    }
    int supplied;
    return integer(&end, &supplied) && *end == 0 && supplied == crc;
}

void collector_line(Collector *controller, uint32_t now, const char *line,
                    int motion_port, void (*send)(char))
{
    if (equals(line, "VER")) { text(send, "COLLECT1\n"); return; }
    if (equals(line, "PING")) { text(send, "PONG\n"); return; }
    if (equals(line, "STOP")) {
        stop(controller);
        text(send, "OK\n");
        return;
    }
    if (equals(line, "ENC")) {
        for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
            if (i) send(' ');
            send('A' + i);
            send('=');
            number(send, controller->counts[i]);
        }
        send('\n');
        return;
    }
    const char *cursor = line + 1;
    const int configure = *line == 'D';
    const int speed = *line == 'W';
    int sequence = 0;
    int values[WHEEL_COUNT];
    int valid = motion_port && (configure || speed);
    if (valid && speed) valid = integer(&cursor, &sequence) && sequence >= 0 && sequence <= 65535;
    for (unsigned i = 0; valid && i < WHEEL_COUNT; ++i) {
        valid = integer(&cursor, &values[i]);
        if (configure) valid = valid && (values[i] == -1 || values[i] == 1);
        if (speed) valid = valid && controller->drive_sign[i] != 0;
    }
    if (!valid || !checksum_ok(line, cursor)) {
        if (motion_port) stop(controller);
        text(send, "ERR\n");
        return;
    }
    if (configure) {
        stop(controller);
        for (unsigned i = 0; i < WHEEL_COUNT; ++i) controller->drive_sign[i] = values[i];
        text(send, "OK\n");
        return;
    }
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        int previous = controller->target[i];
        controller->target[i] = values[i];
        if (!values[i] || !previous || (values[i] > 0) != (previous > 0)) {
            controller->integral[i] = 0;
            controller->duty[i] = 0;
        }
    }
    controller->last_command = now;
    text(send, "W ");
    number(send, (unsigned)sequence);
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        send(' ');
        number(send, controller->counts[i]);
    }
    send('\n');
}
