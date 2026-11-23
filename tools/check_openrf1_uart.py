#!/usr/bin/env python3
"""检查 ROCK 4D 与 OpenRF1 UART2 测试固件的请求和回复。"""
# 来源：ROCK4D_OpenRF1_通信配置指南_20261122/04_ROCK配置与测试/serial_test.py
import argparse
import fcntl
import math
import os
import select
import sys
import termios
import time

def configure(fd):
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)

def exchange(fd, request, expected, timeout):
    payload = (request + '\n').encode('ascii')
    deadline = time.monotonic() + timeout
    while payload:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError('Transmit timed out')
        try:
            count = os.write(fd, payload)
            payload = payload[count:]
        except BlockingIOError:
            continue
    buffer = b''
    observed = []
    while time.monotonic() < deadline:
        if not select.select([fd], [], [], max(0, deadline-time.monotonic()))[0]:
            break
        try:
            chunk = os.read(fd, 256)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        buffer += chunk
        if len(buffer) > 4096:
            raise RuntimeError('Unexpected continuous data on the selected port')
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            line = line.rstrip(b'\r')
            if line == expected.encode('ascii'):
                print(f'PASS {request!r} -> {expected!r}', flush=True)
                return
            observed.append(repr(line))
            if len(observed) > 32:
                raise RuntimeError('Too many unrelated response lines')
    raise TimeoutError(f'{request!r}: expected {expected!r}; received {observed}, partial={buffer!r}')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyS4')
    parser.add_argument('--timeout', type=float, default=3.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive')
    fd = None
    previous = None
    exclusive = False
    try:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        previous = termios.tcgetattr(fd)
        configure(fd)
        print(f'Testing {args.port}: 115200 8N1, no flow control', flush=True)
        exchange(fd, 'PING', 'PONG', args.timeout)
        exchange(fd, 'HELLO_123', 'RX:HELLO_123', args.timeout)
        print('PASS: bidirectional UART communication verified', flush=True)
        return 0
    except (OSError, RuntimeError, termios.error) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        if fd is not None:
            if previous is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSANOW, previous)
                except (OSError, termios.error):
                    pass
            if exclusive:
                try:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                except OSError:
                    pass
            os.close(fd)

if __name__ == '__main__':
    raise SystemExit(main())
