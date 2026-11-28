#!/usr/bin/env python3
"""检查 ROCK 4D 与 OpenRF1 的串口和编码器，可在架空后单轮点动。"""
# 来源：ROCK4D_OpenRF1_通信配置指南_20261122/04_ROCK配置与测试/serial_test.py
import argparse
import fcntl
import json
import math
import os
import select
import sys
import termios
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tasks.ball_collect.openrf1 import (
    c26_wheel_rpm, check_factory_car_link, counter_delta, encoder_counts, open_serial, read_c26_encoders,
)


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

def exchange(fd, request, timeout, expected=None):
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
            raw, buffer = buffer.split(b'\n', 1)
            reply = raw.rstrip(b'\r').decode('ascii')
            if expected is None or reply == expected:
                print(f'{request} -> {reply}', flush=True)
                return reply
    raise TimeoutError(f'{request!r} 回复超时，收到 {buffer!r}')


def check_board(fd, timeout, jog):
    def ask(command, expected=None):
        return exchange(fd, command, timeout, expected)

    ask('PING', 'PONG')
    version = ask('VER')
    if version == 'RX:VER':
        if jog:
            raise RuntimeError('回显固件仅支持通信检查')
        ask('HELLO_123', 'RX:HELLO_123')
        return
    if version not in ('COLLECT1', 'ERR'):
        raise RuntimeError(f'无法识别的固件回复 {version!r}')
    before = encoder_counts(ask('ENC').split())
    if not jog:
        return
    if version != 'ERR':
        raise RuntimeError('JOG 需要四轮点动固件，当前为 COLLECT1')
    wheels = 'ABCD' if jog == 'all' else jog
    for wheel in wheels:
        if ask('JOG ' + wheel) != 'DONE':
            raise RuntimeError('点动未执行或该通道已用过，本次检查结束')
        time.sleep(0.25)  # 等车轮惯性滑行后再比较原始计数。
        after = encoder_counts(ask('ENC').split())
        delta = [counter_delta(a, b) for a, b in zip(before, after)]
        print('编码器增量', ' '.join(f'{ch}={d:+d}' for ch, d in zip('ABCD', delta)), flush=True)
        active = [ch for ch, d in zip('ABCD', delta) if abs(d) >= 3]
        if active != [wheel]:
            raise RuntimeError(f'预期只有 {wheel} 有计数，请核对供电、轮位及编码器')
        before = after


def check_c26_encoders(port, timeout, samples, counts_per_rev):
    link = open_serial(port, timeout)
    try:
        link.reset_input_buffer()
        previous = None
        for index in range(samples):
            if index:
                time.sleep(0.1)
            current = read_c26_encoders(link)
            rpm = None if previous is None else c26_wheel_rpm(previous, current, counts_per_rev)
            print(json.dumps(dict(sample=current[0], counts_abcd=current[1], rpm_abcd=rpm)), flush=True)
            previous = current
    finally:
        link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyS4')
    parser.add_argument('--timeout', type=float, default=3.0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--jog', choices=['A', 'B', 'C', 'D', 'all'],
                      help='底盘架空后使用，每个选中通道点动约 0.2 秒')
    mode.add_argument('--factory-car', action='store_true',
                      help='只发送厂商全零轮速指令并检查 cmdOk')
    mode.add_argument('--c26-encoders', type=int, nargs='?', const=10, metavar='SAMPLES',
                      help='只发送 $ENC! 查询 C26 反馈补丁，默认查询 10 次')
    parser.add_argument('--counts-per-rev', type=int, default=4680,
                        help='C26 编码器模式的轮端每圈计数（默认 4680）')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive')
    if args.c26_encoders is not None:
        if args.c26_encoders < 2 or args.counts_per_rev <= 0:
            parser.error('C26 查询次数至少为 2，每圈计数必须为正整数')
        try:
            check_c26_encoders(args.port, args.timeout, args.c26_encoders, args.counts_per_rev)
        except (OSError, RuntimeError, ValueError, ImportError) as error:
            print(f'FAIL: {error}', file=sys.stderr)
            return 1
        return 0
    if args.factory_car:
        try:
            check_factory_car_link(args.port, args.timeout)
        except (OSError, RuntimeError, ImportError) as error:
            print(f'FAIL: {error}', file=sys.stderr)
            return 1
        print('PASS: $Car:0,0,0,0! -> cmdOk', flush=True)
        return 0
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
        check_board(fd, args.timeout, args.jog)
        print('PASS', flush=True)
        return 0
    except (OSError, RuntimeError, ValueError, termios.error) as error:
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
