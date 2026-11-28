#!/usr/bin/env python3
"""由现场操作员控制起止的一米直线测速。"""
import argparse
import select
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tasks.ball_collect.openrf1 import FACTORY_STOP, open_serial


def send(link, command):
    link.write(command)
    reply = link.read_until(size=5)
    if reply != b"cmdOk":
        raise RuntimeError(f"指令应答异常 {reply!r}")


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="ch340")
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("请在现场可操作的终端运行")
    if input("确认跑道净空、有可立即断电的人员。输入 START 启动 0.10 m/s 指令：") != "START":
        return
    link = open_serial(args.port)
    try:
        link.reset_input_buffer()
        send(link, FACTORY_STOP)
        print("测量开始，车头越过 1 米线立即按回车，60 秒后自动发停车。", flush=True)
        start = time.monotonic()
        try:
            send(link, b"$Car:0.1,0.1,0.1,0.1!")
            reached = bool(select.select([sys.stdin], [], [], max(0, start + 60 - time.monotonic()))[0])
            elapsed = time.monotonic() - start
            if reached:
                reached = bool(sys.stdin.readline())
        finally:
            try:
                send(link, FACTORY_STOP)
            except (OSError, RuntimeError):
                sys.stderr.write("停车指令未确认，请立即切断整车电源。\n")
                raise
        if reached:
            print(f"用时 {elapsed:.2f} s，平均速度 {1/elapsed:.4f} m/s。")
        else:
            print("已结束，本次没有记录到越线按键。")
    finally:
        link.close()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupt)
    try:
        main()
    except KeyboardInterrupt:
        pass
