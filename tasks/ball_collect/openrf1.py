"""OpenRF1 四轮速度指令。串口上的轮序是 A、B、C、D。"""
from binascii import crc_hqx
from math import tau

import serial


def checked_line(command):
    payload = command.encode("ascii")
    return payload + f" {crc_hqx(payload, 0xffff)}\n".encode("ascii")


class OpenRF1:
    def __init__(self, device, counts_per_rev, drive_sign_abcd):
        self.counts_per_rev = counts_per_rev
        self.serial = serial.Serial(device, 115200, timeout=0.15, write_timeout=0.15, exclusive=True)
        self.sequence = 0
        try:
            self.serial.reset_input_buffer()
            if self.exchange(b"VER\n") != ["COLLECT1"]:
                raise RuntimeError("OpenRF1 需要采球固件 COLLECT1")
            command = "D " + " ".join(map(str, drive_sign_abcd))
            if self.exchange(checked_line(command)) != ["OK"]:
                raise RuntimeError("OpenRF1 拒绝轮向配置")
        except BaseException:
            self.serial.close()
            raise

    def exchange(self, payload):
        self.serial.write(payload)
        line = self.serial.read_until(size=80)
        if not line.endswith(b"\n"):
            raise TimeoutError("OpenRF1 回复超时")
        return line.decode("ascii").split()

    def send(self, wheels):
        # 单位为每 20 ms 的千分之一编码器计数，计数以向前滚动为正。
        fl, fr, rl, rr = wheels
        scale = self.counts_per_rev * 20 / tau
        targets = [round(speed * scale) for speed in (rl, fr, fl, rr)]
        self.sequence = (self.sequence + 1) % 65536
        command = f"W {self.sequence} " + " ".join(map(str, targets))
        reply = self.exchange(checked_line(command))
        if len(reply) != 6 or reply[:2] != ["W", str(self.sequence)]:
            raise RuntimeError(f"OpenRF1 拒绝轮速或反馈不匹配 {reply}")
        counts = list(map(int, reply[2:]))
        if not all(0 <= value <= 65535 for value in counts):
            raise RuntimeError("OpenRF1 编码器反馈超出范围")
        return counts

    def close(self):
        try:
            self.serial.write(b"STOP\n")
        finally:
            self.serial.close()
