"""OpenRF1 四轮速度指令。串口上的轮序是 A、B、C、D。"""
from binascii import crc_hqx
from math import remainder, tau


def checked_line(command):
    payload = command.encode("ascii")
    return payload + f" {crc_hqx(payload, 0xffff)}\n".encode("ascii")


def encoder_counts(reply):
    if len(reply) != 4:
        raise ValueError(f"编码器回复缺少四路计数 {reply}")
    counts = []
    for channel, field in zip("ABCD", reply):
        name, separator, value = field.partition("=")
        if name != channel or not separator or not value.isascii() or not value.isdecimal() or int(value) > 65535:
            raise ValueError(f"编码器回复无效 {reply}")
        counts.append(int(value))
    return counts


def counter_delta(before, after):
    return (after - before + 32768) % 65536 - 32768


class WheelHeading:
    """墙标给出绝对航向，编码器补上标签转出视野后的转角。"""

    def __init__(self, model, counts_per_rev, max_wheel_rad_s):
        self.model = model
        self.radians_per_count = tau / counts_per_rev
        self.max_counts_s = max_wheel_rad_s / self.radians_per_count
        self.previous = None
        self.sampled_at = None
        self.yaw = None

    def update(self, counts, tag_yaw, sampled_at):
        if self.previous is not None:
            delta = [counter_delta(a, b) for a, b in zip(self.previous, counts)]
            elapsed = sampled_at - self.sampled_at
            # MCU 每 20 ms 采样。留出两个采样间隔和轮速波动，拒绝跳变或不确定的回绕。
            limit = 2 + 2 * self.max_counts_s * (elapsed + 0.04)
            if elapsed < 0 or limit >= 32768 or max(map(abs, delta)) > limit:
                raise ValueError("编码器计数跳变或采样间隔过长，停车")
            if tag_yaw is None and self.yaw is not None:
                a, b, c, d = delta
                angles = [value * self.radians_per_count for value in (-c, b, -a, d)]
                self.yaw = remainder(self.yaw + self.model.forward(angles)[2], tau)
        if tag_yaw is not None:
            self.yaw = tag_yaw
        self.previous, self.sampled_at = counts, sampled_at
        return self.yaw


class OpenRF1:
    def __init__(self, device, counts_per_rev, drive_sign_abcd):
        import serial
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

    def read_counts(self):
        return encoder_counts(self.exchange(b"ENC\n"))

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
