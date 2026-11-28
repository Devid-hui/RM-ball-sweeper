"""OpenRF1 四轮速度指令。串口上的轮序是 A、B、C、D。"""
from binascii import crc_hqx
from math import isfinite, remainder, tau
from time import monotonic


WHEEL_ORDER = ("fl", "fr", "rl", "rr")
LEGACY_CHANNEL_ORDER = ("rl", "fr", "fl", "rr")
# 必须与 COLLECT1 中的 encoder_sign 一致。
ENCODER_SIGN_ABCD = (-1, 1, -1, 1)
FACTORY_STOP = b"$Car:0,0,0,0!"
C26_SAMPLE_PERIOD_S = 0.020


def factory_car_command(wheels_rad_s, wheel_radius_m, channel_order=WHEEL_ORDER, *,
                        drive_sign_abcd=(1, 1, 1, 1), command_scale=1.0):
    """把运动学输出的四轮角速度换成厂商协议的轮端线速度。"""
    values = []
    for wheel, sign in zip(channel_order, drive_sign_abcd):
        speed = wheels_rad_s[WHEEL_ORDER.index(wheel)] * wheel_radius_m * sign * command_scale
        if not isfinite(speed):
            raise ValueError("厂商轮速指令要求有限数值")
        text = f"{speed:.4f}".rstrip("0").rstrip(".")
        values.append("0" if text == "-0" else text)
    return "$Car:" + ",".join(values) + "!"


def check_factory_car_link(device, timeout=0.15):
    link = open_serial(device, timeout)
    try:
        link.reset_input_buffer()
        send_factory_car(link, FACTORY_STOP)
    finally:
        link.close()


def send_factory_car(link, command):
    link.write(command)
    # 实板的 cmdOk 没有换行，读满 5 字节即可结束。
    reply = link.read_until(size=5)
    if reply != b"cmdOk":
        raise RuntimeError(f"C26 轮速指令期望 cmdOk，实际收到 {reply!r}")


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


def counter_delta(before, after, bits=16):
    half_range = 1 << (bits - 1)
    return (after - before + half_range) % (2 * half_range) - half_range


def read_c26_encoders(link):
    """读取 C26 V2.2 补丁的采样号和四路原始累计计数。"""
    link.write(b"$ENC!")
    reply = link.read_until(size=80)
    if not reply.endswith(b"\n"):
        raise TimeoutError(f"C26 $ENC! 回复超时，收到 {reply!r}，请检查是否已烧录反馈补丁")
    parts = reply.split()
    fields = parts[1:]
    if parts[:1] != [b"ENC"] or len(fields) != 5 or any(
        not field.isdigit() or len(field) > 10 or int(field) >= 2**32 for field in fields
    ):
        raise ValueError(f"C26 编码器回复无效 {reply!r}")
    sample, *counts = map(int, fields)
    return sample, counts


def c26_encoder_interval(before, after):
    old_sample, old_counts = before
    new_sample, new_counts = after
    samples = (new_sample - old_sample) % 2**32
    if samples == 0 and old_counts != new_counts:
        raise ValueError("C26 同一采样号的编码器计数不一致")
    # 诊断每秒至少查一次。允许 5 秒抖动，拒绝重启或过长的采样间隔。
    elapsed = samples * C26_SAMPLE_PERIOD_S
    if elapsed > 5:
        raise ValueError("C26 采样号跳变，请检查控制板是否重启并重新查询")
    deltas = [counter_delta(a, b, 32) for a, b in zip(old_counts, new_counts)]
    return deltas, elapsed


def c26_wheel_rpm(before, after, counts_per_rev):
    """按 MCU 采样时间换算有符号轮端转速，保留编码器原始方向。"""
    deltas, elapsed = c26_encoder_interval(before, after)
    if elapsed == 0:
        return None
    scale = 60 / (counts_per_rev * elapsed)
    return [delta * scale for delta in deltas]


class WheelHeading:
    """墙标给出绝对航向，编码器补上标签转出视野后的转角。"""

    def __init__(self, model, counts_per_rev, max_wheel_rad_s, channel_order=LEGACY_CHANNEL_ORDER, *,
                 c26=False, encoder_sign_abcd=ENCODER_SIGN_ABCD):
        self.model = model
        self.radians_per_count = tau / counts_per_rev
        self.max_counts_s = max_wheel_rad_s / self.radians_per_count
        self.wheel_indices = tuple(WHEEL_ORDER.index(wheel) for wheel in channel_order)
        self.c26 = c26
        self.encoder_sign_abcd = encoder_sign_abcd
        self.previous = None
        self.sampled_at = None
        self.yaw = None

    def update(self, counts, tag_yaw, sampled_at):
        if self.previous is not None:
            if self.c26:
                delta, elapsed = c26_encoder_interval((self.sampled_at, self.previous), (sampled_at, counts))
            else:
                delta = [counter_delta(a, b) for a, b in zip(self.previous, counts)]
                elapsed = sampled_at - self.sampled_at
            # MCU 每 20 ms 采样。留出两个采样间隔和轮速波动，拒绝跳变或不确定的回绕。
            limit = 2 + 2 * self.max_counts_s * (elapsed + 0.04)
            if elapsed < 0 or (not self.c26 and limit >= 32768) or max(map(abs, delta)) > limit:
                raise ValueError("编码器计数跳变或采样间隔过长，停车")
            if tag_yaw is None and self.yaw is not None:
                angles = [0.0] * 4
                for index, value, sign in zip(self.wheel_indices, delta, self.encoder_sign_abcd):
                    angles[index] = value * sign * self.radians_per_count
                self.yaw = remainder(self.yaw + self.model.forward(angles)[2], tau)
        if tag_yaw is not None:
            self.yaw = tag_yaw
        self.previous, self.sampled_at = counts, sampled_at
        return self.yaw


def open_serial(device, timeout=0.15):
    if device == "ch340":
        from tasks.ball_collect.ch340 import Ch340Serial

        return Ch340Serial(timeout)
    import serial

    return serial.Serial(device, 115200, timeout=timeout, write_timeout=0.15, exclusive=True)


class C26:
    def __init__(self, device, config):
        self.config = config
        self.sample = None
        self.last_progress = None
        self.serial = open_serial(device)
        try:
            self.serial.reset_input_buffer()
            send_factory_car(self.serial, FACTORY_STOP)
        except BaseException:
            self.serial.close()
            raise

    def read_feedback(self):
        sample, counts = read_c26_encoders(self.serial)
        now = monotonic()
        if sample != self.sample:
            self.sample, self.last_progress = sample, now
        if now - self.last_progress >= 0.15:
            raise TimeoutError("C26 编码器采样号持续未更新，停车")
        return counts, sample

    def send(self, wheels):
        send_factory_car(self.serial, c26_command(wheels, self.config).encode("ascii"))

    def close(self):
        try:
            self.serial.write(FACTORY_STOP)
        finally:
            self.serial.close()


def c26_command(wheels, config):
    return factory_car_command(
        wheels, config["wheel_radius_m"], config["wheel_order_abcd"],
        drive_sign_abcd=config["drive_sign_abcd"], command_scale=config["c26_command_scale"],
    )


class OpenRF1:
    def __init__(self, device, counts_per_rev, drive_sign_abcd, channel_order=LEGACY_CHANNEL_ORDER):
        self.counts_per_rev = counts_per_rev
        self.wheel_indices = tuple(WHEEL_ORDER.index(wheel) for wheel in channel_order)
        self.serial = open_serial(device)
        self.sequence = 0
        try:
            self.serial.reset_input_buffer()
            version = self.exchange(b"VER\n")
            if version != ["COLLECT1"]:
                raise RuntimeError(f"OpenRF1 需要采球固件 COLLECT1，收到 {version}")
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
            command = payload.split()[0].decode("ascii")
            raise TimeoutError(f"OpenRF1 {command} 回复超时")
        return line.decode("ascii").split()

    def read_counts(self):
        return encoder_counts(self.exchange(b"ENC\n"))

    def read_feedback(self):
        counts = self.read_counts()
        return counts, monotonic()

    def send(self, wheels):
        # 单位为每 20 ms 的千分之一编码器计数，计数以向前滚动为正。
        scale = self.counts_per_rev * 20 / tau
        targets = [round(wheels[index] * scale) for index in self.wheel_indices]
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
