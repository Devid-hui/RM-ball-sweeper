"""通过 libusb 使用 ROCK 4D 上的 CH340，串口参数固定为 115200 8N1。"""
from time import monotonic


class Ch340Serial:
    # CH340 的寄存器配置参考 Linux drivers/usb/serial/ch341.c。
    def __init__(self, timeout=0.15):
        from usb import core, util

        self.usb = util
        self.timeout = timeout
        self.timeout_error = core.USBTimeoutError
        self.buffer = bytearray()
        adapters = list(core.find(find_all=True, idVendor=0x1a86, idProduct=0x7523))
        if len(adapters) != 1:
            raise OSError(f"需连接一只 CH340，当前找到 {len(adapters)} 只")
        self.device = adapters[0]
        try:
            util.claim_interface(self.device, 0)
            self.configure()
        except BaseException:
            self.close()
            raise

    def configure(self):
        version = self.device.ctrl_transfer(0xC0, 0x5F, 0, 0, 2, timeout=500)
        if len(version) != 2 or version[0] < 0x30:
            raise OSError("此入口仅支持使用新式行控制寄存器的 CH340")
        self.control_write(0xA1, 0, 0)       # 初始化串口。
        self.control_write(0x9A, 0x1312, 0xCC83)  # 115200，启用短包。
        self.control_write(0x9A, 0x2518, 0xC3)    # 收发使能，8N1。
        self.control_write(0x9A, 0x2727, 0)       # 关闭硬件流控。

    def control_write(self, request, value, index):
        self.device.ctrl_transfer(0x40, request, value, index, b"", timeout=500)

    def reset_input_buffer(self):
        self.buffer.clear()
        for _ in range(8):
            try:
                self.device.read(0x82, 64, timeout=10)
            except self.timeout_error:
                return
        raise OSError("CH340 串口持续收到数据，无法清空")

    def write(self, payload):
        written = self.device.write(0x02, payload, timeout=150)
        if written != len(payload):
            raise OSError("CH340 串口写入不完整")
        return written

    def read_until(self, size=80):
        deadline = monotonic() + self.timeout
        while len(self.buffer) < size and b"\n" not in self.buffer:
            remaining = deadline - monotonic()
            if remaining <= 0:
                break
            try:
                self.buffer.extend(self.device.read(0x82, 64, timeout=max(1, int(remaining * 1000))))
            except self.timeout_error:
                break
        end = self.buffer.find(b"\n")
        length = min(size, len(self.buffer) if end < 0 else end + 1)
        reply = bytes(self.buffer[:length])
        del self.buffer[:length]
        return reply

    def close(self):
        self.usb.dispose_resources(self.device)
