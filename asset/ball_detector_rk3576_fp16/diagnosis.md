# RK3576 NPU 初始化诊断

2026-11-20 实测。系统为原有 Ubuntu。

- 默认 Linux 7.1.3 没有可供 RKNN 使用的 RKNPU 驱动。
- 临时启动 6.1.115-vendor-rk35xx 后，Wi-Fi 和 SSH 可用。
- RKNPU 0.9.8 初始化在 clk_npu prepare 返回 -71，随后 probe 失败。
- 串口显示当前 BL31 是 Trusted Firmware-A v2.14.0，构建日期 2026-07-24。
- v2.14.0 的 SCMI CLOCK_CONFIG_SET 接收端要求 12 字节请求。Linux 6.1 的旧接口使用 8 字节。
- 上游提交 1db1a46c4232a36de27b591d8b0c13fcf6882fe8 明确修复该兼容问题，并记录 Linux 6.1 的相同 -71 时钟故障。它是当前首要根因判断，仍需要应用兼容修复后验证。
- 重试绑定以采集 SCMI 跟踪时，试验内核失去响应。用户断电重启后恢复 7.1.3，CPU 测试继续。
- 本次未刷写 BL31 或 U-Boot。现有默认启动仍为 7.1.3。

## 可保留 Ubuntu 的修复方向

对候选内核移植 SCMI 时钟协议兼容支持，或构建包含上游修复的 BL31。两者涉及内核或启动固件，均需要单独验证恢复和部署步骤。现阶段没有应用补丁，也没有 NPU FPS。

上游提交
https://github.com/ARM-software/arm-trusted-firmware/commit/1db1a46c4232a36de27b591d8b0c13fcf6882fe8

Linux 6.1 时钟请求定义
https://github.com/torvalds/linux/blob/v6.1/drivers/firmware/arm_scmi/clock.c

TF-A v2.14.0 接收处理
https://github.com/ARM-software/arm-trusted-firmware/blob/v2.14.0/drivers/scmi-msg/clock.c
