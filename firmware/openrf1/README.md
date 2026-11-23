# OpenRF1 V2.2 四轮点动与编码器检测

适用于 STM32F103RCT6。复位后等待串口命令。板载 USB-TTL（USART1）和 ROCK 4D 所接的 MV 口（USART2）均为 115200、8N1、无流控。每条命令以 LF 结束，支持 CRLF。一次使用一个命令发送端，回复会出现在两个串口上。

| 命令 | 回复及行为 |
| --- | --- |
| `PING` | `PONG` |
| `ENC` | 四路原始 16 位计数，例如 `A=0 B=0 C=0 D=0`；读数可在 0 和 65535 之间回绕 |
| `JOG C`（A、B、D 同理） | 对所选驱动通道输出正向 PWM，20% 占空比，约 0.2 秒后返回 `DONE`。同一通道每次复位只执行一次，重复请求返回 `ALREADY` |

手转实测对应关系如下。增量符号以车轮向前滚动为准。

| 轮位 | 前左 | 前右 | 后左 | 后右 |
| --- | --- | --- | --- | --- |
| 编码器通道 | C | B | A | D |
| 向前时原始增量 | − | + | − | + |

实板已验证 Flash 写入、复位后 PING 应答和 ENC 查询。驱动正命令对应的实际轮位和转向仍待通电验证。点动前需架稳车身使四轮离地，移开手和线束，为电机驱动接入匹配的主电源。对本车额定 6 V 的 TT 马达，选用极性正确的 6 V 稳压主电源，按控制板手册选取至少 5 A 的供电能力。USB 可用于下载和查询编码器。

## 构建与烧录

从仓库根目录执行，需要 CMake 3.20+、Ninja 和 GNU Arm Embedded 工具链（实测 GCC 14.2.1）。

```sh
cmake -S firmware/openrf1 -B build-openrf1 -G Ninja
cmake --build build-openrf1
cmp build-openrf1/openrf1_wheel_jog.hex firmware/openrf1/release/openrf1_wheel_jog.hex
```

[可供 FlyMcu 写入的 HEX](release/openrf1_wheel_jog.hex) 与当前实板写入的版本对应。使用 OpenRF1 白色 USB-TTL 对应端口，在 FlyMcu 选择“RTS的高电平复位,DTR低电平进BootLoader”，使用已验证的 9600 下载速率并启用写后校验。烧录会替换板上原程序。应用启动后切换至 115200 发送 PING 和 ENC。

启动代码、链接脚本、CMSIS 和标准外设库来自 ROCK4D_OpenRF1_通信配置指南_20261122 配套工程，第三方文件保留原版权声明。四路引脚映射依据厂家 C25 v5 电机例程。
