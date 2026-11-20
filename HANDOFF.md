# 采球车项目交接

记录日期为 2026-11-20。以下运行状态以本轮最后一次实测为准。接手后先重新确认 SSH 地址、运行内核和 Git 状态。

## 项目目标与限制

用户是计算机专业本科生，需要中文、循序渐进的解释，包括关键决定的原因、验证方法和尚未验证的部分。

项目用于收集 RoboMaster 场地上散落的直径 17 mm 小球。机械采集方案已有，本阶段工作集中在软件与必要传感器。预算有限，目标是降低板端算力、功耗与负担。

已明确的感知目标是约 1 米内发现小球，车辆向前推进时持续发现新球。检测层需要输出球的检测框，后续可继续训练。检测器可输出多个球，由任务层选择当前追踪对象。车速和实际安全控制周期尚未确定。

用户目前要求保留现有 Ubuntu，当前无法更换系统。当前重点是 CPU 参数优化和查明 NPU 初始化失败原因。

用户要求从成熟 RoboMaster 代码迁移时尽量沿用已验证实现，避免无必要修改。当前任务逻辑按输入直接计算速度，使用提前返回，减少深层嵌套。

用户曾讨论 PnP 定位，具体小球定位方法尚未完成。普通球体的 YOLO 矩形框缺少标准 PnP 所需的已知三维点与二维对应关系，需要结合相机标定、地面约束或球体投影模型评估。现有 AprilTag PnP 工具用于标签定位。

## 本地资源

- 工作区为 /Users/devid/Documents/大三上/robic/project
- 成熟 RM 源码为 /Users/devid/Documents/视觉/RM_Vision_2027_fe0ead0/
- 数据包为 /Users/devid/Documents/final数据包/
- 机械资料为 /Volumes/Devid/智能车/，依赖外置卷挂载
- 项目根目录已有 采球车追球进度.xlsx。后续实验结果尚未全部更新进去。编辑前检查是否正被 Excel 打开。
- 两张真实测试图在 test/picture/，原图尺寸为 1440×1080。

交接时 git status 显示以下新增资产尚未跟踪，需要迁移本地文件本身。
asset/ball_detector_p3p4/
asset/ball_detector_cpu_tuning/
asset/ball_detector_rk3576_fp16/
本次交接另外增加 HANDOFF.md 与 test/deployment_records/。核对状态后再自行决定提交，避免遗漏未跟踪文件。

## 硬件与访问

主控为 Radxa ROCK 4D，PCB 丝印 v1.12，SoC RK3576。
板上 CPU 为 4×Cortex-A72 与 4×Cortex-A53，系统可见内存约 5.8 GiB。
板子使用 Ubuntu 系 Armbian，默认运行内核为 7.1.3。另有 7.2.8-edge-rockchip64 文件，默认启动的 Image 实际指向 7.1.3，必须以 uname -r 为准。

ROCK 4D 最近 SSH 为 ubuntu@172.16.183.126。
曾校验的 ED25519 指纹为 SHA256:wWoTgGYPQ7mb8+oMOa3mZnkTm7oTQEog/eC3pMebhpQ。
AMD Ubuntu 转换主机为 devid@10.0.0.214，x86_64，Python 3.12.3，可用空间约 326 GB。
账户密码由用户在新会话中单独提供，本文件不保存密码。

Mac 上调试串口最近为 /dev/cu.usbserial-1140，1500000 波特率，8N1。插拔后端口名可能变化。
默认内核串口为 /dev/ttyS0，候选 6.1 内核为 /dev/ttyFIQ0。
板端现场可能需要用户断电恢复，任何重启先确保当前操作范围和恢复方式。

相机为 Hikrobot MV-CB016-10UC-S，序列号 DA6415777，镜头标注 3.37 mm F2.8。
相机输出实测为 1440×1080。历史在 USB3 5000M 连接下，内存采集可达到 15.01 FPS（目标 15）和 55.26 FPS（目标 60），PNG 保存约 8.54 FPS。
本轮实时联测中，旧采图入口曾保存 3 张图。其后相机在 USB3 的 5000M 链路被系统识别，
但原采图入口和实时检测入口均出现 MVS USB 读写失败，重新插接后也出现枚举序列号为空。
最后一轮在 MVS 重试期间收到中断信号，进程被测试超时器终止。
当前无有效的实时相机 YOLO 帧率，优先恢复 MVS 稳定枚举和取图。
板上专用 udev 规则为 /etc/udev/rules.d/99-hikrobot-da6415777.rules，用户组 hikrobot-camera。
configs/hikrobot_DA6415777.yaml 中内参来自旧 RM omni1，仅是待核验参考，缺少原始标定分辨率。相机到车体外参待测。
驱动板、底盘电机配置和通信协议仍未落实。

## 当前代码结构

- src/main.cpp 为 ball_capture，负责相机采图和视频抽帧。
- io/hikrobot/ 为迁入的海康驱动。io/camera.hpp 为相机接口。
- src/collector_main.cpp 为 ball_collector 决策演示，当前使用模拟输入。
- tasks/ball_collect/decision.hpp/.cpp 定义当前观测输入和速度决策。故障后由上层撤销运动许可，人工授权后恢复。
- tasks/ball_detect/detect_cpu.py 复用 ONNX Runtime 会话和缓冲，支持图片及海康相机输入，输出原图检测框和时间戳。
- RK3576 上相同的 45 张图、800×608 模型和 4 个线程交替回放，旧脚本和新入口输出一致，均约 2.95 FPS。此数据来自图片回放。
- io/chassis/velocity_command.hpp 定义车体速度接口，前进、左移、逆时针为正，单位 m/s、m/s、rad/s。
- tools/apriltag_pnp.cpp 为标签位姿工具，已有独立 CMake 开关。
- README.md 和 CMakeLists.txt 给出了现有入口与构建方式。

定位、目标确认、任务决策与实车底盘之间的集成待完成。

## 原始模型与数据

正式权重为 asset/ball_detector_precision/selected_model.pt。
模型是 YOLO26n，类别仅 {0: ball}，nc=1，end2end=True，reg_max=1。
原始模型 SHA256 为 edf3d308590f4af0decbb5d2981d10a05ec924b473cd3ecd2e3778dfeb2434f0。
它已经是单类别模型，没有其他物体的类别输出可再删除。P3/P4/P5 是不同分辨率的检测分支。
selected_config.json 目前为 single，conf=0.30，NMS IoU=0.70，imgsz=960。
以前 flip_consensus 会额外检测镜像图，已按用户要求改为单次原图推理。旧验证指标仍明确标注为 flip_consensus，不能当作当前精度。
1440×1080 原图在默认矩形预处理下对应网络输入 960×736（宽×高）。

数据包有 462 张正样本、56 张负样本、690 张待确认，共 1208 张。
待确认图片包含机器预测，不可直接当可靠人工标签。
两张 人工复核_含极小框 图片有异常标签。
数据清单.json/.csv 记录原始 train/val/test 划分、来源和哈希。保留原始划分，检查近似帧泄漏。
固定 CPU 量化校准曾使用原 train 中 210 张已确认图片。
固定测试集为原 test 中 45 张，36 张含球、9 张背景、共 336 个标注球。
原 val 的另 45 张含 616 个球，用于之前的 P5 裁剪对照。务必分清两套 45 张集合。
目前报告的这些固定集结果支持工程对照，独立泛化评估仍需新场景。原模型历史训练曝光范围也需核实。

## 已做优化与实测

所有 FPS 都要结合后端、输入尺寸和时长理解。离线图片回放成绩尚未包括实时相机取图和实车控制。

1. 原始 .pt 使用 CPU 单图推理，在 RK3576 重复两张实拍图运行约 10 分钟，655 帧，约 1.09 FPS，过程 CPU 约 570%，温度曾达到约 85°C。
2. 原始三尺度模型导出 ONNX FP32，输入 960×736、4 线程，约 45 秒板端测试为 2.09 FPS。
3. ONNX Runtime 静态 INT8 QDQ、Conv per-channel 量化，确认板端优化图有 102 个 QLinearConv。相同条件约 2.37 FPS，但 TP 从 178 降为 161。量化模型没有替换项目正式权重。
4. P5 分支裁剪候选在 asset/ball_detector_p3p4/。prune_p5.py 删除只供 P5 的第 20–22 层并保留 P3/P4，参数约 250.4 万降到 176.6 万。Mac 单帧中位数从 86.4 ms 降到 77.7 ms。原 val 的 616 个球中 TP 从 421 降为 382。仅验证过训练器完整加载和一次反向传播，尚未实际微调或在板上测速。
5. 卷积通道剪枝仅试验过，精度退化，未采用。

2026-11-20 在恢复后的 7.1.3 上做 ONNX FP32 尺寸与线程短测，每组约 12 秒。
网络输入      2线程 FPS   4线程 FPS   8线程 FPS
960×736       1.47        2.07        2.51
800×608       2.12        2.91        3.55
640×480       3.35        4.50        5.39

同一测试集与 conf=0.30，NMS IoU=0.70。
输入          TP       FP      FN      有球画面命中
960×736       178      45      158     34/36
800×608       180      41      156     33/36
640×480       145      39      191     33/36

800×608、conf=0.20 时 TP=221，FP=79，FN=115，有球画面命中 34/36。
降低置信度会增加误检，无法显著减少卷积计算。该阈值只作为对照候选。
800×608、4线程连续 60.22 秒为 177 帧、2.939 FPS，P95=347.55 ms，过程 CPU=337.7%，RSS=421 MiB，结束温度 80.384°C。
它是当前较合适的 CPU 候选，仍需 1 米实拍移动视频和长时间散热验证。
正式配置没有切换到该候选，candidate_config.json 仍记录 conf=0.30。

持久化资产在 asset/ball_detector_cpu_tuning/。
包含 ball_800.onnx、cpu_tuning.json、cpu_800_sustained.json、candidate_config.json、resolution_accuracy_mac.log。

## NPU 已完成与阻塞

AMD 主机转换目录为 /home/devid/ball-rknn.BTCxn4/，独立 Python venv。
使用 RKNN Toolkit2 2.3.2、Torch 2.4.0+cpu、ONNX 1.17、numpy 1.26.4、protobuf 4.25.4、ORT 1.20.1。
ONNX 在 Mac Ultralytics 8.4.120 导出，再在 AMD Linux 构建 RK3576 FP16。
Linux 转换流程是官方支持路径，Mac 可继续处理原模型和 ONNX。

FP16 RKNN 模型位于 asset/ball_detector_rk3576_fp16/ball-rk3576.rknn，大小 11,959,990 字节。
SHA256 为 2aee8be03fac29d75007067afba5f79519a0d4d22023c67766b3e4e8e2128821。
板端副本为 /home/ubuntu/ball-rknn-fp16.qJVwwA/，有 metadata.yaml 和独立 librknnrt.so。
输入为 1×3×736×960，实际运行提交 RGB uint8 NHWC，由 RKNN 内部做归一化。
RKNN 导出路径关闭 end2end，输出 1×5×14490，使用外部 NMS。检测头变化本身会改变结果，需要与相同检测头的 ONNX 对照。

45 张固定 test 上同检测头 ONNX 参考 TP=218、FP=78、FN=118，RKNN FP16 模拟器 TP=217、FP=78、FN=119，均命中 34/36 张有球画面，9 张背景均未检出。
这些是模拟器结果。板端尚未成功开始 NPU 推理，没有 NPU FPS。

默认 7.1.3 没有 RKNN 可用的 RKNPU 驱动。
已从 Armbian 包下载、解包 6.1.115-vendor-rk35xx，其配置启用 RKNPU，ROCK 4D 设备树启用 NPU。
板载 AIC8800 USB Wi-Fi 驱动已对该内核编译并加载成功。
临时内核、DTB、initramfs 和 trial.scr 在 /boot/rock4d-rknn-trial/。
候选模块在 /lib/modules/6.1.115-vendor-rk35xx/。
临时引导是在 U-Boot 内存中手动加载候选文件进行，没有修改默认启动链接。
最后正常默认项为 /boot/Image -> vmlinuz-7.1.3，/boot/uInitrd -> uInitrd-7.1.3。/boot/dtb 原先已指向 dtb-7.2.8-edge-rockchip64，勿把已有状态误判为本轮更改。

候选 6.1 内核启动后 SSH 和 Wi-Fi 正常，RKNPU 0.9.8 在 clk_npu prepare 返回 -71 后 probe 失败。rknn_init=-1。
串口显示 BL31 为 Trusted Firmware-A v2.14.0，构建日期 2026-07-24。
该版本 SCMI CLOCK_CONFIG_SET 接收端要求 12 字节，Linux 6.1 老接口发送 8 字节。
上游已有明确修复，说明 Linux 6.1 存在相同的时钟 -71 故障。
这是首要根因判断，需要应用修复后的板端验证才能闭环。
本轮尝试重新绑定 RKNPU 以记录 SCMI trace 时，试验内核失去响应。用户断电上电后恢复默认 7.1.3。
后续不要直接重复失败驱动的 bind 操作，先制定可回退的修复测试。
未刷写 BL31/U-Boot，未应用兼容补丁。

上游修复提交
https://github.com/ARM-software/arm-trusted-firmware/commit/1db1a46c4232a36de27b591d8b0c13fcf6882fe8
Linux 6.1 接口
https://github.com/torvalds/linux/blob/v6.1/drivers/firmware/arm_scmi/clock.c
官方转换要求
https://docs.ultralytics.com/integrations/rockchip-rknn
板型文档
https://docs.radxa.com/en/rock4/rock4d

保留 Ubuntu 的后续方向是内核侧 SCMI 时钟协议兼容移植，或使用包含上游修复的 BL31。涉及启动固件写入时，要先给出具体补丁、构建产物、校验与恢复方案，并取得对应操作授权。用户先前授权过一次临时内核试启动，不能据此推断已授权任意固件刷写。

## 实验脚本和可复现性

本次将一部分临时脚本、头文件、固定测试集清单和测速记录复制到 test/deployment_records/，方便接手检查。
这些文件是实验记录，保留当时写死的远端路径。运行前逐项修正路径、依赖和输入输出，不能直接当作项目正式推理入口。
test_manifest.json 的图片为 eval/ 相对路径，图片副本还在板端 /home/ubuntu/ball-onnx-bench.Ng9tXC/eval/。
对应原图可在外部数据包按文件名及哈希查找。
CPU 原始对照记录在 previous_cpu_comparison.csv。

远端原始位置
/home/ubuntu/ball-onnx-bench.Ng9tXC/
/home/ubuntu/npu-test.TIdU98/
/home/ubuntu/rock4d-kernel-check.JGWNvn/
/home/ubuntu/ball-rknn-fp16.qJVwwA/
/home/devid/ball-rknn.BTCxn4/

Mac 的 /tmp/ball-*、/tmp/codex-* 中还有实验文件和旧 SSH 套接字。临时目录和连接会失效，应先使用本项目持久化记录，再检查远端原始目录。

Mac 命令环境有过访问 10.0.0.214 返回 No route to host，但 macOS Terminal 能访问的情况。先检查真实网络路径，可使用已连通的终端。旧 multiplex socket 断电后可能失效，重建连接再验证，不能把单一路径错误当作主机故障。

## 接手后的建议顺序

1. 阅读本文件、README、模型配置、测试 JSON 和 diagnosis.md，确认项目已有入口和证据边界。
2. 重新验证两台主机身份、uname -r、网络、板端温度和串口可用性。需要密码时向用户单独索取。
3. 保留当前 Ubuntu。把 CPU 800 输入作为候选，先核对实际 1 米视频的发现率、连续漏检和新球出现后的发现延迟。
4. NPU 修复先核对 TF-A 与具体 vendor 内核源码，形成最小兼容修复和可恢复的测试产物。当前症状已有明确上游修复线索，避免重复遍历一般相机、YOLO 或量化问题。
5. 核对相机内参和安装外参，确定小球地面定位方案。
6. 按现有模块接入检测结果、目标确认和运动决策。收到实际驱动板型号、协议、CAN ID、单位及反馈定义后再接底盘。
7. 记录模型哈希、输入尺寸、后端、阈值、测试集、FPS、功耗温度和未验证项。需要时更新根目录 Excel。

## 需要用户现场提供或确认的内容

- 给新 AI 当前工作区和相关文件的访问权限。云端 AI 需要上传文件，无法直接读取 Mac 路径。
- 通过合适的私密方式提供两台主机的登录凭据，确认新环境到板子的网络通路。
- 保持 ROCK 4D 供电、网络和 Mac 串口连接，需要时执行断电恢复。
- 提供固定安装姿态下、实测 1 米距离及小车前进时新球入镜的视频。保留原始分辨率。
- 给出相机安装高度、俯仰角、轮胎/底盘尺寸、预期车速和采集入口位置。
- 驱动板到位后提供型号、电机型号、固件/通信协议、供电及编码器反馈信息。
