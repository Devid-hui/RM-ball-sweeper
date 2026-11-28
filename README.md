# RM-ball-sweeper

将 RoboMaster 比赛场地中遗留的 17 mm 小弹丸进行自动追踪和清扫

## 工程结构

```text
src/main.cpp                  ball_capture，现场采图或从视频抽帧
src/collector_main.cpp        ball_collector，模拟观测决策演示
io/hikrobot/                  从原 RM 工程迁入的海康相机驱动
io/chassis/velocity_command.hpp  车体速度接口，供任务层和底盘通信共用
io/chassis/mecanum_kinematics.*   车体速度与四轮角速度的换算
tasks/ball_collect/           根据当前观测和安全条件计算速度
tasks/ball_detect/detect_cpu.py  CPU YOLO 采球检测入口
tasks/ball_detect/ground_position.py  小球地面定位和当帧目标选择
tools/calibrate_ball_camera.py   用棋盘格照片求相机内参
tools/                        日志、退出、队列工具和 AprilTag 位姿解算
```

## 采球流程

### 单球跟随

`--follow-ball` 用于单独验证从相机跟踪到轮速下发的链路。上位机先用两帧中的球心位置确认同一颗球，再把相对车头的前向和横向误差换成车体速度，最后换算成 A/B/C/D 四轮目标。控制板执行电机闭环。默认的跟随速度上限为 0.05 m/s，球心进入叉前预留距离后给零速。丢球时立即给零速，新目标重新确认。该模式省略墙标定位和推送流程。

在项目根目录运行回放以检查 `reason`、`velocity`、`wheels` 和 `factory_car_preview`。

```bash
BALL_COLLECTOR_BUILD_DIR="$PWD/build-collector" tools/run_collector.sh \
  --images test/picture/*.jpg --vision onnx-ground --follow-ball
```

相机预演改用 `--serial DA6415777`。跟随模式和完整任务共用 C26 串口入口。

### 完整巡线与推送

图像回放、相机预演和实车使用同一个 Python 入口。小车保持前摄朝墙标，用麦克纳姆轮沿相邻路段交替前进、倒退，横移换道。发现球后记住离开巡线的位置。连续两帧用墙标确认同一球后，小车移到球后方，对准前叉并低速推送。无墙标时只能低速接近可见球，球心距前叉前缘约 8–9 cm 时停车等墙标。

对准过程中一旦丢球就停车。墙标可用、连续三帧匹配不到原球，且最后记录的球位仍在前叉缓冲距离之外时，才解除锁定并回到中断的巡线位置。

球入叉后可以离开画面。推送和后退途中丢失墙标都停车。按墙标定位到投放区后，小车先停、后退，再从实测车头方向原地转 90°。转动时墙标可能离开前摄视野，实车用四轮编码器补测航向。随后转回朝墙方向，重新看到墙标才平移到刚才中断的路段。日志中的 `releases` 统计完成的投放流程，实际留在区内的球数需要现场清点。

在项目根目录构建运动学绑定并回放。两种视觉方案都需要 NumPy 和带 `aruco.ArucoDetector` 的 OpenCV。构建绑定需要 CMake、C++17、pybind11 和当前 Python 的开发头文件。Ubuntu 已装系统 OpenCV 时，可以用 `python3 -m venv --system-site-packages build-collector/venv` 建立环境，再运行 `. build-collector/venv/bin/activate`。使用 `onnx-ground` 时还需安装 `onnxruntime`。

```bash
cmake -S . -B build-collector -DBALL_BUILD_CAPTURE=OFF -DBALL_BUILD_RUNTIME=ON \
  -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build-collector -j2
PYTHONPATH=build-collector python3 -m tasks.ball_collect.run --images test/picture/*.jpg
PYTHONPATH=build-collector python3 -m tasks.ball_collect.run --images test/picture/*.jpg --vision onnx-ground
PYTHONPATH=build-collector python3 -m unittest discover -s test -p 'test_*.py'
```

默认按提案中的方案 A 运行 `hsv-diameter`，用 HSV 颜色提取圆形轮廓，根据 17 mm 球径及其成像视角测距。`--vision onnx-ground` 使用仓库中的 ONNX 模型识球，并把球心像素投影到距地 8.5 mm 的平面。需长期使用该方案时，将配置文件的 `vision` 设为 `onnx-ground`。图像回放输出 JSON 行。

在 ROCK 4D 上用下文的海康相机依赖构建两个绑定。通过启动脚本做相机预演，核对输出中的 `balls_m`、`tag_pose` 和 `wheels`。脚本在 Python 启动前将海康 SDK 运行库目录加入搜索路径。使用非默认构建目录或解释器时，设置 `BALL_COLLECTOR_BUILD_DIR` 和 `BALL_COLLECTOR_PYTHON`。

```bash
cmake -S . -B build-hik -DBALL_BUILD_CAPTURE=OFF -DBALL_BUILD_RUNTIME=ON \
  -DBALL_WITH_HIKROBOT=ON -DBALL_BUILD_YOLO_CAMERA=ON -DPython3_EXECUTABLE="$(command -v python3)"
cmake --build build-hik -j2
tools/run_collector.sh --serial DA6415777 --frames 60
```

当前板上的厂商固件通过 `$Car:a,b,c,d!` 接收四轮目标线速度，单位为 m/s。
相机预演和图片回放中的 `factory_car_preview` 给出换算后的命令。
已验证的 CH340 链路可在具备 USB 访问权限和 PyUSB 的环境里用以下命令检查。
该检查向控制板发送全零轮速，并核对 `cmdOk` 回复。

```bash
python3 tools/check_openrf1_uart.py --port ch340 --factory-car
```

默认协议为 C26，经 CH340 连接 OpenRF1 的 MV 口。程序启动先发送零速并检查 `cmdOk`。每帧查询 `$ENC!`、处理画面与任务状态，再换算并发送四轮目标。墙标给出绝对航向。墙标离开视野后，通过两次编码器采样的计数差估算转角，重新看到墙标时校正。日志中的 `encoder_abcd`、`encoder_sample` 和 `heading_rad` 分别记录原始计数、控制板采样号和估计航向。正常退出或帧、反馈、应答异常时，程序在关闭串口前尝试发零速。

完整任务上车前，先烧录并架空验证约 500 ms 的通信静默停车。在实车 JSON 中确认轮向和每圈计数后设 `drive_calibrated` 为 `true`，通过静默停车验收后设 `c26_watchdog_verified` 为 `true`。编译与测试步骤见 [C26 固件说明](firmware/openrf1/README.md#c26-v22-编码器反馈补丁)。

```bash
tools/run_collector.sh --serial DA6415777 --drive --config /path/to/vehicle.json
```

加 `--follow-ball` 可先验证单球跟随。使用系统串口时传入 `--port /dev/ttyUSB0` 等实际设备并安装 pyserial。候选固件使用 `--drive --protocol collect1`，默认端口为 `/dev/ttyS4`，也可显式指定 `--port ch340`。车体速度以前进、左移、逆时针为正。

## 实车参数和验收

任务参数集中在 `configs/collector.json`。可复制为场地专用 JSON，再用 `--config` 指定。相机参数在 `configs/hikrobot_DA6415777.yaml`，可用 `--calibration` 切换。

| 配置项 | 起始值 | 实测方法与影响 |
| --- | --- | --- |
| `encoder_counts_per_rev`、`drive_sign_abcd`、`drive_calibrated` | 4680、[1,1,1,1]、false | 架空后手转**输出轮**一整圈，以计数差的绝对值核对每圈计数。C26 计数按 32 位回绕，COLLECT1 按 16 位。分别点动 A/B/C/D，正命令使对应轮向前滚则填 1，反向填 −1。 |
| `c26_encoder_sign_abcd`、`c26_command_scale` | [-1,1,-1,1]、1.0 | 编码器符号用于把向前滚动的计数差统一换成正值。命令比例用于校准 C26 固件的速度单位，默认按计算出的轮端线速度直接发送。此前四轮发 0.10 m/s，实测车速约 0.0244 m/s。按实际地面测速复核比例和速度上限，并在实际车速下核对能否在 `turn_timeout_s` 内完成转向。 |
| `wheel_radius_m`、`wheelbase_m`、`track_m` | 0.03、0.30、0.20 m | 轮半径按实车 60 mm 轮径填写。测前后轮中心距和左右轮中心距，填完整距离。用直线和地面直角线检查位移、转向比例。 |
| `fork_forward_m`、`fork_width_m`、相机 YAML 的 `minimum_candidate_forward_m` | 0.06、0.20、0.06 m | 测车体中心到叉前缘、两杆内宽。在叉内和前方各摆一颗球。新目标的前向门限取前伸和近距门限的较大值，已锁定的球可继续靠近。 |
| `visual_approach_speed_m_s`、`visual_standoff_m`、`push_speed_m_s` | 0.05 m/s、0.08 m、0.05 m/s | 临时遮住墙标，测叉前停车间隙。用单球验证推送速度是否会让球滚出。搜索、对准速度另由 `search_speed_m_s`、`approach_speed_m_s` 控制。单轮上限 `max_wheel_rad_s` 默认为 6 rad/s。 |
| `push_entry_gap_m` | 0.10 m | 有墙标且车头已对准推送方向时，球心离叉前缘在此范围内可开始推送。按当前相机标定投影，球在车体原点前 0.11 m 时已越过图像下缘。用车上实拍复核开始推送前能完整看到球。 |
| `tag_id`、`tag_size_m`、`drop_zone` | 0、0.10 m、[0.12,−0.15,0.34,0.15] m | 量标签黑框外边长和区域四界 [xmin,ymin,xmax,ymax]。X 从墙面指向场内，Y 朝标签图案右侧。用尺核对停车处的球心和前叉与墙面的余量。 |
| `lane_starts`、`lane_end_x_m` | 两条路段远端 X 为 1.10 m，Y 为 ±0.25 m，近端 X 为 0.40 m | 按实地净空和相机有效视宽调整。第二条从近端往远端扫。换道前核对相邻两路能覆盖两路之间的球，倒退路段需保持净空。 |
| `reverse_m`、`turn_direction`、`turn_timeout_s` | 0.20 m、1、10 s | 先量后退至前叉离开球堆所需距离，再对地面直角线量实际转角。1 为左转，−1 为右转。软件进入目标角 0.05 rad（约 2.9°）内时停车。若超时，先查轮序、计数和打滑。 |
| 相机 YAML 的 `camera_matrix`、`distortion_coefficients`、`camera_to_chassis` | 1440×1080，光心高 0.10 m，安装角均为 0° | 核对工作画幅，量相机相对车体中心的位置和俯角。在前方 0.25、0.50、1.00 m 和左右偏移处摆球，逐点比较输出的前向、左向坐标。 |
| `hsv_lower`、`hsv_upper`、`diameter_scale` | [18,45,35]、[46,255,255]、1.0 | 方案 A 的黄绿色起始范围。用现场光照下的球和空地调颜色，避免只提取到球的一小块。用相机光心到球心的实测距离与估计距离之比修正比例，再换距离复测。 |

**2026-11-26 架空单轮实测**。OpenRF1 的 `$Car` 参数按 A/B/C/D 排列，本次对应 **前左、前右、后左、后右**。ROCK 4D 经 CH340 接 OpenRF1 的 MV 口，依次单轮转动约 3 秒，每轮转动和归零指令均收到 `cmdOk`。见 [单轮实测记录](firmware/openrf1/README.md#2026-11-26-架空单轮实测)。

采球程序的 [轮速发送与航向计算](tasks/ball_collect/openrf1.py) 共用 `wheel_order_abcd`。默认示例按 11 月 26 日驱动通道实测填 `["fl","fr","rl","rr"]`，依次表示 A 前左、B 前右、C 后左、D 后右。旧实车配置缺少此项时按当时的 A 后左、C 前左解释。启用 `--drive` 前，逐轮核对编码器通道、向前滚动时的计数符号和正指令转向。`drive_sign_abcd` 只改变对应通道的驱动极性。

从等待相机启动到收到首帧的时限由 `camera_startup_timeout_s` 设置，默认为 8 秒。收到首帧后，按 `max_frame_age_s` 检查连续出帧和发送前的帧年龄。日志中的 `frame_age_ms` 是从驱动收到图像到轮速发送前的时间。图片回放该项为 `null`。

上地前在架空状态检查四轮同向前进、左移和逆时针旋转的轮序，并确认主循环连续发送轮速的间隔低于固件约 500 ms 的停车时限。识球耗时或串口等待过长会使底盘超时归零。地面验收连续做两轮完整推送，遮挡一次墙标验证停车与恢复，记录布球数、实际入区数、漏收位置、重复锁定次数及实测转角。

## 运行决策演示

只构建任务入口时需要 C++17 编译器和 CMake。演示使用模拟输入并打印决策原因和速度。

```bash
cmake -S . -B build-collector -DBALL_BUILD_CAPTURE=OFF
cmake --build build-collector -j2
./build-collector/ball_collector --demo
```

## 麦克纳姆轮解算

解算采用 X 型滚子布局，车体原点取四轮中心。输入依次为轮半径、轴距、轮距（米），以及车体的前进、左移、逆时针速度（m/s、m/s、rad/s）。轴距和轮距填写完整的轮心间距。输出按前左、前右、后左、后右排列，单位为 rad/s，各轮向车头滚动时为正。

```bash
# 用示例尺寸验证纯左移，预期四轮依次为 -4、4、4、-4 rad/s
./build-collector/ball_collector --mecanum 0.05 0.30 0.20 0 0.20 0
# 给模拟决策追加四轮角速度，最后的 6 是可选轮速上限（rad/s）
./build-collector/ball_collector --demo 0.05 0.30 0.20 6
ctest --test-dir build-collector --output-on-failure
```

指定上限时，超限会按同一个比例缩放四轮，保留前进、横移和转向的比例。上述 C++ 演示只打印解算结果。Python 实车入口把四轮指令发给采球固件。

## OpenRF1 串口联通检查

检查工具自动区分早期回显、四轮点动和采球固件。点动固件支持单轮固定脉冲，采球固件通过 USART2 接收四轮闭环指令。固件版本和轮序见 [OpenRF1 说明](firmware/openrf1/README.md)。

通信指南已验证 ROCK 4D 的 UART4-M1（`/dev/ttyS4`）到 OpenRF1 USART2 的基本双向通信。
应用串口为 115200、8N1、无流控。两板使用 3.3 V TTL，ROCK 物理 11（TX）接 RX2，
物理 15（RX）接 TX2，物理 9（GND）接 GND。两板分别供电。

在 ROCK 4D 上关闭占用串口的其他程序，再从项目根目录运行。接线记录在 `configs/chassis_link.yaml`。设备名不同时，给检查工具和采球入口分别传 `--port`。

```bash
python3 tools/check_openrf1_uart.py
```

默认只查询。电机固件会返回 A/B/C/D 四个原始计数，`VER` 返回 `COLLECT1` 表示采球固件。使用点动固件且车已架空时，运行 `python3 tools/check_openrf1_uart.py --jog A` 可点动一个通道。`--jog all` 逐个执行并打印四路计数差，出现错误即停止后续通道。每个通道每次上电或复位只接受一次点动。

## 采球车图像采集

第一阶段支持在 RK3576 Ubuntu 上从单台海康 USB3 工业相机采图，也支持从本地视频抽取连续帧。程序保存原分辨率 PNG 和 `frames.csv`，可直接检查 17 mm 小球的可见范围，或用这些图片做检测标注。

`io/hikrobot/`、`io/camera.hpp`、所需的 `tools/` 文件和许可证来自 RoboMaster 源码。驱动提供限时读取，以便实时检测入口等待新帧并响应 Ctrl+C。新项目的 `src/camera_base.cpp` 提供相机基类的原有默认实现，以便独立链接海康驱动。

## Ubuntu 上连接海康相机

本阶段使用原工程的 USB3 枚举和采集驱动。采球车当前相机的序列号是 `DA6415777`，镜头标注为 `3.37 mm F2.8`。更换相机时可用 `--serial` 指定新序列号。

```bash
sudo apt install g++ cmake libopencv-dev libfmt-dev libspdlog-dev
cmake -S . -B build-hik -DBALL_WITH_HIKROBOT=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-hik -j2
./build-hik/ball_capture --serial=DA6415777 --output=captures/field-01 --frames=100
```

`--exposure-ms=8 --gain=8 --fps=15` 是采集入口的起始参数，可结合实际光照和相机型号调整。文件中的时间从第一张保存的图像开始，来自采集回调的单调时钟。保存图片慢于相机取帧时，原驱动会保留最新帧，连续保存的图片可能跳过部分相机帧。看不到首帧时，先检查 MVS 是否可枚举相机及当前用户是否有设备访问权限，再对照程序的相机日志。按 Ctrl+C 可退出。

`configs/hikrobot_DA6415777.yaml` 复用 RM 中同序列号 `omni1` 的内参与畸变，安装参数按当前默认值写入。RM 未注明该矩阵的标定画幅，配置暂以实拍画幅 1440×1080 运行。用已知距离和左右偏移验证后再用于实车控制。

## 从本地视频提取图片

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./build/ball_capture --video="/path/to/field.mp4" --output=captures/replay-01 --frames=100
```

视频时间由 OpenCV 视频后端提供。每次使用新的输出目录，保留不同场次的数据。

## CPU YOLO 检测

从项目根目录运行。Python 环境需要 NumPy、OpenCV 和 ONNX Runtime。
使用已经装好这些依赖的板端虚拟环境，或者在自己的 Python 环境中安装
`numpy opencv-python-headless onnxruntime`。

```bash
python tasks/ball_detect/detect_cpu.py --images test/picture/*.jpg
```

实时检测在 RK3576 上复用海康驱动，只取队列中的最新帧。构建时需要
`libopencv-dev libfmt-dev libspdlog-dev pybind11-dev python3-dev` 和 CMake 3.18 以上。
下面用板上的现有 Python 环境，按实际路径调整。

```bash
. "$HOME/.venvs/ball-yolo-bench/bin/activate"
cmake -S . -B build-hik -DBALL_WITH_HIKROBOT=ON -DBALL_BUILD_YOLO_CAMERA=ON \
  -DPython3_EXECUTABLE="$(command -v python)" -DCMAKE_BUILD_TYPE=Release
cmake --build build-hik -j2
PYTHONPATH="$PWD/build-hik" python tasks/ball_detect/detect_cpu.py \
  --serial DA6415777 --frames 60 --output ball-detections.jsonl
```

结果每帧一行，`xyxy` 是原图像素坐标，`confidence` 是置信度。
`detect_ms` 包含预处理、推理和后处理，`frame_age_ms` 从驱动收到帧开始计时。
默认使用配置中的 4 个推理线程，测试时可加 `--threads 8` 对照帧率和 CPU 占用。
省略 `--frames` 时持续运行，按 Ctrl+C 退出。
实时模式使用 `--output` 将检测结果和驱动日志写到各自的输出位置。
旧的 PyTorch 离线脚本可加 `--no-preview`，只生成检测框记录。

### 地面区域加速配置

`--profile floor-roi` 去掉原图顶部 15%，用同一组权重导出的 800×512 ONNX 推理。
对 1440×1080 原图，程序保留画面全宽，小球缩放比例与全图配置接近。
检测框会还原为原图坐标，`roi_top_px` 记录裁剪边界，碰到该边界的框跳过地面定位。

```bash
python tasks/ball_detect/detect_cpu.py --images test/picture/*.jpg --profile floor-roi
```

Mac M1 Pro 上分别用 4 线程交替回放同一套 45 张标注图，每配置计时 90 帧。
检测匹配 IoU 为 0.50，置信度门槛为 0.30。

| 配置 | 平均检测耗时 | P95 检测耗时 | 正确检测 | 误检 | 命中有球画面 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `full` 800×608 | 44.34 ms | 47.08 ms | 180 | 41 | 33/36 |
| `floor-roi` 800×512 | 37.17 ms | 39.17 ms | 177 | 53 | 33/36 |

默认使用 `full`。固定测试集中所有标注球均在所选裁剪范围内。
启用地面区域配置前，用相机实际安装姿态检查顶部裁剪线以下是否覆盖 1 米工作范围。
RK3576 上的帧率和温度待实测。

## 地面小球定位

现有海康相机可用 `--ground-calibration configs/hikrobot_DA6415777.yaml` 试跑。地面配置会在选择新目标时跳过前向距离小于 0.06 m 的球。推送逻辑可继续沿已确认的路径前进。

需要另行标定相机时，用车上这台相机以实际工作时的分辨率、裁剪和焦点采集棋盘格照片。
建议采集 10–20 张，改变棋盘角度和距离，并让棋盘覆盖画面四角。
内角点按格线交点计数。下面以横向 9、纵向 6 个内角点、实测格宽 20 mm 为例。

```bash
python tools/calibrate_ball_camera.py --images calibration/*.png \
  --columns 9 --rows 6 --square-mm 20 --output configs/ball_ground.yaml
```

在生成文件末尾追加以下字段，并把每个 `待测` 都替换成实测数字。
车体原点可取两驱动轮轴线中点在地面的投影。选定后，相机和收球口都以同一原点测量。
水平且朝正前的相机三个角度为零。向左转为正偏航，镜头向下为正俯仰，画面的左侧抬高为正横滚。

```yaml
camera_to_chassis:
   x_m: 待测             # 相机光心相对原点的前向距离
   y_m: 待测             # 相机光心相对原点的左向距离
   z_m: 待测             # 相机光心离地高度
   yaw_deg: 待测
   pitch_down_deg: 待测
   roll_deg: 待测
```

```bash
python tasks/ball_detect/detect_cpu.py --images test/picture/*.jpg \
  --ground-calibration configs/ball_ground.yaml
```

实拍定位时也可把同一参数加到海康检测命令。程序先校正框中心的镜头畸变，
再将这条视线与距地面 8.5 mm 的球心平面求交。`position_m` 给出车体前向和左向坐标，
`target_candidate` 指向当帧离车体原点最近的有效框。边缘被裁断的框、车体后方及范围外的球
仍出现在检测结果中，位置字段为 `null`。有效范围默认 1 米，可用 `--max-distance-m` 调整。
用安装后拍到的已知球位核对定位误差，尤其是收球口附近及最远工作距离。

## AprilTag 位姿解算

`apriltag_pnp` 对 `tag36h11` 的 ID 0–5 解算位姿，对应 `tag36h11 IDs 0-9.pdf` 的前 6 页。按实际大小打印，保留四周白边，然后用尺量黑色外边框的边长，单位为毫米。PDF 中的设计边长约 100 mm，命令里的 `--tag-size-mm` 以实测为准。同一批处理的标签应使用相同边长。

在 RK3576 Ubuntu 上，使用 `libopencv-dev`（OpenCV 4.6 及更早版本还需要 `libopencv-contrib-dev`）、`libfmt-dev` 和 `libspdlog-dev` 构建海康取图模式。下例沿用项目记录的海康序列号。如果 `tags` 使用另一台海康相机，填写它的实际序列号。

```bash
cmake -S . -B build-tags -DBALL_BUILD_CAPTURE=OFF -DBALL_BUILD_APRILTAG_PNP=ON -DBALL_WITH_HIKROBOT=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-tags --target apriltag_pnp -j2
./build-tags/apriltag_pnp --serial=DA6415777 --calibration=configs/tags_calibration.yaml --tag-size-mm=100
```

如果 `tags` 是系统视频设备，把 `--serial=DA6415777` 换成 `--device=/dev/tags`（假设这是实际设备路径），也可以用 `--device=0` 指定 OpenCV 摄像头索引。离线调试时换成 `--image=frame.png`。这两种模式构建时可将 `BALL_WITH_HIKROBOT` 设为 `OFF`。相机模式默认持续处理，可用 `--frames=100` 限制帧数或按 Ctrl+C 退出。

将 `tags` 相机在实际分辨率、裁剪和对焦设置下的针孔标定结果保存为 OpenCV YAML，例如 `configs/tags_calibration.yaml`。下方数字仅用于说明格式。仓库现有的 `configs/hikrobot_DA6415777.yaml` 也可用于这台海康相机的离线标签解算，原始标定画幅待核验。

```yaml
%YAML:1.0
---
image_width: 640
image_height: 480
camera_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [800., 0., 320., 0., 800., 240., 0., 0., 1.]
distortion_coefficients: !!opencv-matrix
  rows: 1
  cols: 5
  dt: d
  data: [0., 0., 0., 0., 0.]
```

每帧每个成功解算的标签输出一行，包含帧号、标签 ID、标签中心在相机坐标系中的 `x_m,y_m,z_m`、标签到相机的 Rodrigues 旋转向量（弧度）、相机光心在该标签坐标系中的位置（米），以及四角重投影 RMSE（像素）。相机坐标轴为右、下、前。标签坐标原点为标签中心，按解码后的标签朝向取右、上、朝向标签正面外侧。多个标签各自使用自己的原点。加上 `--output=poses.csv` 可将位姿单独保存到 CSV，海康日志仍显示在终端。
