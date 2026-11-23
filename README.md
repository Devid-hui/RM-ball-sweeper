# RM-ball-sweeper

将 RoboMaster 比赛场地中遗留的 17 mm 小弹丸进行自动追踪和清扫

## 工程结构

```text
src/main.cpp                  ball_capture，现场采图或从视频抽帧
src/collector_main.cpp        ball_collector，模拟观测决策演示
io/hikrobot/                  从原 RM 工程迁入的海康相机驱动
io/chassis/velocity_command.hpp  车体速度接口，供任务层和底盘通信共用
tasks/ball_collect/           根据当前观测和安全条件计算速度
tasks/ball_detect/detect_cpu.py  CPU YOLO 采球检测入口
tasks/ball_detect/ground_position.py  小球地面定位和当帧目标选择
tools/calibrate_ball_camera.py   用棋盘格照片求相机内参
tools/                        日志、退出、队列工具和 AprilTag 位姿解算
```

任务层按当前观测返回速度和原因。搜索时选前方可接近的小球，车头对准球与收球区的推送方向后请求开始推球。
推球口在车体原点前方约 0.06 m，宽约 0.20 m。球心与前叉中线的横向偏差需落在扣除球半径后的宽度内。
推送任务由导航提供，球进入前叉后即使离开画面也可继续推送。
靠墙时墙标过期则停车，恢复观测后可继续。到达墙边停车点时输出零速度。
S 形路线、把墙标位姿换算成停车点以及停车后的后退和 90° 转向，待接入导航和底盘反馈。

急停或视觉与底盘反馈过期时，输出零速度并撤销运动许可。上层确认故障解除并收到人工授权后再恢复运动。
速度约定前进、左移、逆时针为正，单位为 `m/s`、`m/s`、`rad/s`。
ROCK 4D 到 OpenRF1 的串口参数见 `configs/chassis_link.yaml`。

## 运行决策演示

只构建任务入口时需要 C++17 编译器和 CMake。演示使用模拟输入并打印决策原因和速度。

```bash
cmake -S . -B build-collector -DBALL_BUILD_CAPTURE=OFF
cmake --build build-collector -j2
./build-collector/ball_collector --demo
```

## OpenRF1 串口联通检查

通信指南已验证 ROCK 4D 的 UART4-M1（`/dev/ttyS4`）到 OpenRF1 USART2 的基本双向通信。
应用串口为 115200、8N1、无流控。两板使用 3.3 V TTL，ROCK 物理 11（TX）接 RX2，
物理 15（RX）接 TX2，物理 9（GND）接 GND。两板分别供电。

在 ROCK 4D 上关闭占用串口的其他程序，再从项目根目录运行。

```bash
python3 tools/check_openrf1_uart.py
```

检查程序向测试固件发送 `PING\n` 与 `HELLO_123\n`，预期分别收到 `PONG` 和 `RX:HELLO_123`。
这一步验证串口收发。速度指令、轮速与编码器反馈、故障状态和失联停车需要在电机控制固件中实现。

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
