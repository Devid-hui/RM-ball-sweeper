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
tools/                        日志、退出、队列工具和 AprilTag 位姿解算
```

任务层的 `decide(input)` 每次根据当前输入返回速度和原因。急停、收球失败、
缺少运动许可或反馈过期时输出零速度。目标短暂丢失时停车，确认失去目标后恢复搜索。
目标对齐后执行收球，机构报告完成时停车。

运动许可由上层持续提供。收到 `revoke_motion_authorization` 后，上层立即撤销许可，
确认故障解除并收到人工授权后才重新允许移动。
接入实车时，目标确认与地面定位模块提供观测和速度建议，底盘协议接入 `io/chassis/`。
速度约定前进、左移、逆时针为正，单位为 `m/s`、`m/s`、`rad/s`。

## 运行决策演示

只构建任务入口时需要 C++17 编译器和 CMake。演示使用模拟输入并打印决策原因和速度。

```bash
cmake -S . -B build-collector -DBALL_BUILD_CAPTURE=OFF
cmake --build build-collector -j2
./build-collector/ball_collector --demo
```

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

标定参考放在 `configs/hikrobot_DA6415777.yaml`。内参来自 RM 的 `sentry_example.yaml` 中对应的 `omni1`。接入地面定位前，先确认标定分辨率、裁剪和镜头对焦状态与当前采集一致，并用棋盘格或已知点位复核。相机到车体的外参按采球车实际安装重新测量。

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

## AprilTag 位姿解算

`apriltag_pnp` 对 `tag36h11` 的 ID 0–5 解算位姿，对应 `tag36h11 IDs 0-9.pdf` 的前 6 页。按实际大小打印，保留四周白边，然后用尺量黑色外边框的边长，单位为毫米。PDF 中的设计边长约 100 mm，命令里的 `--tag-size-mm` 以实测为准。同一批处理的标签应使用相同边长。

在 RK3576 Ubuntu 上，使用 `libopencv-dev`（OpenCV 4.6 及更早版本还需要 `libopencv-contrib-dev`）、`libfmt-dev` 和 `libspdlog-dev` 构建海康取图模式。下例沿用项目记录的海康序列号。如果 `tags` 使用另一台海康相机，填写它的实际序列号。

```bash
cmake -S . -B build-tags -DBALL_BUILD_CAPTURE=OFF -DBALL_BUILD_APRILTAG_PNP=ON -DBALL_WITH_HIKROBOT=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-tags --target apriltag_pnp -j2
./build-tags/apriltag_pnp --serial=DA6415777 --calibration=configs/tags_calibration.yaml --tag-size-mm=100
```

如果 `tags` 是系统视频设备，把 `--serial=DA6415777` 换成 `--device=/dev/tags`（假设这是实际设备路径），也可以用 `--device=0` 指定 OpenCV 摄像头索引。离线调试时换成 `--image=frame.png`。这两种模式构建时可将 `BALL_WITH_HIKROBOT` 设为 `OFF`。相机模式默认持续处理，可用 `--frames=100` 限制帧数或按 Ctrl+C 退出。

将 `tags` 相机在实际分辨率、裁剪和对焦设置下的针孔标定结果保存为 OpenCV YAML，例如 `configs/tags_calibration.yaml`。下方数字仅用于说明格式。仓库现有的 `configs/hikrobot_DA6415777.yaml` 是待核验的内参参考，缺少标定分辨率。

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
