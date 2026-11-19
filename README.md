# RM-ball-sweeper

将 RoboMaster 比赛场地中遗留的 17 mm 小弹丸进行自动追踪和清扫

## 工程结构

```text
src/main.cpp                  ball_capture，现场采图或从视频抽帧
src/collector_main.cpp        ball_collector，运行状态机演示
io/hikrobot/                  从原 RM 工程迁入的海康相机驱动
io/chassis/velocity_command.hpp  车体速度接口，供任务层和底盘通信共用
tasks/ball_collect/           五状态的采球状态机
tools/                        日志、退出、队列工具和 AprilTag 位姿解算
```

状态机沿用 RM 工程把任务状态集中处理的做法。采球任务用
`Idle`、`Search`、`Approach`、`Collect` 和 `Fault` 五个状态。
每次调用 `step()` 输入当前状态和观测，返回下一状态和一条速度指令。
状态切换时输出零速度，短暂丢球时保持接近状态并停车，持续丢球时回到搜索。
运动中相机或底盘反馈过期会进入故障停机。两路反馈恢复后需要手动复位。

接入实际数据时，在任务层添加 YOLO 推理、地面定位和速度计算。
推理层输出原图球心和帧时间戳，目标确认逻辑按不同帧更新
`TargetStatus`。控制循环根据观测和反馈的时间戳设置
`vision_fresh` 和 `chassis_fresh`。收球机构的完成或超时由上层反馈为
`CollectionStatus`。底盘协议确认后，在 `io/chassis/` 接入发送和反馈。
速度指令在软件内部约定前进、左移、逆时针为正，单位为 `m/s`、`m/s`、`rad/s`。

## 运行状态机演示

只构建任务入口时需要 C++17 编译器和 CMake。演示使用模拟输入并打印状态和速度，便于在没有驱动板的电脑上检查逻辑。

```bash
cmake -S . -B build-collector -DBALL_BUILD_CAPTURE=OFF
cmake --build build-collector -j2
./build-collector/ball_collector --demo
```

预期能看到搜索、接近、收球和恢复搜索。模拟图像过期后，进入 `Fault` 并输出零速度。恢复图像后，演示会在手动复位事件到来时回到 `Idle`。

## 采球车图像采集

第一阶段支持在 RK3576 Ubuntu 上从单台海康 USB3 工业相机采图，也支持从本地视频抽取连续帧。程序保存原分辨率 PNG 和 `frames.csv`，可直接检查 17 mm 小球的可见范围，或用这些图片做检测标注。

`io/hikrobot/`、`io/camera.hpp`、所需的 `tools/` 文件和许可证来自 RoboMaster 源码，按原文件复制。新项目的 `src/camera_base.cpp` 提供相机基类的原有默认实现，以便独立链接海康驱动。

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
