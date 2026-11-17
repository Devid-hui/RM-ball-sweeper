# RM-ball-sweeper

将 RoboMaster 比赛场地中遗留的 17 mm 小弹丸进行自动追踪和清扫

## 采球车图像采集

第一阶段支持在 RK3576 Ubuntu 上从单台海康 USB3 工业相机采图，也支持从本地视频抽取连续帧。程序保存原分辨率 PNG 和 `frames.csv`，可直接检查 17 mm 小球的可见范围，或用这些图片做检测标注。

`io/hikrobot/`、`io/camera.hpp`、所需的 `tools/` 文件和许可证来自 RoboMaster 源码，按原文件复制。新项目的 `src/camera_base.cpp` 提供相机基类的原有默认实现，以便独立链接海康驱动。

## Ubuntu 上连接海康相机

本阶段使用原工程的 USB3 枚举和采集驱动。相机序列号可从机身铭牌或 MVS 客户端读取。若相机标签显示 GigE 或网口连接，需要对照型号选用对应的 MVS 传输方式。

```bash
sudo apt install g++ cmake libopencv-dev libfmt-dev libspdlog-dev
cmake -S . -B build-hik -DBALL_WITH_HIKROBOT=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-hik -j2
./build-hik/ball_capture --serial=你的相机序列号 --output=captures/field-01 --frames=100
```

`--exposure-ms=8 --gain=8 --fps=15` 是采集入口的起始参数，可结合实际光照和相机型号调整。文件中的时间从第一张保存的图像开始，来自采集回调的单调时钟。保存图片慢于相机取帧时，原驱动会保留最新帧，连续保存的图片可能跳过部分相机帧。看不到首帧时，先检查 MVS 是否可枚举相机及当前用户是否有设备访问权限，再对照程序的相机日志。按 Ctrl+C 可退出。

## 从本地视频提取图片

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./build/ball_capture --video="/path/to/field.mp4" --output=captures/replay-01 --frames=100
```

视频时间由 OpenCV 视频后端提供。每次使用新的输出目录，保留不同场次的数据。
