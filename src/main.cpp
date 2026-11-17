#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "tools/exiter.hpp"

#if defined(BALL_WITH_HIKROBOT)
#include "io/hikrobot/hikrobot.hpp"
#include "tools/logger.hpp"
#endif

namespace
{
namespace fs = std::filesystem;

class FrameWriter
{
public:
  explicit FrameWriter(const fs::path & directory) : directory_(directory)
  {
    fs::create_directories(directory_);
    if (!fs::is_empty(directory_)) {
      throw std::runtime_error("输出目录已有内容，请使用另一个目录");
    }
    index_.open(directory_ / "frames.csv");
    index_.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    index_ << "frame,file,time_ms,width,height\n";
  }

  void write(const cv::Mat & image, double timestamp_ms)
  {
    const auto filename = cv::format("frame_%06d.png", count_ + 1);
    if (!cv::imwrite((directory_ / filename).string(), image)) {
      throw std::runtime_error("保存图片失败 " + filename);
    }
    ++count_;
    index_ << count_ << ',' << filename << ',';
    if (std::isfinite(timestamp_ms)) index_ << timestamp_ms;
    index_ << ',' << image.cols << ',' << image.rows << '\n';
  }

  int count() const { return count_; }
  void close() { index_.close(); }

private:
  fs::path directory_;
  std::ofstream index_;
  int count_ = 0;
};
}  // namespace

int main(int argc, char ** argv)
{
  const cv::String keys =
    "{help h usage ? | | 查看用法}"
    "{video | | 本地视频文件}"
    "{serial | | 海康 USB3 相机序列号}"
    "{output | | 输出目录}"
    "{frames | 100 | 最多保存的帧数}"
    "{exposure-ms | 8 | 相机曝光时间，毫秒}"
    "{gain | 8 | 相机增益}"
    "{fps | 15 | 相机目标帧率}";

  try {
    cv::CommandLineParser args(argc, argv, keys);
    if (argc == 1 || args.has("help")) {
      args.printMessage();
      return 0;
    }

    const auto video = args.get<cv::String>("video");
    const auto serial = args.get<cv::String>("serial");
    const auto output = args.get<cv::String>("output");
    const auto limit = args.get<int>("frames");
    const auto exposure_ms = args.get<double>("exposure-ms");
    const auto gain = args.get<double>("gain");
    const auto fps = args.get<double>("fps");
    if (!args.check()) {
      args.printErrors();
      return 2;
    }
    if (video.empty() == serial.empty() || output.empty() || limit < 1) {
      throw std::invalid_argument("请指定 --video 或 --serial、--output 和正数 --frames");
    }
    if (!serial.empty() && (!std::isfinite(exposure_ms) || exposure_ms <= 0 ||
        !std::isfinite(gain) || gain < 0 || !std::isfinite(fps) || fps <= 0)) {
      throw std::invalid_argument("曝光和帧率须为正数，增益须为非负数");
    }

    tools::Exiter exiter;
    if (!video.empty()) {
      cv::VideoCapture capture(video);
      if (!capture.isOpened()) throw std::runtime_error("无法打开视频 " + video);

      FrameWriter writer(output);
      cv::Mat image;
      while (!exiter.exit() && writer.count() < limit && capture.read(image)) {
        const double timestamp_ms = capture.get(cv::CAP_PROP_POS_MSEC);
        writer.write(image, timestamp_ms);
      }
      writer.close();
      std::cout << "保存了 " << writer.count() << " 帧\n";
      return writer.count() > 0 ? 0 : 1;
    }

#if defined(BALL_WITH_HIKROBOT)
    FrameWriter writer(output);
    fs::create_directories("logs");
    tools::logger();
    io::HikRobot camera(exposure_ms, gain, "collector", serial, fps);

    using Clock = std::chrono::steady_clock;
    cv::Mat image;
    Clock::time_point first_frame{};
    Clock::time_point timestamp{};
    while (!exiter.exit() && writer.count() < limit) {
      camera.read(image, timestamp);
      if (image.empty()) break;
      if (writer.count() == 0) first_frame = timestamp;
      const auto elapsed = std::chrono::duration<double, std::milli>(timestamp - first_frame);
      writer.write(image, elapsed.count());
    }
    writer.close();
    std::cout << "保存了 " << writer.count() << " 帧\n";
    return writer.count() > 0 ? 0 : 1;
#else
    throw std::runtime_error("海康模式需要在 Linux aarch64 上启用 BALL_WITH_HIKROBOT");
#endif
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
