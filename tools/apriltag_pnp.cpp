#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#if CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 7
#include <opencv2/aruco.hpp>
#else
#include <opencv2/objdetect/aruco_detector.hpp>
#endif

#include "tools/exiter.hpp"

#if defined(BALL_WITH_HIKROBOT)
#include <filesystem>
#include <memory>

#include "io/hikrobot/hikrobot.hpp"
#include "tools/logger.hpp"
#endif

namespace
{
struct Calibration
{
  cv::Size size;
  cv::Mat camera_matrix;
  cv::Mat distortion;
};

Calibration load_calibration(const std::string & path)
{
  cv::FileStorage file(path, cv::FileStorage::READ);
  if (!file.isOpened()) throw std::runtime_error("无法读取相机标定文件 " + path);

  Calibration calibration;
  file["image_width"] >> calibration.size.width;
  file["image_height"] >> calibration.size.height;
  file["camera_matrix"] >> calibration.camera_matrix;
  file["distortion_coefficients"] >> calibration.distortion;

  auto & matrix = calibration.camera_matrix;
  const auto & distortion = calibration.distortion;
  if (calibration.size.width <= 0 || calibration.size.height <= 0 ||
      matrix.rows != 3 || matrix.cols != 3 || matrix.channels() != 1 ||
      distortion.channels() != 1 || (distortion.rows != 1 && distortion.cols != 1) ||
      !cv::checkRange(matrix) || !cv::checkRange(distortion)) {
    throw std::invalid_argument("标定文件需要有效的图像尺寸、3x3 内参和畸变系数向量");
  }
  const auto count = distortion.total();
  if (count != 4 && count != 5 && count != 8 && count != 12 && count != 14) {
    throw std::invalid_argument("畸变系数数量应为 4、5、8、12 或 14");
  }
  matrix.convertTo(matrix, CV_64F);
  if (matrix.at<double>(0, 0) <= 0 || matrix.at<double>(1, 1) <= 0) {
    throw std::invalid_argument("相机内参焦距须为正数");
  }
  calibration.distortion.convertTo(calibration.distortion, CV_64F);
  return calibration;
}

void write_pose(
  std::ostream & output, std::uint64_t frame, int id, const Calibration & calibration,
  const std::vector<cv::Point3d> & tag_corners, const std::vector<cv::Point2f> & image_corners)
{
  cv::Vec3d rotation, translation;
  // The planar homography initialization handles the front-on tag case.
  if (!cv::solvePnP(
      tag_corners, image_corners, calibration.camera_matrix, calibration.distortion,
      rotation, translation, false, cv::SOLVEPNP_ITERATIVE) ||
      !cv::checkRange(rotation) || !cv::checkRange(translation)) {
    return;
  }

  cv::Matx33d tag_to_camera;
  cv::Rodrigues(rotation, tag_to_camera);
  for (const auto & corner : tag_corners) {
    if ((tag_to_camera * cv::Vec3d(corner.x, corner.y, corner.z) + translation)[2] <= 0) return;
  }

  std::vector<cv::Point2d> projected;
  cv::projectPoints(
    tag_corners, rotation, translation, calibration.camera_matrix, calibration.distortion, projected);
  double squared_error = 0;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    const auto delta = projected[i] - cv::Point2d(image_corners[i]);
    squared_error += delta.dot(delta);
  }
  const auto rmse = std::sqrt(squared_error / projected.size());
  if (!std::isfinite(rmse)) return;

  const auto camera_in_tag = -(tag_to_camera.t() * translation);
  output << frame << ',' << id << ','
         << translation[0] << ',' << translation[1] << ',' << translation[2] << ','
         << rotation[0] << ',' << rotation[1] << ',' << rotation[2] << ','
         << camera_in_tag[0] << ',' << camera_in_tag[1] << ',' << camera_in_tag[2] << ','
         << rmse << std::endl;
}
}  // namespace

int main(int argc, char ** argv)
{
  const cv::String keys =
    "{help h usage ? | | 查看用法}"
    "{calibration | | OpenCV 相机标定文件}"
    "{tag-size-mm | 0 | 标签黑色外边框的实测边长，毫米}"
    "{image | | 离线图片文件}"
    "{device | | 摄像头索引或 Linux 视频设备路径}"
    "{serial | | 海康 USB3 相机序列号，默认 DA6415777}"
    "{exposure-ms | 8 | 海康曝光时间，毫秒}"
    "{gain | 8 | 海康增益}"
    "{fps | 15 | 海康目标帧率}"
    "{frames | 0 | 最多处理的相机帧数，0 表示持续运行}"
    "{output | | CSV 文件路径，默认输出到终端}";

  try {
    cv::CommandLineParser args(argc, argv, keys);
    if (argc == 1 || args.has("help")) {
      args.printMessage();
      return 0;
    }

    const auto calibration_path = args.get<cv::String>("calibration");
    const auto tag_size_mm = args.get<double>("tag-size-mm");
    const auto image_path = args.get<cv::String>("image");
    const auto device = args.get<cv::String>("device");
    const auto serial = args.get<cv::String>("serial");
    const auto frame_limit = args.get<int>("frames");
    const auto output_path = args.get<cv::String>("output");
    if (!args.check()) {
      args.printErrors();
      return 2;
    }
    const double half = tag_size_mm / 2000.0;
    if (calibration_path.empty() || !std::isfinite(half) || half <= 0 || frame_limit < 0) {
      throw std::invalid_argument("请传入 --calibration、正数 --tag-size-mm 和非负 --frames");
    }
    if ((!image_path.empty() && !device.empty()) ||
        (!serial.empty() && (!image_path.empty() || !device.empty()))) {
      throw std::invalid_argument("--image、--device 和 --serial 只能指定其中一种");
    }

    const auto calibration = load_calibration(calibration_path);
    // Match the detector's decoded corner order: TL, TR, BR, BL.
    const std::vector<cv::Point3d> tag_corners = {
      {-half, half, 0}, {half, half, 0}, {half, -half, 0}, {-half, -half, 0}};

    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
#if CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 7
    const auto parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#else
    cv::aruco::DetectorParameters parameters;
    parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    const cv::aruco::ArucoDetector detector(dictionary, parameters);
#endif

    tools::Exiter exiter;
    cv::Mat image;
    cv::VideoCapture capture;
#if defined(BALL_WITH_HIKROBOT)
    std::unique_ptr<io::HikRobot> hik_camera;
#endif
    if (!image_path.empty()) {
      image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
      if (image.empty()) throw std::runtime_error("无法读取图片 " + image_path);
    } else if (!device.empty()) {
      if (device.find_first_not_of("0123456789") == std::string::npos) {
        capture.open(std::stoi(device));
      } else {
        capture.open(device, cv::CAP_V4L2);
      }
      if (!capture.isOpened()) throw std::runtime_error("无法打开摄像头设备 " + device);
      capture.set(cv::CAP_PROP_FRAME_WIDTH, calibration.size.width);
      capture.set(cv::CAP_PROP_FRAME_HEIGHT, calibration.size.height);
    } else {
#if defined(BALL_WITH_HIKROBOT)
      const auto exposure_ms = args.get<double>("exposure-ms");
      const auto gain = args.get<double>("gain");
      const auto fps = args.get<double>("fps");
      if (!args.check()) {
        args.printErrors();
        return 2;
      }
      if (!std::isfinite(exposure_ms) || exposure_ms <= 0 || !std::isfinite(gain) || gain < 0 ||
          !std::isfinite(fps) || fps <= 0) {
        throw std::invalid_argument("海康曝光和帧率须为正数，增益须为非负数");
      }
      std::filesystem::create_directories("logs");
      tools::logger();
      hik_camera = std::make_unique<io::HikRobot>(
        exposure_ms, gain, "tags", serial.empty() ? "DA6415777" : serial, fps);
#else
      throw std::runtime_error("请指定 --device 或 --image，海康模式需要启用 BALL_WITH_HIKROBOT");
#endif
    }

    std::ofstream csv;
    if (!output_path.empty()) {
      csv.exceptions(std::ofstream::failbit | std::ofstream::badbit);
      csv.open(output_path);
    }
    std::ostream & output = output_path.empty() ? std::cout : csv;
    output << std::fixed << std::setprecision(8)
           << "frame,tag_id,x_m,y_m,z_m,rx_rad,ry_rad,rz_rad,"
           << "camera_x_in_tag_m,camera_y_in_tag_m,camera_z_in_tag_m,reprojection_rmse_px"
           << std::endl;

    const auto limit = image_path.empty() ? static_cast<std::uint64_t>(frame_limit) : 1;
    for (std::uint64_t frame = 1; !exiter.exit() && (limit == 0 || frame <= limit); ++frame) {
#if defined(BALL_WITH_HIKROBOT)
      if (hik_camera) {
        std::chrono::steady_clock::time_point timestamp;
        hik_camera->read(image, timestamp);
      }
#endif
      if (capture.isOpened()) capture.read(image);
      if (exiter.exit()) break;
      if (image.empty()) throw std::runtime_error("摄像头取帧失败");
      if (image.size() != calibration.size) {
        throw std::runtime_error(cv::format(
          "图像尺寸 %dx%d 与标定尺寸 %dx%d 不匹配", image.cols, image.rows,
          calibration.size.width, calibration.size.height));
      }

      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<int> ids;
#if CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR < 7
      cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);
#else
      detector.detectMarkers(image, corners, ids);
#endif
      for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] >= 0 && ids[i] <= 9) {
          write_pose(output, frame, ids[i], calibration, tag_corners, corners[i]);
        }
      }
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
