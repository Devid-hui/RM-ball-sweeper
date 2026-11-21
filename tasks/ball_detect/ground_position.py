"""用已标定的相机估计地面小球在车体坐标系中的位置。"""
from math import cos, hypot, isfinite, radians, sin

import cv2
import numpy as np


BALL_RADIUS_M = 0.0085
# OpenCV 的右、下、前映射到车体的前、左、上。
LEVEL_CAMERA_TO_CHASSIS = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def camera_rotation(yaw_deg, pitch_down_deg, roll_deg):
    yaw, pitch, roll = map(radians, (yaw_deg, pitch_down_deg, roll_deg))
    turn = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    tilt = np.array([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    bank = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    return turn @ tilt @ bank @ LEVEL_CAMERA_TO_CHASSIS


def read_real(section, key):
    node = section.getNode(key)
    if not (node.isReal() or node.isInt()) or not isfinite(node.real()):
        raise ValueError(f"标定项 {key} 需要填写有限数值")
    return node.real()


class GroundProjector:
    def __init__(self, calibration_path, max_distance_m=1.0):
        if not isfinite(max_distance_m) or max_distance_m <= 0:
            raise ValueError("定位范围须为正数")
        self.max_distance_m = max_distance_m
        file = cv2.FileStorage()
        try:
            if not file.open(str(calibration_path), cv2.FILE_STORAGE_READ):
                raise ValueError(f"无法读取标定文件 {calibration_path}")
            width = read_real(file, "image_width")
            height = read_real(file, "image_height")
            self.matrix = file.getNode("camera_matrix").mat()
            self.distortion = file.getNode("distortion_coefficients").mat()
            self.minimum_candidate_forward_m = 0.0
            if not file.getNode("minimum_candidate_forward_m").empty():
                self.minimum_candidate_forward_m = read_real(file, "minimum_candidate_forward_m")
            pose = file.getNode("camera_to_chassis")
            if not pose.isMap():
                raise ValueError("标定文件须填写 camera_to_chassis")
            self.origin = np.array([read_real(pose, axis) for axis in ("x_m", "y_m", "z_m")])
            self.rotation = camera_rotation(
                read_real(pose, "yaw_deg"), read_real(pose, "pitch_down_deg"),
                read_real(pose, "roll_deg"),
            )
        except (cv2.error, SystemError) as error:
            raise ValueError(f"标定文件格式无效 {calibration_path}") from error
        finally:
            file.release()
        if width <= 0 or height <= 0 or not width.is_integer() or not height.is_integer():
            raise ValueError("标定分辨率必须是正整数")
        self.image_size = int(width), int(height)
        if not 0 <= self.minimum_candidate_forward_m < self.max_distance_m:
            raise ValueError("近距筛选门限须大于等于零且小于定位范围")
        if (self.matrix is None or self.matrix.shape != (3, 3)
                or not np.isfinite(self.matrix).all()
                or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0):
            raise ValueError("camera_matrix 须为有效的 3×3 相机内参矩阵")
        if (self.distortion is None or self.distortion.ndim != 2 or 1 not in self.distortion.shape
                or self.distortion.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(self.distortion).all()):
            raise ValueError("distortion_coefficients 须为有效的 OpenCV 畸变系数向量")
        if self.origin[2] <= BALL_RADIUS_M:
            raise ValueError("相机光心高度必须大于球半径")
        self.matrix = self.matrix.astype(np.float64)
        self.distortion = self.distortion.astype(np.float64)

    def position(self, box, *, top_edge=0):
        left, top, right, bottom = box
        width, height = self.image_size
        if not (0 < left < right < width and top_edge < top < bottom < height):
            return None  # 裁断的框无法可靠估计球心。
        center = np.array([[[(left + right) / 2, (top + bottom) / 2]]], dtype=np.float64)
        normalized_x, normalized_y = cv2.undistortPoints(center, self.matrix, self.distortion)[0, 0]
        ray = self.rotation @ np.array([normalized_x, normalized_y, 1.0])
        if not np.isfinite(ray).all() or ray[2] >= -1e-9:
            return None
        # 球心位于离地一个半径的平面上，用框中心的射线估算球心。
        depth = (BALL_RADIUS_M - self.origin[2]) / ray[2]
        forward, leftward = (self.origin + depth * ray)[:2]
        distance = hypot(forward, leftward)
        if forward <= 0 or distance > self.max_distance_m:
            return None
        return {"x_m": float(forward), "y_m": float(leftward), "distance_m": distance}

    def locate(self, record):
        size = record["width"], record["height"]
        if size != self.image_size:
            raise ValueError(f"输入图尺寸 {size} 与标定尺寸 {self.image_size} 不一致")
        nearest = None
        top_edge = record.get("roi_top_px", 0)
        for index, detection in enumerate(record["detections"]):
            position = self.position(detection["xyxy"], top_edge=top_edge)
            detection["position_m"] = position
            if position is None or position["x_m"] < self.minimum_candidate_forward_m:
                continue
            if nearest is None or position["distance_m"] < nearest["distance_m"]:
                nearest = {"detection_index": index, **position}
        record["target_candidate"] = nearest
