"""用墙标建立地面坐标系。墙外为 X 正，沿墙向标签右侧为 Y 正。"""
from math import atan2

import cv2
import numpy as np


class WallTag:
    def __init__(self, camera, tag_id, size_m):
        self.camera = camera
        self.tag_id = tag_id
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            cv2.aruco.DetectorParameters(),
        )
        half = size_m / 2
        self.corners = np.array([[-half, half, 0], [half, half, 0],
                                 [half, -half, 0], [-half, -half, 0]])

    def pose(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        matches = [quad for quad, tag_id in zip(corners, ids.flat) if tag_id == self.tag_id]
        if len(matches) != 1:
            return None
        pixels = matches[0].reshape(4, 2)
        if np.any(pixels < 2) or np.any(pixels >= np.array(self.camera.image_size) - 2):
            return None
        camera = self.camera
        ok, rvec, translation = cv2.solvePnP(
            self.corners, pixels, camera.matrix, camera.distortion, flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or not np.isfinite(rvec).all() or not np.isfinite(translation).all():
            return None
        tag_to_camera = cv2.Rodrigues(rvec)[0]
        if np.any((self.corners @ tag_to_camera.T + translation.ravel())[:, 2] <= 0):
            return None
        projected = cv2.projectPoints(
            self.corners, rvec, translation, camera.matrix, camera.distortion,
        )[0].reshape(4, 2)
        if np.sqrt(np.mean(np.sum((projected - pixels) ** 2, axis=1))) > 2.5:
            return None
        chassis_to_tag = tag_to_camera.T @ camera.rotation.T
        origin = -tag_to_camera.T @ translation.ravel() - chassis_to_tag @ camera.origin
        forward = chassis_to_tag[:, 0]
        if origin[2] <= 0 or np.hypot(forward[0], forward[2]) < 0.8:
            return None
        return float(origin[2]), float(origin[0]), atan2(forward[0], forward[2])
