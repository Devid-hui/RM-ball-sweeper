"""用颜色和轮廓的圆形填充率筛选小球。"""
from math import pi

import cv2


class HsvBallDetector:
    def __init__(self, config):
        self.lower = tuple(config["hsv_lower"])
        self.upper = tuple(config["hsv_upper"])
        if len(self.lower) != 3 or len(self.upper) != 3:
            raise ValueError("HSV 上下界各需三个整数")
        for low, high, limit in zip(self.lower, self.upper, (179, 255, 255)):
            if type(low) is not int or type(high) is not int or not 0 <= low <= high <= limit:
                raise ValueError("HSV 上下界超出 OpenCV H、S、V 范围")

    def detect(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.medianBlur(cv2.inRange(hsv, self.lower, self.upper), 3)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes, fill_ratios = [], []
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            if radius < 4:
                continue
            fill = cv2.contourArea(contour) / (pi * radius ** 2)
            if fill >= 0.66:
                boxes.append((x - radius, y - radius, x + radius, y + radius))
                fill_ratios.append(fill)
        return boxes, fill_ratios
