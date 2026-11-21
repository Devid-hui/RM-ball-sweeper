"""从棋盘格照片生成 OpenCV 相机内参文件。"""
import argparse
from math import isfinite
from pathlib import Path

import cv2
import numpy as np


def calibrate(paths, columns, rows, square_mm):
    corners_3d = np.zeros((rows * columns, 3), dtype=np.float32)
    corners_3d[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2) * square_mm
    object_points = []
    image_points = []
    image_size = None
    for path in paths:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
        if gray is None:
            raise ValueError(f"无法读取 {path}")
        size = gray.shape[1], gray.shape[0]
        if image_size is not None and size != image_size:
            raise ValueError(f"图像尺寸不同 {path} 为 {size}，预期 {image_size}")
        image_size = size
        found, corners = cv2.findChessboardCornersSB(gray, (columns, rows))
        if not found:
            print(f"跳过未识别到完整棋盘格的照片 {path}")
            continue
        object_points.append(corners_3d)
        image_points.append(corners)
    if len(image_points) < 3:
        raise ValueError("至少需要 3 张成功识别角点的不同角度照片")
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None,
    )
    return image_size, rms, matrix, distortion, len(image_points)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, nargs="+", required=True)
    parser.add_argument("--columns", type=int, required=True, help="棋盘格横向内角点数")
    parser.add_argument("--rows", type=int, required=True, help="棋盘格纵向内角点数")
    parser.add_argument("--square-mm", type=float, required=True, help="打印后实测的格子边长")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if min(args.columns, args.rows) < 3 or not isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error("内角点的行列数均须至少为 3，格子边长须为正数")
    try:
        size, rms, matrix, distortion, count = calibrate(
            args.images, args.columns, args.rows, args.square_mm,
        )
        # 使用内存生成 OpenCV YAML，再以独占创建模式写入，避免覆盖实测标定。
        file = cv2.FileStorage(".yaml", cv2.FILE_STORAGE_WRITE | cv2.FILE_STORAGE_MEMORY)
        file.write("image_width", size[0])
        file.write("image_height", size[1])
        file.write("camera_matrix", matrix)
        file.write("distortion_coefficients", distortion)
        with args.output.open("x") as output:
            output.write(file.releaseAndGetString())
    except (ValueError, OSError, cv2.error) as error:
        parser.error(str(error))
    print(f"使用 {count} 张图，重投影 RMS {rms:.3f} px，已保存到 {args.output}")


if __name__ == "__main__":
    main()
