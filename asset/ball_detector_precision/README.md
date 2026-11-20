# 小球检测模型

本目录提供单类别 `ball` 检测权重和图片检测脚本。默认每张原始图像推理一次，置信度阈值为 0.30，NMS 的 IoU 阈值为 0.7。模型预处理使用 `imgsz=960`，输出检测框采用原图像素坐标。

在本机 PowerShell 中运行：

```powershell
& "E:\conda\miniconda\python.exe" -P "E:\桌面\robodata\ball_detector_precision\detect.py" --source "图片文件或图片文件夹的完整路径"
```

结果保存在本目录的 detections 子目录，包括带框预览和 predictions.json。预览图包含绘制的框，不应用作训练原图。没有可用 GPU 时脚本会使用 CPU。

原有的翻转一致性模式在同场景的 26 张测试图中有 54 次检测，其中 53 次与人工框匹配、1 次未匹配。标注的 336 个球中有 283 个未检出。测试结果、数据来源和详细评估保存在 `robodata/pseudo_20261118/domain_precision_20261119`。当前的原图单次推理模式需要重新评估精确率和召回率。

请将本目录内的权重、配置和 Python 文件一起使用。
