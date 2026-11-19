# 小球检测模型

本目录提供单类别 ball 检测权重和图片检测脚本。默认使用 0.30 置信度、0.7 NMS 和原图/水平翻转结果一致性筛选。脚本会对结果进行与测试一致的后处理。

在本机 PowerShell 中运行：

```powershell
& "E:\conda\miniconda\python.exe" -P "E:\桌面\robodata\ball_detector_precision\detect.py" --source "图片文件或图片文件夹的完整路径"
```

结果保存在本目录的 detections 子目录，包括带框预览和 predictions.json。预览图包含绘制的框，不应用作训练原图。没有可用 GPU 时脚本会使用 CPU。

当前模型在同场景的 26 张测试图中有 54 次检测，其中 53 次与人工框匹配、1 次未匹配；标注的 336 个球中有 283 个未检出。此结果不能保证其他场景没有误检。原测试集和新测试集结果、数据来源以及详细评估保存在 robodata/pseudo_20261118/domain_precision_20261119。

请保持本目录内模型、配置和 Python 文件一起使用。直接加载权重但不运行检测脚本，不会自动执行翻转一致性筛选。
