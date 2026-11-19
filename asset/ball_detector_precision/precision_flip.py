"""评估原图与水平翻转图一致检测的精确率。"""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import torch
from torchvision.ops import box_iou
from ultralytics import YOLO

# 显式允许加载同目录的项目模块，兼容 Python 安全路径模式。
sys.path.insert(0,str(Path(__file__).resolve().parent))
from infer_consistency import normalized_boxes, match_views
from precision_metrics import THRESHOLDS

def consensus_boxes(original, flipped, threshold):
    """保留两个视图均达阈值且一对一重合的检测框。"""
    first=[item for item in original if item['confidence']>=threshold]
    second=[item for item in flipped if item['confidence']>=threshold]
    pairs,_,_=match_views(first,second)
    result=[]
    for pair in pairs:
        left=first[pair['original_index']]
        right=second[pair['flipped_index']]
        result.append({'xyxy':left['xyxy'],'confidence':min(left['confidence'],right['confidence'])})
    return sorted(result,key=lambda item:-item['confidence'])

def measure_flip(weights, image_dir, label_dir):
    """逐阈值统计双视图一致检测与人工标注的匹配结果。"""
    images=sorted(image_dir.glob('*.jpg'))
    if not images:
        raise ValueError(f'图片目录为空: {image_dir}')
    model=YOLO(str(weights))
    statistics={threshold:{'tp':0,'fp':0,'fn':0,'background_images':0,'background_images_with_detections':0} for threshold in THRESHOLDS}
    for start in range(0,len(images),8):
        batch=images[start:start+8]
        originals=[]
        for path in batch:
            image=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f'无法读取图片: {path}')
            originals.append(image)
        first=model.predict(source=originals,imgsz=960,conf=0.10,batch=8,device=0,verbose=False)
        second=model.predict(source=[cv2.flip(image,1) for image in originals],imgsz=960,conf=0.10,batch=8,device=0,verbose=False)
        if len(first)!=len(batch) or len(second)!=len(batch):
            raise RuntimeError('预测结果数量不匹配')
        for image,orig_result,flip_result in zip(batch,first,second):
            height,width=orig_result.orig_shape
            truth=[]
            label=label_dir/f'{image.stem}.txt'
            if not label.is_file():
                raise FileNotFoundError(label)
            for line in label.read_text(encoding='utf-8-sig').splitlines():
                category,x,y,w,h=map(float,line.split())
                if category!=0:
                    raise ValueError(f'类别错误: {label}')
                truth.append([(x-w/2)*width,(y-h/2)*height,(x+w/2)*width,(y+h/2)*height])
            ground=torch.tensor(truth,dtype=torch.float32).reshape(-1,4)
            a=normalized_boxes(orig_result,False)
            b=normalized_boxes(flip_result,True)
            for threshold,counts in statistics.items():
                paired=consensus_boxes(a,b,threshold)
                boxes=torch.tensor([item['xyxy'] for item in paired],dtype=torch.float32).reshape(-1,4)
                overlap=box_iou(boxes,ground) if len(boxes) and len(ground) else torch.empty((len(boxes),len(ground)))
                matched=set()
                for i in range(len(boxes)):
                    available=[j for j in range(len(ground)) if j not in matched]
                    if available:
                        best=max(available,key=lambda j:float(overlap[i,j]))
                        if float(overlap[i,best])>=0.5:
                            matched.add(best)
                            counts['tp']+=1
                            continue
                    counts['fp']+=1
                counts['fn']+=len(ground)-len(matched)
                if not len(ground):
                    counts['background_images']+=1
                    counts['background_images_with_detections']+=bool(len(boxes))
    for counts in statistics.values():
        tp,fp,fn=counts['tp'],counts['fp'],counts['fn']
        counts['detections']=tp+fp
        counts['precision']=tp/(tp+fp) if tp+fp else None
        counts['recall']=tp/(tp+fn) if tp+fn else None
    return statistics
