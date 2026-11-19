"""使用验证集为小球检测模型选择偏重精确率的置信度。"""
from pathlib import Path
import argparse
import json
import torch
from torchvision.ops import box_iou, nms
from ultralytics import YOLO

ROOT = Path('E:/桌面/robodata')
OUT = ROOT/'pseudo_20261118'/'domain_precision_20261119'
THRESHOLDS = [round(x/100,2) for x in range(10,96,5)]
MIN_PREDICTIONS = 20
TARGET_PRECISION = 0.99

def measure(weights, image_dir, label_dir):
    """按各阈值、一对一 IoU 0.5 匹配统计检测结果。"""
    images = sorted(image_dir.glob('*.jpg'))
    if not images:
        raise ValueError(f'图片目录为空: {image_dir}')
    model=YOLO(str(weights))
    statistics={threshold:{'tp':0,'fp':0,'fn':0,'background_images':0,'background_images_with_detections':0} for threshold in THRESHOLDS}
    for start in range(0,len(images),8):
        batch=images[start:start+8]
        results=model.predict(source=[str(p) for p in batch],imgsz=960,conf=0.10,batch=8,device=0,verbose=False)
        if len(results)!=len(batch):
            raise RuntimeError('预测结果数量不匹配')
        for image,result in zip(batch,results):
            height,width=result.orig_shape
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
            raw=result.boxes.cpu()
            keep=nms(raw.xyxy,raw.conf,0.7) if len(raw) else torch.empty(0,dtype=torch.long)
            boxes=raw.xyxy[keep];scores=raw.conf[keep]
            overlaps=box_iou(boxes,ground) if len(boxes) and len(ground) else torch.empty((len(boxes),len(ground)))
            for threshold,counts in statistics.items():
                used=set()
                n_detected=0
                for i in range(len(boxes)):
                    if float(scores[i])<threshold:
                        continue
                    n_detected+=1
                    candidates=[j for j in range(len(ground)) if j not in used]
                    if candidates:
                        best=max(candidates,key=lambda j:float(overlaps[i,j]))
                        if float(overlaps[i,best])>=0.5:
                            used.add(best)
                            counts['tp']+=1
                            continue
                    counts['fp']+=1
                counts['fn']+=len(ground)-len(used)
                if not len(ground):
                    counts['background_images']+=1
                    counts['background_images_with_detections']+=bool(n_detected)
    for threshold,counts in statistics.items():
        tp,fp,fn=counts['tp'],counts['fp'],counts['fn']
        counts['detections']=tp+fp
        counts['precision']=tp/(tp+fp) if tp+fp else None
        counts['recall']=tp/(tp+fn) if tp+fn else None
    return statistics

def select_threshold(statistics):
    """精确率达到目标时最大化检出数量，否则选精确率最高的阈值。"""
    eligible=[(threshold,counts) for threshold,counts in statistics.items() if counts['detections']>=MIN_PREDICTIONS]
    if not eligible:
        raise ValueError('验证集检测数量不足以选择阈值')
    high_precision=[(threshold,counts) for threshold,counts in eligible if counts['precision']>=TARGET_PRECISION]
    if high_precision:
        threshold,counts=max(high_precision,key=lambda item:(item[1]['tp'],item[1]['precision'],-item[0]))
        strategy='target_precision'
    else:
        threshold,counts=max(eligible,key=lambda item:(item[1]['precision'],item[1]['tp'],-item[0]))
        strategy='highest_precision'
    return {'threshold':threshold,'strategy':strategy,'metrics':counts}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--weights',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    statistics=measure(Path(args.weights),OUT/'dataset/images/val',OUT/'dataset/labels/val')
    result={'weights':args.weights,'selection':select_threshold(statistics),'thresholds':statistics,
            'validation_images':len(list((OUT/'dataset/images/val').glob('*.jpg'))),
            'min_predictions':MIN_PREDICTIONS,'target_precision':TARGET_PRECISION,
            'match_iou':0.5,'nms_iou':0.7}
    Path(args.output).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result['selection'],indent=2))

if __name__=='__main__':
    main()
