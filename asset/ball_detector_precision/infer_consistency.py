"""使用水平翻转一致性检查，为自动伪标签训练保留候选证据。"""
from pathlib import Path
from collections import Counter
import json
import cv2
import numpy as np
import torch
from scipy.optimize import linear_sum_assignment
from torchvision.ops import box_iou, nms
from ultralytics import YOLO

ROOT = Path('E:/桌面/robodata')
OUT = ROOT/'pseudo_20261118'/'automatic_round_20261119'
MODEL = ROOT/'pseudo_20261118'/'reviewed_training'/'runs'/'reviewed'/'weights'/'best.pt'

def normalized_boxes(result, flipped):
    """去重并把翻转图片的框还原到原图坐标。"""
    boxes = result.boxes.cpu()
    if not len(boxes):
        return []
    indices = nms(boxes.xyxy, boxes.conf, 0.7).tolist()
    width = result.orig_shape[1]
    output=[]
    for index in indices:
        x1,y1,x2,y2 = [float(x) for x in boxes.xyxy[index]]
        if flipped:
            x1,x2=width-x2,width-x1
        output.append({'xyxy':[x1,y1,x2,y2], 'confidence':float(boxes.conf[index])})
    return output

def match_views(original, flipped):
    """按 IoU 一对一匹配两次预测，返回匹配及不一致情况。"""
    if not original or not flipped:
        return [], list(range(len(original))), list(range(len(flipped)))
    a=torch.tensor([item['xyxy'] for item in original],dtype=torch.float32)
    b=torch.tensor([item['xyxy'] for item in flipped],dtype=torch.float32)
    overlap=box_iou(a,b).numpy()
    # 优先使 IoU 达标的配对数量最大，再在同数量配对中优先选择更高 IoU。
    bonus=len(original)+len(flipped)+1
    utility=np.where(overlap>=0.5,bonus+overlap,0.0)
    rows,columns=linear_sum_assignment(-utility)
    pairs=[]
    for i,j in zip(rows,columns):
        if overlap[i,j]>=0.5:
            pairs.append({'original_index':int(i),'flipped_index':int(j),'iou':float(overlap[i,j])})
    used_a={item['original_index'] for item in pairs}
    used_b={item['flipped_index'] for item in pairs}
    return pairs,[i for i in range(len(original)) if i not in used_a],[i for i in range(len(flipped)) if i not in used_b]

def main():
    """读取本地候选并对原图、翻转图分别推理。"""
    destination=OUT/'consistency_predictions.json'
    if destination.exists():
        raise FileExistsError(destination)
    candidates=json.loads((OUT/'eligible.json').read_text(encoding='utf-8'))
    model=YOLO(str(MODEL))
    records=[]
    for start in range(0,len(candidates),8):
        batch=candidates[start:start+8]
        originals=[]
        for item in batch:
            path=Path(item['image'])
            image=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f'无法读取图片: {path}')
            originals.append(image)
        result_a=model.predict(source=originals,imgsz=960,conf=0.15,batch=8,device=0,verbose=False)
        result_b=model.predict(source=[cv2.flip(image,1) for image in originals],imgsz=960,conf=0.15,batch=8,device=0,verbose=False)
        if len(result_a)!=len(batch) or len(result_b)!=len(batch):
            raise RuntimeError('推理输出数量不匹配')
        for item,ra,rb in zip(batch,result_a,result_b):
            a=normalized_boxes(ra,False);b=normalized_boxes(rb,True)
            pairs,unmatched_a,unmatched_b=match_views(a,b)
            records.append({**item,'width':ra.orig_shape[1],'height':ra.orig_shape[0],
                            'original':a,'flipped':b,'matches':pairs,
                            'unmatched_original':unmatched_a,'unmatched_flipped':unmatched_b})
        print(f'processed {min(start+8,len(candidates))}/{len(candidates)}',flush=True)
    destination.write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    stats=Counter()
    for record in records:
        if not record['original'] and not record['flipped']:
            stats['neither_view_detected']+=1
        if record['matches'] and not record['unmatched_original'] and not record['unmatched_flipped']:
            stats['fully_matched']+=1
            if all(min(record['original'][p['original_index']]['confidence'],record['flipped'][p['flipped_index']]['confidence'])>=0.25 for p in record['matches']):
                stats['fully_matched_min_conf_025']+=1
            if all(min(record['original'][p['original_index']]['confidence'],record['flipped'][p['flipped_index']]['confidence'])>=0.35 for p in record['matches']):
                stats['fully_matched_min_conf_035']+=1
    stats['total']=len(records)
    (OUT/'consistency_summary.json').write_text(json.dumps(stats,indent=2),encoding='utf-8')
    print(json.dumps(stats,indent=2),flush=True)

if __name__ == '__main__':
    raise SystemExit('本文件提供一致性检测函数，请使用 detect.py 运行图片检测。')
