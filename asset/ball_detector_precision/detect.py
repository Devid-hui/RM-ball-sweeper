"""使用已选高精确率模型检测本地图片并保存预览。"""
from pathlib import Path
from datetime import datetime
import argparse
import hashlib
import json
import sys
import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageOps
from torchvision.ops import nms
from ultralytics import YOLO

# 显式允许加载同目录的项目模块，兼容 Python 安全路径模式。
sys.path.insert(0,str(Path(__file__).resolve().parent))
from infer_consistency import normalized_boxes
from precision_flip import consensus_boxes

ROOT = Path(__file__).resolve().parent
EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp'}

def main():
    """读取图片，使用验证集选出的置信度和固定 NMS 进行检测。"""
    parser = argparse.ArgumentParser(description='检测图片中的小球')
    parser.add_argument('--source', required=True, help='图片或图片文件夹')
    parser.add_argument('--output', help='新建结果目录，默认保存在当前项目中')
    args = parser.parse_args()
    source = Path(args.source)
    if source.is_file():
        images = [source] if source.suffix.lower() in EXTENSIONS else []
    elif source.is_dir():
        images = sorted(path for path in source.iterdir() if path.is_file() and path.suffix.lower() in EXTENSIONS)
    else:
        raise FileNotFoundError(f'找不到图片或目录: {source}')
    if not images:
        raise ValueError('没有找到可检测图片')
    if len({path.name.casefold() for path in images}) != len(images):
        raise ValueError('图片文件名重复')
    config = json.loads((ROOT/'selected_config.json').read_text(encoding='utf-8'))
    weights = ROOT/'selected_model.pt'
    if hashlib.sha256(weights.read_bytes()).hexdigest() != config['sha256']:
        raise ValueError('模型与已校准的检测配置不一致')
    mode=config['inference_mode']
    if mode not in ('single','flip_consensus') or config['nms_iou']!=0.7 or config['flip_match_iou']!=0.5:
        raise ValueError('模型检测方式或阈值配置不受支持')
    output = Path(args.output) if args.output else ROOT/'detections'/datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    previews = output/'previews'
    previews.mkdir()
    model = YOLO(str(weights))
    device = 0 if torch.cuda.is_available() else 'cpu'
    records=[]
    for start in range(0,len(images),8):
        batch=images[start:start+8]
        if mode=='flip_consensus':
            originals=[]
            for path in batch:
                image=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f'无法读取图片: {path}')
                originals.append(image)
            predictions=model.predict(source=originals,imgsz=config['imgsz'],
                                      conf=0.10,batch=8,device=device,verbose=False)
            flipped=model.predict(source=[cv2.flip(image,1) for image in originals],imgsz=config['imgsz'],
                                  conf=0.10,batch=8,device=device,verbose=False)
            if len(flipped)!=len(batch):
                raise RuntimeError('翻转检测结果数量不匹配')
        else:
            predictions=model.predict(source=[str(path) for path in batch],imgsz=config['imgsz'],
                                      conf=0.10,batch=8,device=device,verbose=False)
            flipped=[None]*len(batch)
        if len(predictions)!=len(batch):
            raise RuntimeError('检测结果数量不匹配')
        for path,result,flipped_result in zip(batch,predictions,flipped):
            if mode=='flip_consensus':
                detected=consensus_boxes(normalized_boxes(result,False),normalized_boxes(flipped_result,True),config['confidence'])
            else:
                boxes=result.boxes.cpu()
                indices=nms(boxes.xyxy,boxes.conf,config['nms_iou']).tolist() if len(boxes) else []
                detected=[{'xyxy':[float(value) for value in boxes.xyxy[index]],
                           'confidence':float(boxes.conf[index])} for index in indices
                           if float(boxes.conf[index])>=config['confidence']]
            with Image.open(path) as original:
                preview=ImageOps.exif_transpose(original).convert('RGB')
            if preview.size != (result.orig_shape[1],result.orig_shape[0]):
                raise ValueError(f'预测图像尺寸与预览不一致: {path}')
            draw=ImageDraw.Draw(preview)
            for item in detected:
                x1,y1,x2,y2=item['xyxy']
                score=item['confidence']
                draw.rectangle((x1,y1,x2,y2),outline='red',width=2)
                draw.text((x1,max(0,y1-12)),f'{score:.2f}',fill='yellow')
            preview.save(previews/path.name)
            records.append({'image':str(path),'detections':detected})
    (output/'predictions.json').write_text(json.dumps({'config':config,'results':records},ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'完成 {len(records)} 张图片，结果保存在 {output}')

if __name__ == '__main__':
    main()
