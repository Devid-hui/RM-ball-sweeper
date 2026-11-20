from pathlib import Path
import json
import cv2
import numpy as np
import onnxruntime as ort
from rknn.api import RKNN

def boxes(raw):
    rows = raw[0].T
    rows = rows[rows[:, 4] >= .30]
    xywh = rows[:, :4].copy()
    xywh[:, :2] -= xywh[:, 2:] / 2
    keep = np.asarray(cv2.dnn.NMSBoxes(xywh.tolist(), rows[:, 4].tolist(), 0., .7),dtype=int).ravel()
    return np.round(np.column_stack([xywh[keep], rows[keep,4]]),3).tolist()

def check(result, operation):
    if result != 0:
        raise RuntimeError(f"{operation} returned {result}")

model=RKNN(verbose=False)
try:
    check(model.config(mean_values=[[0,0,0]], std_values=[[255,255,255]],target_platform='rk3576'),'config')
    check(model.load_onnx(model='ball_fp16.onnx'),'load')
    check(model.build(do_quantization=False),'build')
    check(model.export_rknn('ball-rk3576.rknn'),'export')
    check(model.init_runtime(),'simulator')
    ref=ort.InferenceSession('ball_fp16.onnx', providers=['CPUExecutionProvider'])
    records=[]
    for path in sorted(Path('.').glob('*.jpg')):
        original=cv2.imread(str(path))
        assert original.shape[:2]==(1080,1440)
        resized=cv2.resize(original,(960,720))
        padded=cv2.copyMakeBorder(resized,8,8,0,0,cv2.BORDER_CONSTANT,value=(114,114,114))
        rgb=cv2.cvtColor(padded,cv2.COLOR_BGR2RGB)[None]
        raw_ref=ref.run(None,{'images':np.ascontiguousarray(rgb.transpose(0,3,1,2),dtype=np.float32)/255})[0]
        raw=model.inference(inputs=[rgb],data_format=['nhwc'])[0]
        assert raw.shape==raw_ref.shape==(1,5,14490) and np.isfinite(raw).all()
        records.append(dict(image=path.name,onnx_xywh_score=boxes(raw_ref),sim_xywh_score=boxes(raw)))
    assert len(records)==2
    Path('sim_check.json').write_text(json.dumps(records, indent=2))
    print([(len(i['onnx_xywh_score']),len(i['sim_xywh_score'])) for i in records],flush=True)
finally:
    model.release()
