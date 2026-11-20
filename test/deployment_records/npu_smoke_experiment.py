import ctypes as ct
from pathlib import Path
import sys
import time
import json
import cv2
import numpy as np

here = Path(__file__).resolve().parent
baseline = Path.home() / 'ball-onnx-bench.Ng9tXC'
sys.path.insert(0, str(baseline))
from evaluate import match
cv2.setNumThreads(1)
native = ct.CDLL(str(here / 'trial_runtime.so'))
ctx = ct.c_uint64()
native.open_model.argtypes = [ct.c_char_p, ct.POINTER(ct.c_uint64)]
native.run_frame.argtypes = [ct.c_uint64, ct.c_void_p, ct.c_void_p]
native.rknn_destroy.argtypes = [ct.c_uint64]
model = Path.home() / 'ball-rknn-fp16.qJVwwA/ball-rk3576.rknn'
rc = native.open_model(str(model).encode(), ct.byref(ctx))
if rc:
    sys.exit(f'open_model failed: {rc}')
raw = np.empty((5,14490), np.float32)
def detect(image):
    if image.shape[:2] != (1080,1440):
        raise ValueError(image.shape)
    resized = cv2.resize(image, (960,720))
    rgb = cv2.cvtColor(cv2.copyMakeBorder(resized,8,8,0,0,cv2.BORDER_CONSTANT,value=(114,114,114)),cv2.COLOR_BGR2RGB)
    rc = native.run_frame(ctx, rgb.ctypes.data, raw.ctypes.data)
    if rc:
        raise RuntimeError(f'run_frame returned {rc}')
    rows = raw.T[raw[4] >= .30]
    rects = rows[:,:4].copy()
    rects[:,:2] -= rects[:,2:] / 2
    keep = np.asarray(cv2.dnn.NMSBoxes(rects.tolist(),rows[:,4].tolist(),0.,.7),dtype=int).ravel()
    boxes = rects[keep].copy()
    boxes[:,2:] += boxes[:,:2]
    boxes[:,::2] = np.clip(boxes[:,::2]*1.5,0,1440)
    boxes[:,1::2] = np.clip((boxes[:,1::2]-8)*1.5,0,1080)
    return boxes,rows[keep,4]
try:
    tp=fp=fn=hit=background_fp=0
    images=[]
    for row in json.loads((baseline/'test_manifest.json').read_text()):
        image=cv2.imread(str(baseline/row['image']))
        images.append(image)
        truth=np.array(row['boxes'],np.float32).reshape(-1,4)*[1440,1080,1440,1080]
        boxes,scores=detect(image)
        m=match(boxes,scores,truth)
        tp+=m
        fp+=len(boxes)-m
        fn+=len(truth)-m
        hit+=int(m>0)
        background_fp+=int(len(truth)==0 and len(boxes)>0)
    lat=[]
    start=time.monotonic()
    while time.monotonic()-start<45:
        tick=time.monotonic()
        detect(images[len(lat)%len(images)])
        lat.append(time.monotonic()-tick)
    result=dict(tp=tp,fp=fp,fn=fn,hit=hit,background_fp=background_fp,frames=len(lat),
                fps=len(lat)/(time.monotonic()-start),p95_ms=float(np.percentile(lat,95)*1000))
    (here/'result.json').write_text(json.dumps(result,indent=2))
    print(result)
finally:
    native.rknn_destroy(ctx)
