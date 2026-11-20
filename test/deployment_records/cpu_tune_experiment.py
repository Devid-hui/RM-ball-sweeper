import sys, os, json, time
from pathlib import Path
import numpy as np
import cv2
import onnxruntime as ort
import psutil
root=Path.home()/'ball-onnx-bench.Ng9tXC'
sys.path.insert(0,str(root))
import evaluate as ev
cv2.setNumThreads(1)
rows=json.loads((root/'test_manifest.json').read_text())
images=[ev.read_image(root/r['image']) for r in rows]
all_results=[]
for width,height in [(960,736),(800,608),(640,480)]:
    ev.HEIGHT,ev.WIDTH=height,width
    for threads in [2,4,8]:
        opts=ort.SessionOptions()
        opts.intra_op_num_threads=threads
        opts.inter_op_num_threads=1
        opts.add_session_config_entry('session.intra_op.allow_spinning','0')
        path=root/'ball_fp32.onnx' if width==960 else Path(__file__).parent/f'ball_{width}.onnx'
        session=ort.InferenceSession(str(path),opts,providers=['CPUExecutionProvider'])
        for _ in range(3): ev.detect(session,images[10])
        cpu0=sum(psutil.Process().cpu_times()[:2])
        timings=[]
        start=time.monotonic()
        while time.monotonic()-start<12:
            tick=time.monotonic()
            ev.detect(session,images[len(timings)%len(images)])
            timings.append(time.monotonic()-tick)
        elapsed=time.monotonic()-start
        record=dict(width=width,height=height,threads=threads,fps=len(timings)/elapsed,
                    p95_ms=float(np.percentile(timings,95)*1000),
                    cpu_pct=100*(sum(psutil.Process().cpu_times()[:2])-cpu0)/elapsed)
        if threads==4:
            metrics={str(c):dict(tp=0,fp=0,fn=0,hit=0,background_fp=0) for c in [.1,.2,.3]}
            for image,row in zip(images,rows):
                tensor, (scale,left,top,w,h)=ev.make_input(image)
                raw=session.run(None,{'images':tensor})[0][0]
                truth=np.array(row['boxes'],np.float32).reshape(-1,4)*[w,h,w,h]
                for conf,counts in metrics.items():
                    preds=raw[(raw[:,4]>=float(conf))&(raw[:,5]==0)]
                    boxes=preds[:,:4].copy()
                    scores=preds[:,4]
                    boxes[:,::2]=np.clip((boxes[:,::2]-left)/scale,0,w)
                    boxes[:,1::2]=np.clip((boxes[:,1::2]-top)/scale,0,h)
                    xywh=np.column_stack((boxes[:,:2],boxes[:,2:]-boxes[:,:2]))
                    keep=np.asarray(cv2.dnn.NMSBoxes(xywh.tolist(),scores.tolist(),0.,.7),dtype=int).ravel()
                    matched=ev.match(boxes[keep],scores[keep],truth)
                    counts['tp']+=matched
                    counts['fp']+=len(keep)-matched
                    counts['fn']+=len(truth)-matched
                    counts['hit']+=int(matched>0)
                    counts['background_fp']+=int(len(truth)==0 and len(keep)>0)
            record['accuracy']=metrics
        print(json.dumps(record),flush=True)
        all_results.append(record)
        (Path(__file__).parent/'cpu_tuning.json').write_text(json.dumps(all_results,indent=2))
