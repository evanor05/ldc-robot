#!/usr/bin/env python3
"""Actual local-model comparison: fixed-confidence traffic P/R, not mAP or car accuracy."""
import argparse
import json
from pathlib import Path
import statistics
import sys
import time
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'robot_ws/src/ldc_vision'))
from ldc_vision.worker_client import WorkerClient


def iou(a,b):
    intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union>0 else 0


def count_matches(predictions,groundtruth,class_id):
    expected=[box for cls,box in groundtruth if cls==class_id];used=set();tp=fp=0
    for item in sorted((p for p in predictions if p['class_id']==class_id),key=lambda p:-p['score']):
        matches=[(iou(item['xyxy'],box),index) for index,box in enumerate(expected) if index not in used]
        score,index=max(matches,default=(0,None))
        if score>=.5:tp+=1;used.add(index)
        else:fp+=1
    return tp,fp,len(expected)-len(used)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--models',nargs='+',required=True);p.add_argument('--images',required=True)
    p.add_argument('--output',required=True);p.add_argument('--repeats',type=int,default=3);p.add_argument('--device',default='0',choices=['0','cpu'])
    args=p.parse_args()
    if not 1<=args.repeats<=50:p.error('repeats 范围 1–50')
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    images=[]
    for path in sorted(Path(args.images).glob('*.jpg')):
        label=path.with_suffix('.txt')
        if not label.is_file():raise ValueError('每张验证图必须有人工作成的完整 YOLO 标签，包括无目标图的空文件')
        jpeg=path.read_bytes();frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        if frame is None:raise ValueError('图像不能解码')
        h,w=frame.shape[:2];labels=[]
        for line in label.read_text().splitlines():
            values=list(map(float,line.split()));cls=int(values[0]);x,y,bw,bh=values[1:]
            if not all(np.isfinite(values)) or values[0]!=cls or not 0<=cls<80 or not 0<=x<=1 or not 0<=y<=1 or not 0<bw<=1 or not 0<bh<=1:raise ValueError('YOLO 标签非法')
            labels.append((cls,[(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h]))
        images.append((path.name,jpeg,labels))
    if not images:raise ValueError('没有带标签的 JPG 样本')
    summaries=[]
    for model in args.models:
        worker=WorkerClient('/home/<user>/esp/yolo-bench-env/bin/python',str(Path(model).resolve()),args.device,640,.25)
        durations=[];counts={9:[0,0,0],11:[0,0,0]};predictions=[]
        try:
            ready=worker.start()
            for repetition in range(args.repeats):
                for index,(name,jpeg,labels) in enumerate(images):
                    start=time.perf_counter();result=worker.infer(dict(id=f'{repetition}:{index}',header=dict(sec=1,nanosec=index,frame_id='camera_optical_frame')),jpeg)
                    durations.append((time.perf_counter()-start)*1000)
                    if repetition==0:
                        predictions.append(dict(image=name,detections=result['detections']))
                        for cls in counts:
                            counts[cls]=[a+b for a,b in zip(counts[cls],count_matches(result['detections'],labels,cls))]
            metrics={}
            for cls,(tp,fp,fn) in counts.items():
                metrics['traffic light' if cls==9 else 'stop sign']=dict(tp=tp,fp=fp,fn=fn,precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn) if tp+fn else None)
            summaries.append(dict(model=Path(model).name,sha256=ready['model_sha256'],images=len(images),imgsz=640,confidence=.25,iou=.5,
                                  median_ipc_latency_ms=statistics.median(durations),p95_ipc_latency_ms=float(np.percentile(durations,95)),metrics=metrics))
            (output/(Path(model).stem+'.predictions.json')).write_text(json.dumps(predictions,indent=2))
        finally:worker.close()
    report=dict(device=args.device,results=summaries,note='同一小样本固定阈值的真实推理比较；不等于 mAP，不代表实车精度，不含相机/ROS延迟。')
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));print(json.dumps(report,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
