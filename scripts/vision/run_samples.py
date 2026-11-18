#!/usr/bin/env python3
"""Run actual installed weights through the production IPC worker; save auditable results."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'robot_ws/src/ldc_vision'))
from ldc_vision.worker_client import WorkerClient
from ldc_vision.results import checked_detections


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--images',nargs='+',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--worker-python',default='/home/<user>/esp/yolo-bench-env/bin/python');parser.add_argument('--device',default='0')
    parser.add_argument('--model',default=str(Path.home()/'.local/share/ldc/models/yolo26n.pt'))
    args=parser.parse_args();output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    client=WorkerClient(args.worker_python,args.model,args.device,640,.25);summary=[]
    try:
        ready=client.start()
        for idx,filename in enumerate(args.images):
            original=Path(filename);jpeg=original.read_bytes();frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            h,w=frame.shape[:2];header=dict(sec=1,nanosec=idx,frame_id='camera_optical_frame')
            result=client.infer(dict(id=str(idx),header=header),jpeg)
            checked=checked_detections(result,str(idx),header,w,h)
            # Coordinate assertion is against native output dimensions of non-square inputs.
            for d in checked:
                x1,y1,x2,y2=d['xyxy'];assert 0<=x1<x2<=w and 0<=y1<y2<=h
                cv2.rectangle(frame,(int(x1),int(y1)),(int(x2),int(y2)),(0,230,80),2)
                cv2.putText(frame,f"{d['name']} {d['score']:.2f}",(int(x1),max(15,int(y1)-4)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,230,80),1)
            stem=original.stem;(output/(stem+'.original.jpg')).write_bytes(jpeg)
            (output/(stem+'.json')).write_text(json.dumps(result,indent=2));cv2.imwrite(str(output/(stem+'.annotated.jpg')),frame)
            summary.append(dict(image=original.name,width=w,height=h,classes=sorted({d['name'] for d in checked})))
        (output/'summary.json').write_text(json.dumps(dict(model_sha256=hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),device=args.device,results=summary),indent=2))
        print(json.dumps(summary,indent=2))
        classes={cls for item in summary for cls in item['classes']}
        assert 'traffic light' in classes and 'stop sign' in classes and 'person' in classes,'代表类别未被真实模型检出'
    finally:client.close()
if __name__=='__main__':main()
