#!/usr/bin/env python3
"""LDC 离线标定：审计、模板、只读采集、候选拟合；不改在线参数。"""
import argparse
import csv
import json
import os
from pathlib import Path
import yaml
import cv2
from inventory import read_inventory
from measurements import SCHEMAS,analyze
from camera import make_board,calibrate_folder,camera_ray


def dump(path,data):
    with Path(path).open('x') as handle:json.dump(data,handle,indent=2,ensure_ascii=False,allow_nan=False)


def initialize(folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    baseline=read_inventory();dump(folder/'baseline.json',baseline)
    geometry=dict(status='unmeasured',units='m',reference='base_link: x forward, y left, z up',
                  body_extents_from_base_link_m=dict(front=None,back=None,left=None,right=None,height=None),
                  left_wheel_diameter_m=None,right_wheel_diameter_m=None,mechanical_track_m=None,
                  axle_center_x_m=None,sensor_extrinsics_measured=dict(lidar=False,camera=False,imu=False),
                  sensor_extrinsics_from_base_link={name:dict(x_m=None,y_m=None,z_m=None,roll_deg=None,pitch_deg=None,yaw_deg=None) for name in ('laser_frame','camera_link','imu_link')})
    (folder/'dimensions.yaml').write_text(yaml.safe_dump(geometry,allow_unicode=True,sort_keys=False))
    for kind,fields in SCHEMAS.items():
        with (folder/(kind+'.csv')).open('x',newline='') as handle:csv.writer(handle).writerow(fields)
    make_board(folder/'checkerboard_9x6_20mm_A4.svg')
    (folder/'README.md').write_text('# 标定记录\n\n所有 CSV 初始为空，尺寸为 null，未填入模拟测量。详细操作见仓库 docs/LDC标定与精度验收.md。\n参考数据必须来自卷尺、角度基准或人工标注，不能拿里程计/AMCL 自己当真值。\n')
    return dict(session=str(folder),status='awaiting_measurements',warnings=baseline['warnings'])


def main():
    parser=argparse.ArgumentParser(description=__doc__);commands=parser.add_subparsers(dest='command',required=True)
    p=commands.add_parser('init');p.add_argument('folder')
    commands.add_parser('audit')
    p=commands.add_parser('analyze');p.add_argument('folder');p.add_argument('--output',required=True)
    p=commands.add_parser('record');p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=30);p.add_argument('--camera-every',type=float,default=0);m=p.add_mutually_exclusive_group();m.add_argument('--sim',action='store_true');m.add_argument('--real',action='store_true')
    p=commands.add_parser('summarize');p.add_argument('jsonl');p.add_argument('--output',required=True)
    p=commands.add_parser('camera');p.add_argument('images');p.add_argument('--output',required=True);p.add_argument('--cols',type=int,default=9);p.add_argument('--rows',type=int,default=6);p.add_argument('--square-mm',type=float,default=20)
    p=commands.add_parser('ray');p.add_argument('calibration');
    for name in ('u','v'):p.add_argument('--'+name,type=float,required=True)
    for name in ('width','height'):p.add_argument('--'+name,type=int,required=True)
    args=parser.parse_args()
    try:
        if args.command=='init':result=initialize(args.folder)
        elif args.command=='audit':result=read_inventory()
        elif args.command=='analyze':result=analyze(args.folder);dump(args.output,result)
        elif args.command=='record':
            if not 0<args.seconds<=600 or not 0<=args.camera_every<=60:raise ValueError('记录时长为 (0,600]s，拍照间隔为 [0,60]s，0 表示不拍照')
            if args.sim:
                os.environ.update(ROS_DOMAIN_ID='197',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
            elif args.real:
                os.environ.update(ROS_DOMAIN_ID=os.environ.get('ROBOT_REAL_DOMAIN_ID','0'),ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET')
                for key in ('ROS_LOCALHOST_ONLY','FASTDDS_BUILTIN_TRANSPORTS'):os.environ.pop(key,None)
            from record import record
            result=record(args.output,args.seconds,args.camera_every)
        elif args.command=='summarize':
            from summarize import summarize
            result=summarize(args.jsonl);dump(args.output,result)
        elif args.command=='camera':result=calibrate_folder(args.images,args.output,args.cols,args.rows,args.square_mm)
        else:result=camera_ray(args.calibration,args.u,args.v,args.width,args.height)
        print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    except (ValueError,OSError,KeyError,TypeError,cv2.error) as error:
        parser.exit(1,f'标定未完成：{error}\n原配置未改动；请修正输入并选择新的输出路径。\n')
if __name__=='__main__':main()
