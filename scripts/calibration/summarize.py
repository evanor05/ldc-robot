"""Summarize passive odometry captures; reject clock resets and pose discontinuities."""
import json
import math
from pathlib import Path
import statistics
import numpy as np


def yaw_of(q):
    values=[q[k] for k in ('x','y','z','w')]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):raise ValueError('无效四元数')
    norm=math.sqrt(sum(v*v for v in values))
    if not .8<norm<1.2:raise ValueError('四元数模长无效')
    x,y,z,w=[v/norm for v in values]
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def summarize(path):
    with Path(path).open() as handle:
        return summarize_rows(json.loads(line) for line in handle)


def summarize_rows(rows):
    odom=[];imus=[];walls=[];counts={}
    for row in rows:
        topic=row['topic'];data=row['data'];counts[topic]=counts.get(topic,0)+1
        if topic=='/odom':odom.append(data)
        elif topic=='/imu':imus.append(data)
        elif topic=='/scan':
            increment=data.get('angle_increment')
            if not isinstance(increment,(int,float)) or not math.isfinite(increment) or increment==0:continue
            points=[]
            for i,r in enumerate(data['ranges']):
                angle=data['angle_min']+i*data['angle_increment']
                if r is not None and math.isfinite(r) and data['range_min']<=r<=data['range_max'] and abs(angle)<=math.radians(20):
                    points.append([r*math.cos(angle),r*math.sin(angle)])
            if len(points)>=8:
                points=np.array(points)
                if float(np.linalg.norm(np.ptp(points,axis=0)))<.01:continue
                mean=points.mean(axis=0)
                _,_,vt=np.linalg.svd(points-mean,full_matrices=False);normal=vt[-1]
                if np.dot(normal,mean)<0:normal=-normal
                residual=float(np.sqrt(np.mean(((points-mean)@normal)**2)))
                if residual<=.02:
                    walls.append(dict(distance=float(np.dot(normal,mean)),bearing=math.degrees(math.atan2(normal[1],normal[0])),rms=residual))
    result=dict(counts=counts,odom=None,imu=None,front_wall=None)
    if odom:
        first=odom[0];frames=(first['header']['frame_id'],first['child_frame_id'])
        initial_yaw=yaw_of(first['pose']['pose']['orientation']);last_yaw=initial_yaw;total_yaw=0.;previous=None
        for msg in odom:
            p=msg['pose']['pose']['position'];angle=yaw_of(msg['pose']['pose']['orientation'])
            stamp=msg['header']['stamp'];stamp=stamp['sec']+stamp['nanosec']/1e9
            if stamp<=0 or not all(isinstance(p[k],(int,float)) and math.isfinite(p[k]) for k in ('x','y','z')):raise ValueError('里程计位置或时间无效')
            if frames!=(msg['header']['frame_id'],msg['child_frame_id']):raise ValueError('记录中里程计参考系发生变化')
            if previous:
                dt=stamp-previous[0];delta=(angle-last_yaw+math.pi)%(2*math.pi)-math.pi
                if not 0<dt<=.5:raise ValueError('里程计重复、回拨或断流超过 0.5s，不能可靠累计角度')
                if abs(delta)>1.5*dt+.1 or math.hypot(p['x']-previous[1]['x'],p['y']-previous[1]['y'])>.5*dt+.05:
                    raise ValueError('里程计发生重置/跳变或超过当前固件运动上限，不能用于尺度拟合')
                total_yaw+=delta
            previous=(stamp,p);last_yaw=angle
        start=first['pose']['pose']['position'];end=odom[-1]['pose']['pose']['position']
        dx,dy=end['x']-start['x'],end['y']-start['y']
        result['odom']=dict(samples=len(odom),forward_delta_m=dx*math.cos(initial_yaw)+dy*math.sin(initial_yaw),
                            lateral_delta_m=-dx*math.sin(initial_yaw)+dy*math.cos(initial_yaw),
                            unwrapped_yaw_deg=math.degrees(total_yaw),source='unknown_until_firmware_verified')
    if imus:
        gyro={axis:[m['angular_velocity'][axis] for m in imus if m['angular_velocity_covariance'][0]!=-1] for axis in ('x','y','z')}
        valid=all(len(v)>1 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in v) for v in gyro.values())
        if valid:result['imu']=dict(samples=len(imus),gyro_mean_rad_s={a:statistics.mean(v) for a,v in gyro.items()},
                                   gyro_sample_std_rad_s={a:statistics.stdev(v) for a,v in gyro.items()},
                                   note='只有确认全程静止时，均值才能作为零偏候选；不自动应用')
    if walls:result['front_wall']=dict(samples=len(walls),distance_m=statistics.median(w['distance'] for w in walls),
                                      normal_bearing_deg=statistics.median(w['bearing'] for w in walls),
                                      note='laser_frame 中前方 ±20° 近似平面，不是车头距墙；需确认该区域只有同一面墙')
    return result
