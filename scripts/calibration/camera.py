"""Offline checkerboard calibration, with held-out views and candidate-only output."""
import hashlib
import math
from pathlib import Path
import cv2
import numpy as np
import yaml


def board_points(cols, rows, square_mm):
    if not 3<=cols<=20 or not 3<=rows<=20 or not 1<=square_mm<=100:
        raise ValueError('棋盘内角点行列应为 3–20，格长单位 mm（1–100）')
    points = np.zeros((cols*rows,3),np.float32)
    points[:,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2)*(square_mm/1000)
    return points


def make_board(path, cols=9, rows=6, square_mm=20):
    board_points(cols,rows,square_mm)
    width,height = (cols+1)*square_mm,(rows+1)*square_mm
    if width>260 or height>170:
        raise ValueError('图案超出 A4 横向可打印范围，请减小格长')
    x0,y0 = (297-width)/2,(210-height)/2-5
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="297mm" height="210mm" viewBox="0 0 297 210">',
           '<rect width="297" height="210" fill="white"/>']
    for y in range(rows+1):
        for x in range(cols+1):
            if (x+y)%2==0:
                parts.append(f'<rect x="{x0+x*square_mm}" y="{y0+y*square_mm}" width="{square_mm}" height="{square_mm}" fill="black"/>')
    parts += ['<path d="M 98.5 197 H 198.5 M 98.5 194 V 200 M 198.5 194 V 200" stroke="black" fill="none" stroke-width=".3"/>',
              f'<text x="148.5" y="190" text-anchor="middle" font-size="3">{cols} x {rows} inner corners; square {square_mm} mm; bottom scale 100 mm; print 100%</text></svg>']
    with Path(path).open('x') as handle:handle.write('\n'.join(parts))


def reprojection_rms(objects, image, rotation, translation, matrix, distortion):
    projected,_ = cv2.projectPoints(objects,rotation,translation,matrix,distortion)
    return float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-image.reshape(-1,2))**2,axis=1))))


def solve_views(object_views, image_views, size):
    if len(image_views)<15:
        raise ValueError('至少需要 15 个有效视角；建议拍摄 25–40 张不同位置和倾角的照片')
    holdout = [i for i in range(len(image_views)) if i%5==4]
    train = [i for i in range(len(image_views)) if i not in holdout]
    rms,matrix,distortion,rotations,translations = cv2.calibrateCamera(
        [object_views[i] for i in train],[image_views[i] for i in train],size,None,None)
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or matrix[0,0]<=0 or matrix[1,1]<=0:
        raise ValueError('相机参数拟合无效')
    held_errors=[]
    for i in holdout:
        ok,rvec,tvec = cv2.solvePnP(object_views[i],image_views[i],matrix,distortion)
        if not ok:raise ValueError('保留视角的 PnP 验证失败')
        held_errors.append(reprojection_rms(object_views[i],image_views[i],rvec,tvec,matrix,distortion))
    centers = [points.reshape(-1,2).mean(axis=0) for points in image_views]
    bins = {(min(2,int(c[0]*3/size[0])),min(2,int(c[1]*3/size[1]))) for c in centers}
    normal_angles=[]
    for rvec in rotations:
        rotation,_ = cv2.Rodrigues(rvec)
        normal_angles.append(math.degrees(math.acos(float(np.clip(abs(rotation[2,2]),-1,1)))))
    tilt_span=max(normal_angles)-min(normal_angles)
    quality = bool(rms<=1 and max(held_errors)<=1.5 and len(bins)>=6 and tilt_span>=15)
    report=dict(status='candidate_only',training_rms_px=float(rms),holdout_rms_px=held_errors,
                training_views=len(train),holdout_views=len(holdout),center_coverage_bins=len(bins),
                tilt_span_deg=tilt_span,quality_checks_pass=quality,
                warnings=['重投影误差不是实际测距精度；打印格长、外参和独立距离/角度仍须实测。'])
    return matrix,distortion,report


def calibrate_folder(folder, output, cols=9, rows=6, square_mm=20):
    objects=board_points(cols,rows,square_mm)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    object_views,image_views,records=[],[],[];size=None;seen=set()
    paths=sorted(p for p in Path(folder).iterdir() if p.suffix.lower() in ('.jpg','.jpeg','.png'))
    for path in paths:
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in seen:
            records.append(dict(file=path.name,status='duplicate'));continue
        seen.add(digest)
        image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        if image is None:
            records.append(dict(file=path.name,status='unreadable'));continue
        current=(image.shape[1],image.shape[0])
        if size is not None and size!=current:
            raise ValueError('照片分辨率混杂；请保持相机分辨率/裁剪模式一致，分开标定')
        size=current
        ok,corners=cv2.findChessboardCornersSB(image,(cols,rows))
        records.append(dict(file=path.name,status='found' if ok else 'not_found',sha256=digest))
        if ok:object_views.append(objects.copy());image_views.append(corners.astype(np.float32))
    import json
    (output/'images.json').write_text(json.dumps(records,indent=2))
    matrix,distortion,report=solve_views(object_views,image_views,size)
    projection=np.column_stack((matrix,np.zeros(3)))
    data=dict(image_width=size[0],image_height=size[1],camera_name='ldc_camera_candidate',distortion_model='plumb_bob',
              camera_matrix=dict(rows=3,cols=3,data=matrix.reshape(-1).tolist()),
              distortion_coefficients=dict(rows=1,cols=distortion.size,data=distortion.reshape(-1).tolist()),
              rectification_matrix=dict(rows=3,cols=3,data=np.eye(3).reshape(-1).tolist()),
              projection_matrix=dict(rows=3,cols=4,data=projection.reshape(-1).tolist()))
    (output/'camera.candidate.yaml').write_text(yaml.safe_dump(data,sort_keys=False))
    report.update(square_mm=square_mm,inner_corners=[cols,rows],image_size=list(size))
    (output/'report.json').write_text(json.dumps(report,indent=2))
    return report


def camera_ray(path,u,v,width,height):
    data=yaml.safe_load(Path(path).read_text())
    if (data['image_width'],data['image_height'])!=(width,height):raise ValueError('标定尺寸与输入像素尺寸不一致')
    if data['distortion_model']!='plumb_bob':raise ValueError('当前射线工具仅支持 plumb_bob 模型')
    matrix=np.array(data['camera_matrix']['data'],float).reshape(3,3)
    distortion=np.array(data['distortion_coefficients']['data'],float)
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or matrix[0,0]<=0 or matrix[1,1]<=0:
        raise ValueError('无效或未标定的相机参数')
    if not 0<=u<width or not 0<=v<height:raise ValueError('像素超出图像边界')
    x,y=cv2.undistortPoints(np.array([[[u,v]]],float),matrix,distortion).reshape(2)
    ray=np.array([x,y,1.]);ray/=np.linalg.norm(ray)
    return dict(frame='camera_optical_frame',unit_ray=ray.tolist(),bearing_right_deg=math.degrees(math.atan2(x,1)),
                elevation_up_deg=-math.degrees(math.atan2(y,math.sqrt(1+x*x))),distance_m=None,
                note='只有方向，没有距离；转换到车体仍需实测外参')
