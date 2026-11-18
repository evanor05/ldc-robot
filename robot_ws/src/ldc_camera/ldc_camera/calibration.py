"""CameraInfo only uses actual calibration, never guessed focal lengths."""
import math
import yaml
from sensor_msgs.msg import CameraInfo


class Calibration:
    def __init__(self, path=''):
        self.data = None
        self.error = '未标定：CameraInfo.K[0]=0'
        if path:
            try:
                with open(path, encoding='utf-8') as handle:
                    self.data = yaml.safe_load(handle)
            except (OSError, yaml.YAMLError) as error:
                self.error = f'标定文件不可用：{error}'

    def message(self, header, width, height):
        msg = CameraInfo(header=header, width=width, height=height)
        if not self.data:
            return msg, self.error
        try:
            d = self.data
            if (d['image_width'], d['image_height']) != (width, height):
                raise ValueError('标定尺寸与实际图像不匹配')
            k, r, p = [d[name]['data'] for name in ('camera_matrix', 'rectification_matrix', 'projection_matrix')]
            distortion = d['distortion_coefficients']['data']
            if not all(isinstance(values,list) for values in (k,r,p,distortion)):
                raise ValueError('标定矩阵必须为数值列表')
            model=d.get('distortion_model','plumb_bob')
            if model not in ('plumb_bob','rational_polynomial','equidistant'):
                raise ValueError('不支持的畸变模型')
            if len(k) != 9 or len(r) != 9 or len(p) != 12 or not all(math.isfinite(v) for v in k+r+p+distortion) or k[0] <= 0:
                raise ValueError('标定矩阵无效')
            msg.k, msg.r, msg.p, msg.d = k, r, p, distortion
            msg.distortion_model = model
            return msg, '已加载匹配尺寸的标定'
        except (KeyError, TypeError, ValueError) as error:
            return CameraInfo(header=header,width=width,height=height), f'未标定：{error}'
