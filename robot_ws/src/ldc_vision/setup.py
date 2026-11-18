from glob import glob
from setuptools import setup
setup(name='ldc_vision', version='0.1.0', packages=['ldc_vision'],
      data_files=[('share/ament_index/resource_index/packages',['resource/ldc_vision']),
                  ('share/ldc_vision',['package.xml']),
                  ('share/ldc_vision/launch',glob('launch/*.launch.py')),
                  ('share/ldc_vision/config',glob('config/*.yaml'))],
      entry_points={'console_scripts':['detector = ldc_vision.node:main']})
