from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ldc_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LDC team',
    maintainer_email='todo@todo.todo',
    description='Frontier exploration and velocity safety gate for LDC autonomous mapping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = ldc_exploration.frontier_explorer:main',
            'cmd_vel_gate = ldc_exploration.cmd_vel_gate:main',
        ],
    },
)
