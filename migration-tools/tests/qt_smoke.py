"""Offscreen startup and SIGINT cleanup of both actual Qt executables."""
import os
os.environ['ROS_DOMAIN_ID']='197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
os.environ['QT_QPA_PLATFORM']='offscreen'
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from ament_index_python.packages import get_package_prefix
logs=Path(tempfile.mkdtemp(prefix='ros-car-qt-'))
for name in ('ldc_qt_gui','ldc_servo_gui'):
    with open(logs/f'{name}.log','w') as log:
        process=subprocess.Popen([str(Path(get_package_prefix(name))/'lib'/name/name)],stdout=log,stderr=subprocess.STDOUT)
        try:
            time.sleep(2)
            assert process.poll() is None,f'{name} exited at startup; logs={logs}'
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5)==0,f'{name} failed shutdown; logs={logs}'
        finally:
            if process.poll() is None:
                process.kill();process.wait()
print(json.dumps({'result':'PASS','checks':['both Qt applications start offscreen','both exit cleanly on SIGINT'],
    'limitation':'no screen interaction, GPU or vehicle control verification','logs':str(logs)}))
