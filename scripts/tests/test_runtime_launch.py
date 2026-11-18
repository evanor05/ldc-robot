import argparse
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('runtime_launch',ROOT/'scripts/runtime/launch.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
sys.path.insert(0,str(ROOT/'robot_ws/src/ldc_camera'))
from ldc_camera.lease import CameraLease


def options(**overrides):
    args=dict(sim=True,api_port=None,ws_port=None,web_port=5173,camera_url='',no_vision=False,image='/home/<user>/ros-car-migration-audit/yolo-integration/bus.jpg',transport='micro-ros',no_chassis=False,qt=False,rviz=False,rviz_view='diagnostics',device='0')
    return argparse.Namespace(**(args|overrides))


class RuntimeLaunchTests(unittest.TestCase):
    def test_simulation_never_launches_real_transport_or_arms_motion(self):
        plan=module.make_plan(options(qt=True));names=[c['name'] for c in plan['commands']]
        self.assertIn('robot-simulation',names);self.assertNotIn('agent',names);self.assertNotIn('chassis',names)
        self.assertEqual(plan['web_port'],5173);self.assertEqual(plan['api_port'],18310)
        self.assertFalse(any('--enable' in c['command'] or 'start_navigation.sh' in ' '.join(c['command']) for c in plan['commands']))
    def test_invalid_urls_and_port_collisions_are_rejected_before_spawn(self):
        for args in [options(camera_url='http://192.168.1.2/'),options(api_port=5173),options(web_port=80),options(camera_url='http://user:pass@127.0.0.1/')]:
            with self.assertRaises(ValueError):module.make_plan(args)
    def test_real_micro_ros_and_mavlink_are_exclusive(self):
        for transport,expected in [('micro-ros','agent'),('mavlink','mavlink')]:
            names=[c['name'] for c in module.make_plan(options(sim=False,transport=transport))['commands']]
            self.assertIn(expected,names);self.assertNotIn('mavlink' if expected=='agent' else 'agent',names)
    def test_camera_acquisition_lock_is_single_owner(self):
        with tempfile.TemporaryDirectory() as temp:
            first=CameraLease('/camera/image_raw/compressed',temp)
            try:
                with self.assertRaises(RuntimeError):CameraLease('/camera/image_raw/compressed',temp)
                second=CameraLease('/camera/other',temp);second.close()
            finally:first.close()
            again=CameraLease('/camera/image_raw/compressed',temp);again.close()
    def test_process_identity_matches_current_process(self):self.assertIsNotNone(module.process_identity(os.getpid()))
