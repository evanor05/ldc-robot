"""Daily launcher must not start unverified automatic motion; manual still works."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class MappingLaunchTests(unittest.TestCase):
    def test_manual_preserves_explicit_simulation_and_defaults_to_real(self):
        scripts = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'scripts/runtime').mkdir(parents=True)
            (root / 'bin').mkdir()
            shutil.copy2(scripts / 'start_mapping_task.sh', root / 'scripts/start.sh')
            shutil.copy2(scripts / 'runtime/environment.bash', root / 'scripts/runtime/environment.bash')
            (root / 'scripts/env.bash').write_text(': # No actual ROS setup in this fixture\n')
            ros = root / 'bin/ros2'
            ros.write_text('#!/usr/bin/python3\nimport json,os,sys\n'
                           'keys=("ROS_DOMAIN_ID","ROBOT_SIMULATION","ROS_AUTOMATIC_DISCOVERY_RANGE",'
                           '"ROS_LOCALHOST_ONLY","FASTDDS_BUILTIN_TRANSPORTS")\n'
                           'open(os.environ["FAKE_ROS_LOG"],"w").write(json.dumps('
                           '{"args":sys.argv[1:],"env":{k:os.environ.get(k) for k in keys}}))\n')
            ros.chmod(0o755)
            log = root / 'ros.json'
            base = dict(os.environ, PATH=str(root / 'bin') + ':/usr/bin:/bin', FAKE_ROS_LOG=str(log))
            base.pop('ROBOT_SIMULATION', None)
            base.pop('ROBOT_REAL_DOMAIN_ID', None)
            for flag, real_domain in [(None, '0'), ('0', '0'), ('1', '0'), ('0', '43')]:
                with self.subTest(simulation=flag, real_domain=real_domain):
                    env = dict(base)
                    env['ROBOT_REAL_DOMAIN_ID'] = real_domain
                    if flag is not None:
                        env['ROBOT_SIMULATION'] = flag
                    result = subprocess.run(['bash', str(root / 'scripts/start.sh')],
                                            env=env, capture_output=True, text=True, timeout=5)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    observed = json.loads(log.read_text())
                    self.assertEqual(observed['args'], ['launch', 'slam_gmapping', 'slam_gmapping.launch.py'])
                    self.assertEqual(observed['env'], {
                        'ROS_DOMAIN_ID': '197' if flag == '1' else real_domain,
                        'ROBOT_SIMULATION': '1' if flag == '1' else '0',
                        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST' if flag == '1' else 'SUBNET',
                        'ROS_LOCALHOST_ONLY': '1' if flag == '1' else None,
                        'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4' if flag == '1' else None,
                    })

    def test_auto_rejected_before_ros_and_manual_kept(self):
        source=Path(__file__).resolve().parents[1]/'start_mapping_task.sh'
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'runtime').mkdir();(root/'bin').mkdir()
            shutil.copy2(source,root/'start.sh')
            (root/'runtime/environment.bash').write_text('ldc_environment() { :; }\n')
            ros=root/'bin/ros2'
            ros.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$FAKE_ROS_LOG"\n')
            ros.chmod(0o755)
            log=root/'ros.log'
            env=dict(os.environ,PATH=str(root/'bin')+':/usr/bin:/bin',FAKE_ROS_LOG=str(log))
            auto=subprocess.run(['bash',str(root/'start.sh'),'--auto'],env=env,capture_output=True,text=True,timeout=5)
            self.assertNotEqual(auto.returncode,0)
            self.assertFalse(log.exists())
            self.assertIn('控制权交接',auto.stderr)
            manual=subprocess.run(['bash',str(root/'start.sh'),'--manual'],env=env,capture_output=True,text=True,timeout=5)
            self.assertEqual(manual.returncode,0)
            self.assertEqual(log.read_text().splitlines(),['launch','slam_gmapping','slam_gmapping.launch.py'])
