#!/usr/bin/env python3
"""Own and clean up every isolated simulation process, including on test failure."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '197' or os.environ.get('ROBOT_SIMULATION') != '1':
        parser.error('Run scripts/offline_console.sh')
    run = args.output or Path(tempfile.mkdtemp(prefix='ldc-console-sim-'))
    run.mkdir(parents=True, exist_ok=True)
    maps = run / 'maps'; maps.mkdir(exist_ok=True)
    # 诊断包可以 tail 该目录中的有限长度日志（P2-3）。
    env = dict(os.environ, ROBOT_MAPS_DIR=str(maps.resolve()),
                   ROBOT_CONSOLE_LOG_DIR=str(run.resolve()), VITE_CLIENT_LOG='1')
    processes = []; logs = []
    def start(name, command, cwd=ROOT):
        log = (run / (name + '.log')).open('w'); logs.append(log)
        proc = subprocess.Popen(command, cwd=cwd, env=env, start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
        processes.append(proc); return proc
    def cleanup(*_):
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try: proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL); proc.wait()
        for log in logs: log.close()
    def interrupted(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        # Fail before spawning if another console owns any of our ports.
        import socket
        for port in (18310, 8766, 5174):
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(('127.0.0.1', port))
        robot = start('robot', [sys.executable, str(ROOT / 'scripts/offline_robot.py')])
        env['ROBOT_SIM_PID'] = str(robot.pid)
        backend = start('backend', ['node', 'server.js'], ROOT / 'web_console')
        env['ROBOT_BACKEND_PID'] = str(backend.pid)
        start('foxglove', ['ros2', 'run', 'foxglove_bridge', 'foxglove_bridge', '--ros-args', '-p', 'port:=8766', '-p', 'capabilities:=[connectionGraph]', '-p',
            "topic_whitelist:=['^/(scan|odom|map|global_costmap/costmap|plan|tf|tf_static|battery_state|imu)$']"])
        start('frontend', ['npm', 'run', 'dev', '--', '--host', '127.0.0.1', '--port', '5174', '--strictPort'], ROOT / 'web_console')
        print(f'仿真控制台 http://localhost:5174/\n日志及地图 {run}', flush=True)
        if args.test:
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/offline_acceptance.py'), '--output', str(run)], env=env)
            if result.returncode != 0: return result.returncode
            # A clean backend restart must restore saved map metadata, never motion.
            backend.send_signal(signal.SIGTERM); backend.wait(timeout=8)
            start('backend-restarted', ['node', 'server.js'], ROOT / 'web_console')
            import json
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            def api(endpoint):
                with opener.open('http://localhost:18310/api/' + endpoint, timeout=15) as response:
                    return json.load(response)
            deadline = time.monotonic() + 20
            while True:
                try:
                    restored = api('mapping_status')
                    state = api('navigation_status')
                    break
                except (OSError, ValueError):
                    if time.monotonic() > deadline: raise
                    time.sleep(.2)
            report_path = run / 'acceptance.json'
            report = json.loads(report_path.read_text())
            assert restored['lastSavedMap']['path'] == report['map']['path']
            assert not restored['isRunning'] and not state['isRunning']
            assert state['mode'] == 'locked' and state['goalId'] is None
            report['checks'].append('backend restart restores saved map and leaves motion locked')
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print('PASS backend restart restores map metadata and leaves motion locked', flush=True)
            return 0
        while all(proc.poll() is None for proc in processes): time.sleep(.5)
        raise RuntimeError('A simulation service exited; inspect logs')
    except KeyboardInterrupt:
        return 0
    finally:
        cleanup()

if __name__ == '__main__': raise SystemExit(main())
