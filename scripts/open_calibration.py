"""Open the real calibration wizard without starting chassis or authorizing motion."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
PAGE = 'http://localhost:5173/#calibration'


def console_ready(port):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(f'http://127.0.0.1:{port}/api/calibration/sessions', timeout=2) as response:
            result = json.loads(response.read(2*1024*1024))
        if not isinstance(result, dict) or not isinstance(result.get('sessions'), list) or not isinstance(result.get('simulation'), bool):
            return False
        if result['simulation']:
            raise RuntimeError('当前 5173 控制台属于仿真，请从独立仿真入口停止后再打开实车标定。')
        with opener.open('http://127.0.0.1:5173/', timeout=2) as response:
            return 'LDC' in response.read(65536).decode('utf-8')
    except (OSError, ValueError):
        return False


def main():
    parser = argparse.ArgumentParser(description='打开 localhost:5173 标定向导；未启动网页时自动启动网页栈并保留日志。')
    parser.add_argument('--check', action='store_true', help='只检查实车网页是否可用，不启动或打开窗口')
    args = parser.parse_args()
    port = int(os.environ.get('ROBOT_API_PORT', '3000'))
    child = None
    def interrupted(*_): raise KeyboardInterrupt()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    try:
        ready = console_ready(port)
        if args.check:
            print('实车标定网页已就绪。' if ready else '实车网页尚未就绪；正常双击入口会尝试启动。')
            return 0 if ready else 1
        if not ready:
            for endpoint in (port, 5173, 8765):
                with socket.socket() as sock:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    try: sock.bind(('0.0.0.0', endpoint))
                    except OSError:
                        raise RuntimeError(f'端口 {endpoint} 已被占用，但标定网页未就绪。请检查已有日志；本入口不会终止其他服务。')
            print('启动实车网页栈（不启动底盘、不启用运动）……', flush=True)
            child = subprocess.Popen(['bash', str(ROOT/'scripts/start_web.sh'), '--real'], cwd=ROOT, start_new_session=True)
            deadline = time.monotonic()+60
            while not console_ready(port):
                if child.poll() is not None:
                    raise RuntimeError(f'网页启动退出，代码 {child.returncode}，请查看上方日志。')
                if time.monotonic() >= deadline:
                    raise RuntimeError('网页启动超过 60 秒，请查看上方日志。')
                time.sleep(.3)
        print('标定向导：'+PAGE, flush=True)
        opened = subprocess.run(['xdg-open', PAGE], check=False, timeout=15)
        if opened.returncode:
            print('浏览器未自动打开，请访问上方地址。', flush=True)
        if child:
            print('此窗口管理新启动的网页栈；按 Ctrl+C 停止。采集日志见网页与桌面日志目录。', flush=True)
            return child.wait()
        print('已复用正在运行的实车网页；关闭此日志窗口不影响它。', flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print('标定向导未打开：'+str(error), flush=True)
        return 1
    finally:
        if child is not None:
            subprocess.run(['node', '--input-type=module', '-e',
                "import {terminateProcess} from './web_console/managedProcess.js'; await terminateProcess({pid:Number(process.argv[1])});",
                str(child.pid)], cwd=ROOT, check=True, timeout=20)


if __name__ == '__main__':
    raise SystemExit(main())
