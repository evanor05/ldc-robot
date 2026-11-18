#!/usr/bin/env python3
"""Own the LDC host stack, preserve external sessions, and never arm motion."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request
from urllib.parse import urlsplit

ROOT=Path(__file__).resolve().parents[2]
PR_SET_CHILD_SUBREAPER=36


def process_identity(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(') ',1)[1].split()
        return None if fields[0]=='Z' else fields[19]
    except OSError:return None


def make_plan(args):
    sim=args.sim;mode='--sim' if sim else '--real'
    api=args.api_port or (18310 if sim else 3000);ws=args.ws_port or (8766 if sim else 8765)
    for port in (api,ws,args.web_port):
        if not 1024<=port<=65535:raise ValueError('端口必须在 1024–65535')
    if len({api,ws,args.web_port})!=3:raise ValueError('API/网页/WebSocket 端口不得相同')
    camera_url=args.camera_url or ('http://127.0.0.1:18322/stream' if sim else '')
    if camera_url:
        parsed=urlsplit(camera_url)
        if parsed.scheme not in ('http','https') or not parsed.hostname or parsed.username or parsed.password:raise ValueError('相机需要无凭据 HTTP(S) URL')
        if sim and parsed.hostname not in ('127.0.0.1','localhost','::1'):raise ValueError('仿真仅使用 loopback 图像源')
    commands=[]
    def add(name,command,required=True):commands.append(dict(name=name,command=command,required=required))
    def component(name):return ['bash',str(ROOT/f'scripts/start_{name}.sh'),mode]
    if sim:
        add('robot-simulation',[sys.executable,str(ROOT/'scripts/offline_robot.py')])
        if not args.no_vision and not args.camera_url:
            if not Path(args.image).is_file():raise ValueError('仿真需要现有 JPEG，请传 --image /ABS/PATH/image.jpg')
            add('camera-simulation',[sys.executable,str(ROOT/'scripts/vision/mock_camera.py'),'--image',args.image])
    else:
        if args.transport!='none':add('agent' if args.transport=='micro-ros' else 'mavlink',component('agent' if args.transport=='micro-ros' else 'mavlink'))
        if not args.no_chassis:add('chassis',component('chassis'))
    add('foxglove',component('foxglove'));add('backend',component('backend'));add('frontend',component('frontend'))
    if args.qt:add('qt',['bash',str(ROOT/'scripts/start_qt.sh'),mode],False)
    if args.rviz:add('rviz',['bash',str(ROOT/'scripts/start_rviz.sh'),mode,'--'+args.rviz_view] if args.rviz_view!='navigation' else ['bash',str(ROOT/'scripts/start_rviz.sh'),mode],False)
    return dict(simulation=sim,mode=mode,api_port=api,ws_port=ws,web_port=args.web_port,
                url=f'http://localhost:{args.web_port}/',camera_url=camera_url,commands=commands,
                vision=not args.no_vision,device=args.device,transport='simulation' if sim else args.transport)


def request(port,endpoint,body=None):
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    req=urllib.request.Request(f'http://127.0.0.1:{port}/api/'+endpoint,
                              data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'})
    with opener.open(req,timeout=12) as response:return json.load(response)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    modes=p.add_mutually_exclusive_group();modes.add_argument('--real',action='store_true');modes.add_argument('--sim',action='store_true')
    operations=p.add_mutually_exclusive_group();operations.add_argument('--status',action='store_true');operations.add_argument('--stop',action='store_true')
    p.add_argument('--dry-run',action='store_true');p.add_argument('--qt',action='store_true');p.add_argument('--rviz',action='store_true')
    p.add_argument('--rviz-view',choices=['diagnostics','vision','navigation'],default='diagnostics')
    p.add_argument('--transport',choices=['micro-ros','mavlink','none'],default='micro-ros');p.add_argument('--no-chassis',action='store_true')
    p.add_argument('--no-vision',action='store_true');p.add_argument('--camera-url',default='');p.add_argument('--device',choices=['0','cpu'],default='0')
    p.add_argument('--image',default='/home/<user>/ros-car-migration-audit/yolo-integration/bus.jpg')
    p.add_argument('--web-port',type=int,default=5173);p.add_argument('--api-port',type=int);p.add_argument('--ws-port',type=int)
    args=p.parse_args();mode='sim' if args.sim else 'real'
    state_dir=Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/f'ldc-runtime-{os.getuid()}'
    state_path=state_dir/(mode+'.json')
    if args.status or args.stop:
        try:state=json.loads(state_path.read_text())
        except (OSError,ValueError):state=dict(running=False,mode=mode)
        alive=bool(state.get('running') and process_identity(state.get('pid'))==state.get('identity'))
        state['running']=alive
        if args.stop and alive:
            os.kill(state['pid'],signal.SIGINT)
            deadline=time.monotonic()+25
            while time.monotonic()<deadline and process_identity(state['pid'])==state['identity']:time.sleep(.1)
            if process_identity(state['pid'])==state['identity']:raise RuntimeError('退出尚未完成，请查看本次日志，不强杀其他会话')
            state=json.loads(state_path.read_text())
            state['running']=False
        print(json.dumps(state,ensure_ascii=False,indent=2));return 0
    plan=make_plan(args)
    if args.dry_run:print(json.dumps(plan,ensure_ascii=False,indent=2));return 0
    if (args.qt or args.rviz) and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise ValueError('当前没有桌面显示；先不加 --qt/--rviz，桌面终端再启动对应脚本')
    state_dir.mkdir(mode=0o700,parents=True,exist_ok=True)
    lock=(state_dir/(mode+'.lock')).open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        print(state_path.read_text() if state_path.exists() else '同模式实例正在启动，请稍后用 --status 查询');return 0
    ports=[plan['api_port'],plan['ws_port'],plan['web_port']]
    if plan['vision']:ports.append(18321 if args.sim else 18320)
    if args.sim and plan['vision'] and not args.camera_url:ports.append(18322)
    if len(set(ports))!=len(ports):raise ValueError('自定义端口与视觉/模拟源端口冲突')
    for port in ports:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
    if not args.sim and args.transport=='micro-ros':
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:sock.bind(('0.0.0.0',int(os.environ.get('ROBOT_AGENT_PORT','8888'))))
    run=Path(tempfile.mkdtemp(prefix=f'ldc-{mode}-'))
    env=dict(os.environ,ROBOT_API_PORT=str(plan['api_port']),VITE_ROBOT_API_PORT=str(plan['api_port']),
             VITE_ROBOT_WS_PORT=str(plan['ws_port']),ROBOT_WEB_PORT=str(plan['web_port']),ROBOT_CONSOLE_LOG_DIR=str(run),
             ROS_DOMAIN_ID='197' if args.sim else os.environ.get('ROBOT_REAL_DOMAIN_ID','0'),ROBOT_SIMULATION='1' if args.sim else '0')
    if args.sim:env.update(ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_BUILTIN_TRANSPORTS='UDPv4',ROBOT_MAPS_DIR=str(run/'maps'))
    else:
        env['ROS_AUTOMATIC_DISCOVERY_RANGE']='SUBNET'
        for key in ('ROS_LOCALHOST_ONLY','FASTDDS_BUILTIN_TRANSPORTS'):env.pop(key,None)
    state=dict(running=True,phase='starting',pid=os.getpid(),identity=process_identity(os.getpid()),mode=mode,
               url=plan['url'],logs=str(run),api_port=plan['api_port'],children={})
    def save():
        tmp=state_path.with_suffix('.tmp');tmp.write_text(json.dumps(state,ensure_ascii=False,indent=2));tmp.replace(state_path)
    # Reparent escaped ROS/worker descendants to this supervisor if a backend
    # crashes. The setting applies only to this process tree, never globally.
    import ctypes
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(PR_SET_CHILD_SUBREAPER,1,0,0,0)!=0:raise OSError(ctypes.get_errno(),'cannot become child subreaper')
    children=[];logs=[]
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:
        save()
        subprocess.run([sys.executable,str(ROOT/'scripts/prepare_runtime.py')],env=env,check=True,stdout=subprocess.DEVNULL)
        for entry in plan['commands']:
            log=(run/(entry['name']+'.log')).open('w');logs.append(log)
            child=subprocess.Popen(entry['command'],cwd=ROOT,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
            children.append((entry,child));state['children'][entry['name']]=child.pid;save()
        deadline=time.monotonic()+20
        while True:
            if any(c.poll() is not None for entry,c in children if entry['required']):raise RuntimeError(f'启动进程退出，请查看 {run}')
            try:request(plan['api_port'],'vision/status');break
            except OSError:
                if time.monotonic()>deadline:raise RuntimeError('Node API 启动超时')
                time.sleep(.2)
        if plan['vision']:
            request(plan['api_port'],'vision/start',dict(url=plan['camera_url'],device=args.device,filter='all'))
        state['phase']='running';save()
        print(f'{"仿真" if args.sim else "实车"} LDC 已启动：{plan["url"]}\n日志：{run}\n车辆未自动授权或下发目标；Qt 默认只观察。Ctrl+C 清理本次进程。',flush=True)
        while True:
            for entry,child in children:
                if entry['required'] and child.poll() is not None:raise RuntimeError(f'{entry["name"]} 退出，请查看 {run}')
            time.sleep(.5)
    except KeyboardInterrupt:return 0
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        state['phase']='stopping';save()
        cleanup_errors=[]
        for entry,child in reversed(children):
            result=subprocess.run(['node','--input-type=module','-e',
                "import {terminateProcess} from './web_console/managedProcess.js';await terminateProcess({pid:Number(process.argv[1])});",str(child.pid)],cwd=ROOT)
            if result.returncode:cleanup_errors.append(entry['name'])
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:cleanup_errors.append(entry['name'])
        # A killed launch parent may have orphaned children in other sessions.
        adopted=Path(f'/proc/{os.getpid()}/task/{os.getpid()}/children').read_text().split()
        if adopted:
            result=subprocess.run(['node','--input-type=module','-e',
                "import {terminateProcess} from './web_console/managedProcess.js';await Promise.all(process.argv.slice(1).map(pid=>terminateProcess({pid:Number(pid)})));",*adopted],cwd=ROOT)
            if result.returncode:cleanup_errors.append('adopted-children')
        while True:
            try:
                pid,_=os.waitpid(-1,os.WNOHANG)
                if pid==0:break
            except ChildProcessError:break
        for log in logs:log.close()
        state.update(running=False,phase='stopped',cleanup_errors=cleanup_errors);save();lock.close()
        if cleanup_errors:raise RuntimeError('部分子进程未退出：'+','.join(cleanup_errors))
if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,RuntimeError) as error:raise SystemExit(f'LDC 启动未完成：{error}')
