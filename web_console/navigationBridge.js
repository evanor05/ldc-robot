import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';

// One ROS owner per backend; no shell arguments derived from HTTP input.
export class NavigationBridge {
    // timeoutMs 可注入：生产默认 10s；测试用小值覆盖，避免一条超时用例把测试拖 10 秒。
    constructor(command, { timeoutMs = 10000 } = {}) {
        this.command = command;
        this.timeoutMs = timeoutMs;
        this.pending = new Map();
        this.sequence = 0;
        this.process = null;
    }
    start() {
        if (this.process) return;
        const child = spawn('bash', ['-c', this.command], { stdio: ['pipe', 'pipe', 'inherit'] });
        this.process = child;
        child.stdin.on('error', () => {});
        const lines = createInterface({ input: child.stdout });
        lines.on('line', line => {
            let data;
            try { data = JSON.parse(line); } catch { return; }
            const pending = this.pending.get(data.id);
            if (!pending) return;
            clearTimeout(pending.timer);
            this.pending.delete(data.id);
            if (data.success) pending.resolve(data);
            else pending.reject(new Error(data.error || 'ROS 操作失败'));
        });
        const exited = () => {
            if (this.process !== child) return;
            this.process = null;
            for (const pending of this.pending.values()) {
                clearTimeout(pending.timer);
                pending.reject(new Error('ROS 控制服务已退出，运动已锁定'));
            }
            this.pending.clear();
        };
        child.on('error', exited);
        child.on('exit', exited);
    }
    request(op, data = {}) {
        this.start();
        const id = ++this.sequence;
        return new Promise((resolve, reject) => {
            const timer = setTimeout(() => {
                this.pending.delete(id);
                reject(new Error('ROS 控制服务响应超时'));
                // Stop an outstanding goal even if its acceptance reply arrived late.
                if (op === 'goals') this.process?.stdin.write(JSON.stringify({ op: 'estop' }) + '\n');
            }, this.timeoutMs);
            this.pending.set(id, { resolve, reject, timer });
            this.process.stdin.write(JSON.stringify({ ...data, op, id }) + '\n');
        });
    }
    close() {
        this.process?.stdin.end(JSON.stringify({ op: 'shutdown' }) + '\n');
    }
}
