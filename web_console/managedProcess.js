import fs from 'node:fs';
import { setTimeout as delay } from 'node:timers/promises';

function identity(pid) {
    try {
        const stat = fs.readFileSync(`/proc/${pid}/stat`, 'utf8').split(') ').at(-1).split(' ');
        return { pid, start: stat[19], zombie: stat[0] === 'Z' };
    } catch { return null; }
}
function tree(pid, result = new Map()) {
    const entry = identity(pid);
    if (!entry || result.has(pid)) return result;
    result.set(pid, entry);
    try {
        // Python starts the YOLO worker from its inference thread. Linux lists
        // that child under the creating thread, not necessarily the main TID.
        for (const tid of fs.readdirSync(`/proc/${pid}/task`)) {
            let children;
            try { children = fs.readFileSync(`/proc/${pid}/task/${tid}/children`, 'utf8'); }
            catch { continue; }
            for (const child of children.trim().split(/\s+/).filter(Boolean)) tree(Number(child), result);
        }
    } catch {}
    return result;
}
function alive(entry) {
    const current = identity(entry.pid);
    return current && !current.zombie && current.start === entry.start;
}
function signal(entry, name) {
    if (!alive(entry)) return;
    try { process.kill(entry.pid, name); }
    catch (error) { if (error.code !== 'ESRCH') throw error; }
}

// launch_ros children can own separate process groups. Track identities before
// their launch parent exits, and wait for every owned descendant, not just ros2.
export async function terminateProcess(proc) {
    if (!proc?.pid) return;
    const owned = tree(proc.pid);
    for (const entry of owned.values()) signal(entry, 'SIGTERM');
    const deadline = Date.now() + 4000;
    while ([...owned.values()].some(alive) && Date.now() < deadline) await delay(50);
    for (const entry of owned.values()) signal(entry, 'SIGKILL');
    const killDeadline = Date.now() + 2000;
    while ([...owned.values()].some(alive) && Date.now() < killDeadline) await delay(50);
    if ([...owned.values()].some(alive)) throw new Error('部分 ROS 子进程未能退出，请查看日志');
}
