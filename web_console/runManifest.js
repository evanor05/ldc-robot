import { execFile } from 'node:child_process';

/**
 * 建图运行的证据采集（server.js ↔ scripts/run_manifest.py）。
 *
 * 现场问题：17:37–18:00 出现 8 段建图会话、7 段无产物，但本机**没有任何建图会话的日志**
 * （log/ 只有构建日志，logs/runs 全是导航会话，frontend-events 里 0 条 mapping 事件）。
 * 根因无法事后对账，只能补采集：让下一次建图从启动到收尾自己留下证据。
 *
 * 设计约束：
 * - 全程 best-effort：run_manifest 写不出来绝不影响建图/停止本身；
 * - 复用 python 端已有的 manifest/summary 规范，不在这里另造一套格式；
 * - 可注入 exec，便于离线断言 argv（不真的起 python）。
 */
export function createRunManifest({
    python = 'python3',
    scriptPath,
    cwd,
    runsDir = '',
    timeoutMs = 8000,
    exec = execFile
} = {}) {
    let currentRunId = null;
    let everStarted = false;

    const environment = () => (runsDir
        ? { ...process.env, ROBOT_RUNS_DIR: runsDir }
        : { ...process.env });

    function call(args) {
        return new Promise(resolve => {
            try {
                exec(python, [scriptPath, ...args],
                    { cwd, env: environment(), timeout: timeoutMs },
                    (error, stdout) => resolve(error ? null : String(stdout || '')));
            } catch {
                resolve(null);
            }
        });
    }

    return {
        get runId() { return currentRunId },
        get everStarted() { return everStarted },

        /** 记录一次建图启动的生效参数。返回 runId（失败为 null）。 */
        async start(kind, params = {}) {
            const args = ['start', '--kind', kind];
            for (const [key, value] of Object.entries(params)) {
                if (value === undefined || value === null || value === '') continue;
                args.push('--param', `${key}=${value}`);
            }
            const output = await call(args);
            const match = /runId=(\S+)/.exec(output || '');
            currentRunId = match ? match[1] : null;
            if (currentRunId) everStarted = true;
            return currentRunId;
        },

        /**
         * 收尾：status 取 success | failed | aborted | timeout。
         * 返回 python 的输出（失败为 null），调用方不需要关心。
         */
        async finish(status, reason = '') {
            const runId = currentRunId;
            currentRunId = null;
            if (!runId) return null;
            const args = ['finish', runId, '--status', status];
            if (reason) args.push('--reason', String(reason).slice(0, 400));
            return call(args);
        }
    };
}
