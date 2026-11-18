import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { spawn } from 'node:child_process';
import { terminateProcess } from '../managedProcess.js';

test('cleanup includes a YOLO-like child created by a non-main Python thread', async () => {
    const source = `import subprocess,threading,time\ndef run():\n p=subprocess.Popen(['sleep','3600'],start_new_session=True)\n print(p.pid,flush=True)\n time.sleep(3600)\nthreading.Thread(target=run).start()\ntime.sleep(3600)\n`;
    const parent = spawn('/usr/bin/python3', ['-c', source], { stdio: ['ignore', 'pipe', 'inherit'] });
    let child;
    try {
        child = await new Promise(resolve => parent.stdout.once('data', output => resolve(Number(output.toString().trim()))));
        await terminateProcess(parent);
        let state;
        try { state = fs.readFileSync(`/proc/${child}/stat`, 'utf8').split(') ')[1].split(' ')[0]; }
        catch (e) { assert.equal(e.code, 'ENOENT'); }
        assert.ok(!state || state === 'Z', 'thread-owned worker survived cleanup');
    } finally {
        try { parent.kill('SIGKILL'); } catch {}
        if (child) { try { process.kill(child, 'SIGKILL'); } catch {} }
    }
});
