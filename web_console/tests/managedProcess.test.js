import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { spawn } from 'node:child_process';
import { terminateProcess } from '../managedProcess.js';

test('cleanup kills a detached grandchild even after its launch parent exits', async () => {
    const childCode = `process.on('SIGTERM',()=>{}); console.log('ready'); setInterval(()=>{},1000)`;
    const parentCode = `const {spawn}=require('node:child_process');
      const child=spawn(process.execPath,['-e',${JSON.stringify(childCode)}],{detached:true,stdio:['ignore','pipe','ignore']});
      child.stdout.once('data',()=>console.log(child.pid)); setInterval(()=>{},1000);`;
    const proc = spawn(process.execPath, ['-e', parentCode], { stdio: ['ignore', 'pipe', 'inherit'] });
    const exited = new Promise(resolve => proc.once('exit', resolve));
    let descendant;
    try {
        descendant = await new Promise(resolve => proc.stdout.once('data', data => resolve(Number(data.toString().trim()))));
        await terminateProcess(proc);
        await exited;
        let state;
        try { state = fs.readFileSync(`/proc/${descendant}/stat`, 'utf8').split(') ')[1].split(' ')[0]; }
        catch (error) { assert.equal(error.code, 'ENOENT'); }
        assert.ok(!state || state === 'Z');
    } finally {
        try { proc.kill('SIGKILL'); } catch {}
        if (descendant) { try { process.kill(descendant, 'SIGKILL'); } catch {} }
    }
});
