// Exercise the app's actual WebSocket/CDR code against the installed bridge.
import assert from 'node:assert/strict'
import { spawn } from 'node:child_process'
import { mkdtempSync, openSync, closeSync } from 'node:fs'
import { tmpdir } from 'node:os'
import { join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { setTimeout as delay } from 'node:timers/promises'
import { useFoxglove } from '../src/composables/useFoxglove.js'

process.env.ROS_DOMAIN_ID='197'
process.env.ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'
globalThis.window=new EventTarget()
const logs=mkdtempSync(join(tmpdir(),'ros-car-web-'))
const fd=openSync(join(logs,'bridge.log'),'w')
const bridge=spawn('ros2',['run','foxglove_bridge','foxglove_bridge','--ros-args','-p','port:=18765','-p','address:=127.0.0.1'],{stdio:['ignore',fd,fd]})
const peer=spawn('/usr/bin/python3',[fileURLToPath(new URL('../../migration-tools/tests/foxglove_peer.py',import.meta.url))],{stdio:['ignore','pipe','pipe']})
let peerOutput=''
peer.stdout.on('data',data=>{peerOutput+=data})
peer.stderr.on('data',data=>{peerOutput+=data})
const fox=useFoxglove()
let battery
let subscription
async function until(predicate,seconds=15) {
  const deadline=Date.now()+seconds*1000
  while (!predicate() && Date.now()<deadline) await delay(100)
  assert.ok(predicate(), `Protocol test timeout; logs=${logs}; peer=${peerOutput}`)
}
try {
  await delay(1000)
  fox.connect('ws://127.0.0.1:18765')
  await until(()=>fox.connectionState.value==='connected' && fox.findChannelByTopic('/migration_check/battery_state'))
  subscription=fox.subscribe('/migration_check/battery_state',data=>{battery=data})
  await until(()=>battery!==undefined)
  assert.ok(Math.abs(battery.voltage-7.8)<1e-5)
  assert.equal(battery.percentage,0.5)
  const channel=fox.clientAdvertise('/migration_check/web_twist','geometry_msgs/Twist','json')
  const deadline=Date.now()+12000
  while (!peerOutput.includes('TWIST_OK') && Date.now()<deadline) {
    fox.sendTwist(channel,0.123,0.321)
    await delay(100)
  }
  assert.ok(peerOutput.includes('TWIST_OK'),`Web JSON Twist did not reach ROS; logs=${logs}`)
  console.log(JSON.stringify({result:'PASS',checks:['real bridge WebSocket handshake','real ROS BatteryState CDR to frontend parser','frontend JSON Twist to real ROS subscriber'],logs}))
} finally {
  if(subscription)fox.unsubscribe(subscription)
  fox.disconnect()
  bridge.kill('SIGINT');peer.kill('SIGINT')
  await Promise.all([bridge,peer].map(p=>p.exitCode!==null ? Promise.resolve():new Promise(resolve=>p.once('exit',resolve))))
  closeSync(fd)
}
