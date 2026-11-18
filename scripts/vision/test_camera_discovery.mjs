#!/usr/bin/env node
// Offline regression for the firmware-compatible discovery listener port.
// The fake firmware listens on a dynamic port and replies to the fixed client
// port passed via bindPort (dictated by the firmware's udp_discovery.c).
import assert from 'node:assert/strict';
import dgram from 'node:dgram';
import { discoverCamera } from '../../web_console/cameraDiscovery.js';

function frame(sequence, text) {
  const payload = Buffer.alloc(54);
  payload[0] = 6;
  payload.write(text, 1, 'ascii');
  const header = Buffer.from([payload.length, 0, 0, sequence & 0xff, 1, 1, 253, 0, 0]);
  // The host parser intentionally does not trust CRC; the real firmware sends
  // valid MAVLink v2 CRC bytes.  This mock only exercises framing/port rules.
  return Buffer.concat([Buffer.from([0xfd]), header, payload, Buffer.alloc(2)]);
}

const server = dgram.createSocket('udp4');
await new Promise((resolve, reject) => {
  server.once('error', reject);
  server.bind(0, '127.0.0.1', resolve);
});
const firmwarePort = server.address().port;
const probe = dgram.createSocket('udp4');
await new Promise(resolve => probe.bind(0, '127.0.0.1', resolve));
const clientPort = probe.address().port;
probe.close();
server.on('message', () => {
  for (const [i, text] of [
    'DEVICE_NAME:mock_camera',
    'DEVICE_IP:127.0.0.1',
    'STREAM_PORT:18324',
  ].entries()) {
    server.send(frame(i, text), clientPort, '127.0.0.1');
  }
});
const result = await discoverCamera({
  timeoutMs: 1000,
  port: firmwarePort,
  bindPort: clientPort,
  broadcast: '127.0.0.1',
});
assert.equal(result.stream_url, 'http://127.0.0.1:18324/');
server.close();

const blocker = dgram.createSocket('udp4');
await new Promise(resolve => blocker.bind(0, '127.0.0.1', resolve));
const busyPort = blocker.address().port;
let rejected = false;
try {
  await discoverCamera({ timeoutMs: 300, port: 33333, bindPort: busyPort });
} catch (error) {
  rejected = error.code === 'EADDRINUSE';
}
blocker.close();
assert.equal(rejected, true, 'occupied discovery port must reject clearly');
console.log('camera_discovery_fixed_port_mock_ok');
