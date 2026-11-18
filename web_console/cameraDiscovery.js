// MAVLink-style UDP discovery client for the ESP32-S3 camera module.
//
// Protocol source: firmware/camera_stream/main/udp_discovery.c and
// firmware/camera_stream/scripts/test_discovery.py.
//
// The camera listens on UDP 33333 and only answers after it receives a GCS
// heartbeat. It then emits STATUSTEXT frames with DEVICE_IP and STREAM_PORT.

import dgram from 'dgram';

export const DISCOVERY_PORT = 33333;
const MAVLINK_V2_MAGIC = 0xfd;
const MAVLINK_V1_MAGIC = 0xfe;
const MSG_ID_HEARTBEAT = 0;
const MSG_ID_STATUSTEXT = 253;
const HEARTBEAT_LEN = 9;
const HEARTBEAT_CRC_EXTRA = 50;
const STATUSTEXT_CRC_EXTRA = 83;
const MAV_TYPE_GCS = 6;
const MAV_AUTOPILOT_INVALID = 8;
const MAV_STATE_ACTIVE = 4;

function crcAccumulate(data, crc) {
  data &= 0xff;
  data ^= crc & 0xff;
  data ^= (data << 4) & 0xff;
  return (((data << 8) & 0xffff) | (crc >> 8)) ^ (data >> 4) ^ ((data << 3) & 0xffff);
}

function crcCalculate(buffer, crcExtra) {
  let crc = 0xffff;
  for (const byte of buffer) {
    crc = crcAccumulate(byte, crc);
  }
  return crcAccumulate(crcExtra, crc);
}

function packMavlinkV2(sequence, sysId, compId, msgId, payload, crcExtra) {
  const header = Buffer.from([
    payload.length,
    0,
    0,
    sequence & 0xff,
    sysId & 0xff,
    compId & 0xff,
    msgId & 0xff,
    (msgId >> 8) & 0xff,
    (msgId >> 16) & 0xff,
  ]);
  const crc = crcCalculate(Buffer.concat([header, payload]), crcExtra);
  return Buffer.concat([
    Buffer.from([MAVLINK_V2_MAGIC]),
    header,
    payload,
    Buffer.from([crc & 0xff, (crc >> 8) & 0xff]),
  ]);
}

export function buildGcsHeartbeat(sequence = 0) {
  const payload = Buffer.alloc(HEARTBEAT_LEN);
  payload[4] = MAV_TYPE_GCS;
  payload[5] = MAV_AUTOPILOT_INVALID;
  payload[6] = 0;
  payload[7] = MAV_STATE_ACTIVE;
  payload[8] = 3;
  return packMavlinkV2(
    sequence,
    255,
    0,
    MSG_ID_HEARTBEAT,
    payload,
    HEARTBEAT_CRC_EXTRA,
  );
}

export function parseStatustextFrames(data) {
  const texts = [];
  let index = 0;

  while (index < data.length) {
    const magic = data[index];
    if (magic === MAVLINK_V2_MAGIC) {
      if (index + 12 > data.length) break;
      const payloadLen = data[index + 1];
      const frameLen = 10 + payloadLen + 2;
      if (index + frameLen > data.length) break;
      const msgId = data[index + 7] | (data[index + 8] << 8) | (data[index + 9] << 16);
      if (msgId === MSG_ID_STATUSTEXT && payloadLen > 0) {
        const payload = data.subarray(index + 10, index + 10 + payloadLen);
        const text = payload.subarray(1, 51).toString('utf8').replace(/\0[\s\S]*$/, '').trim();
        if (text) texts.push(text);
      }
      index += frameLen;
    } else if (magic === MAVLINK_V1_MAGIC) {
      if (index + 8 > data.length) break;
      const payloadLen = data[index + 1];
      const frameLen = 6 + payloadLen + 2;
      if (index + frameLen > data.length) break;
      const msgId = data[index + 5];
      if (msgId === MSG_ID_STATUSTEXT && payloadLen > 0) {
        const payload = data.subarray(index + 6, index + 6 + payloadLen);
        const text = payload.subarray(1, 51).toString('utf8').replace(/\0[\s\S]*$/, '').trim();
        if (text) texts.push(text);
      }
      index += frameLen;
    } else {
      index += 1;
    }
  }

  return texts;
}

/**
 * Broadcast a GCS heartbeat and resolve the first complete camera record.
 *
 * The firmware replies to the fixed discovery port (33333), not to the
 * caller's ephemeral source port.  The listener therefore binds `bindPort`
 * (default: `port`).  Binding is exclusive by default so a second discovery
 * client gets a clear EADDRINUSE error instead of silently stealing replies.
 *
 * @returns {Promise<{ip:string,name:string,http_port:number,stream_port:number,stream_url:string}|null>}
 */
export function discoverCamera({
  timeoutMs = 2000,
  port = DISCOVERY_PORT,
  bindPort = port,
  broadcast = '255.255.255.255',
} = {}) {
  return new Promise((resolve, reject) => {
    const socket = dgram.createSocket('udp4');
    const devices = new Map();
    let sequence = 0;
    let heartbeatTimer = null;
    let timeout = null;
    let finished = false;

    const finish = (result, error) => {
      if (finished) return;
      finished = true;
      if (heartbeatTimer) clearInterval(heartbeatTimer);
      if (timeout) clearTimeout(timeout);
      try {
        socket.close();
      } catch {
        // Socket may already be closed.
      }
      if (error) reject(error);
      else resolve(result);
    };

    timeout = setTimeout(() => finish(null), Math.max(100, timeoutMs));

    const sendHeartbeat = () => {
      if (finished) return;
      const packet = buildGcsHeartbeat(sequence);
      sequence = (sequence + 1) & 0xff;
      socket.send(packet, port, broadcast, () => {});
    };

    socket.on('message', (data, rinfo) => {
      const ip = rinfo.address;
      const device = devices.get(ip) || {
        ip,
        name: '',
        http_port: 80,
        stream_port: 81,
        stream_url: '',
        seen_ip: false,
        seen_stream_port: false,
      };

      for (const text of parseStatustextFrames(data)) {
        if (text.startsWith('DEVICE_NAME:')) {
          device.name = text.slice('DEVICE_NAME:'.length);
        } else if (text.startsWith('DEVICE_IP:')) {
          const value = text.slice('DEVICE_IP:'.length).trim();
          if (value) {
            device.ip = value;
            device.seen_ip = true;
          }
        } else if (text.startsWith('HTTP_PORT:')) {
          const value = Number.parseInt(text.slice('HTTP_PORT:'.length).trim(), 10);
          if (Number.isFinite(value)) device.http_port = value;
        } else if (text.startsWith('STREAM_PORT:')) {
          const value = Number.parseInt(text.slice('STREAM_PORT:'.length).trim(), 10);
          if (Number.isFinite(value)) {
            device.stream_port = value;
            device.seen_stream_port = true;
          }
        }
      }

      devices.set(ip, device);
      if (device.seen_ip && device.seen_stream_port) {
        const result = {
          ip: device.ip,
          name: device.name,
          http_port: device.http_port,
          stream_port: device.stream_port,
          stream_url: `http://${device.ip}:${device.stream_port}/`,
        };
        finish(result);
      }
    });

    socket.on('error', (error) => {
      // EADDRINUSE is a configuration/ownership problem, not "no camera".
      finish(null, error);
    });
    socket.bind({ port: bindPort, address: '0.0.0.0', exclusive: true }, () => {
      try {
        socket.setBroadcast(true);
      } catch {
        // Some platforms deny SO_BROADCAST; the send below may still work.
      }
      sendHeartbeat();
      heartbeatTimer = setInterval(sendHeartbeat, 700);
    });
  });
}
