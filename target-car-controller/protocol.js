const SOF = Buffer.from([0xaa, 0x55]);
const VERSION = 0x01;
const MAX_PAYLOAD = 40;

const CMD = Object.freeze({
  PING: 0x01,
  HEARTBEAT: 0x02,
  SET_MODE: 0x10,
  SET_REMOTE: 0x11,
  ESTOP: 0x12,
  CLEAR_ESTOP: 0x13,
  SET_AUTO_SPEED: 0x14,
  SET_REMOTE_MAPPING: 0x15,
  ACK: 0x80,
  PONG: 0x81,
  STATUS: 0x82
});

const MODE = Object.freeze({
  STOP: 0,
  REMOTE: 1,
  AUTO_ROTATE: 2,
  AUTO_MOVE: 3,
  AUTO_BOTH: 4
});

function crc16Modbus(data) {
  let crc = 0xffff;
  for (const byte of data) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = (crc & 1) ? ((crc >>> 1) ^ 0xa001) : (crc >>> 1);
    }
  }
  return crc & 0xffff;
}

function encodeFrame(command, sequence, payload = Buffer.alloc(0)) {
  const body = Buffer.from(payload);
  if (body.length > MAX_PAYLOAD) throw new Error('payload too long');

  const frame = Buffer.alloc(8 + body.length);
  SOF.copy(frame, 0);
  frame[2] = VERSION;
  frame[3] = command;
  frame[4] = sequence;
  frame[5] = body.length;
  body.copy(frame, 6);

  const crc = crc16Modbus(frame.subarray(2, 6 + body.length));
  frame.writeUInt16LE(crc, 6 + body.length);
  return frame;
}

class FrameDecoder {
  constructor() {
    this.buffer = Buffer.alloc(0);
    this.crcErrors = 0;
  }

  push(chunk) {
    this.buffer = Buffer.concat([this.buffer, Buffer.from(chunk)]);
    const frames = [];

    while (this.buffer.length >= 8) {
      const sofIndex = this.buffer.indexOf(SOF);
      if (sofIndex < 0) {
        this.buffer = this.buffer.at(-1) === SOF[0]
          ? this.buffer.subarray(this.buffer.length - 1)
          : Buffer.alloc(0);
        break;
      }
      if (sofIndex > 0) this.buffer = this.buffer.subarray(sofIndex);
      if (this.buffer.length < 8) break;

      const payloadLength = this.buffer[5];
      if (payloadLength > MAX_PAYLOAD) {
        this.buffer = this.buffer.subarray(1);
        continue;
      }

      const frameLength = 8 + payloadLength;
      if (this.buffer.length < frameLength) break;

      const candidate = this.buffer.subarray(0, frameLength);
      const expected = candidate.readUInt16LE(6 + payloadLength);
      const actual = crc16Modbus(candidate.subarray(2, 6 + payloadLength));
      if (expected !== actual || candidate[2] !== VERSION) {
        this.crcErrors += 1;
        this.buffer = this.buffer.subarray(1);
        continue;
      }

      frames.push({
        version: candidate[2],
        command: candidate[3],
        sequence: candidate[4],
        payload: Buffer.from(candidate.subarray(6, 6 + payloadLength))
      });
      this.buffer = this.buffer.subarray(frameLength);
    }

    return frames;
  }
}

function decodeStatus(payload) {
  if (payload.length < 24) return null;
  const feedbackCurrentAvailable = payload[20] === 0x43 && payload[21] === 0x49;
  const status = {
    mode: payload[0],
    emergencyStop: payload[1] !== 0,
    linkOnline: payload[2] !== 0,
    motors: {
      driverReady: payload[22] !== 0,
      trackOnline: (payload[3] & 0x01) !== 0,
      armorOnline: (payload[3] & 0x02) !== 0,
      trackTargetRpm: payload.readInt16LE(4),
      trackSpeedRpm: payload.readInt16LE(6),
      trackFeedbackCurrent: feedbackCurrentAvailable ? payload.readInt16LE(8) : null,
      armorTargetRpm: payload.readInt16LE(10),
      armorSpeedRpm: payload.readInt16LE(12),
      armorFeedbackCurrent: feedbackCurrentAvailable ? payload.readInt16LE(14) : null,
      trackEcd: payload.readUInt16LE(16),
      armorEcd: payload.readUInt16LE(18)
    },
    ranging: null
  };
  if (payload.length >= 29) {
    const flags = payload[23];
    status.ranging = {
      initialized: (flags & 0x01) !== 0,
      leftValid: (flags & 0x02) !== 0,
      rightValid: (flags & 0x04) !== 0,
      leftEndpoint: (flags & 0x08) !== 0,
      rightEndpoint: (flags & 0x10) !== 0,
      leftAlive: payload.length >= 35 ? (flags & 0x20) !== 0 : (flags & 0x02) !== 0,
      rightAlive: payload.length >= 35 ? (flags & 0x40) !== 0 : (flags & 0x04) !== 0,
      leftMm: payload.readUInt16LE(24),
      rightMm: payload.readUInt16LE(26),
      autoDirection: payload.readInt8(28),
      leftRangeStatus: payload.length >= 35 ? payload[33] : 0,
      rightRangeStatus: payload.length >= 35 ? payload[34] : 0
    };
  }
  if (payload.length >= 33) {
    status.autoSpeeds = {
      trackRpm: payload.readUInt16LE(29),
      armorRpm: payload.readUInt16LE(31)
    };
  }
  if (payload.length >= 37) {
    status.remoteTrackFullScaleRpm = payload.readUInt16LE(35);
  }
  if (payload.length >= 39) {
    status.remoteArmorFullScaleRpm = payload.readUInt16LE(37);
  }
  return status;
}

module.exports = { CMD, MODE, FrameDecoder, encodeFrame, decodeStatus };
