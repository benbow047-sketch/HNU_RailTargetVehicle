const { app, BrowserWindow, ipcMain } = require('electron');
const { SerialPort } = require('serialport');
const path = require('path');
const { CMD, MODE, FrameDecoder, encodeFrame, decodeStatus } = require('./protocol');

let mainWindow = null;
let port = null;
let sequence = 0;
let heartbeatTimer = null;
let decoder = new FrameDecoder();
let disconnecting = false;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 520,
    height: 920,
    minWidth: 460,
    minHeight: 760,
    title: '轨道靶车控制',
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      nodeIntegration: false,
      contextIsolation: true
    }
  });
  mainWindow.loadFile('index.html');
}

function nextSequence() {
  sequence = (sequence + 1) & 0xff;
  return sequence;
}

function sendFrame(command, payload = Buffer.alloc(0)) {
  if (!port || !port.isOpen) throw new Error('串口未连接');
  const frame = encodeFrame(command, nextSequence(), payload);
  return new Promise((resolve, reject) => {
    port.write(frame, (error) => {
      if (error) return reject(error);
      port.drain((drainError) => drainError ? reject(drainError) : resolve(true));
    });
  });
}

function publishConnection(connected, portPath = null, message = '') {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('serial-connection', { connected, port: portPath, message });
  }
}

function handleIncoming(data) {
  if (disconnecting) return;
  for (const frame of decoder.push(data)) {
    const event = {
      command: frame.command,
      sequence: frame.sequence,
      payload: Array.from(frame.payload)
    };
    if (frame.command === CMD.ACK && frame.payload.length >= 2) {
      event.ack = { command: frame.payload[0], status: frame.payload[1] };
    } else if (frame.command === CMD.STATUS) {
      event.status = decodeStatus(frame.payload);
    }
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('device-frame', event);
    }
  }
}

function stopHeartbeat() {
  if (heartbeatTimer) clearInterval(heartbeatTimer);
  heartbeatTimer = null;
}

function startHeartbeat() {
  stopHeartbeat();
  heartbeatTimer = setInterval(() => {
    if (port && port.isOpen) sendFrame(CMD.HEARTBEAT).catch(() => {});
  }, 200);
}

function closePort() {
  stopHeartbeat();
  return new Promise((resolve) => {
    if (!port || !port.isOpen) {
      port = null;
      resolve();
      return;
    }
    const closingPort = port;
    port = null;
    closingPort.close(() => resolve());
  });
}

ipcMain.handle('serial-list', async () => {
  const ports = await SerialPort.list();
  return ports.map((item) => ({
    path: item.path,
    name: item.friendlyName || item.manufacturer || item.path
  }));
});

ipcMain.handle('serial-connect', async (_event, portPath) => {
  disconnecting = false;
  await closePort();
  decoder = new FrameDecoder();
  port = new SerialPort({
    path: portPath,
    baudRate: 115200,
    dataBits: 8,
    stopBits: 1,
    parity: 'none',
    autoOpen: false
  });

  await new Promise((resolve, reject) => {
    port.open((error) => error ? reject(error) : resolve());
  });

  port.on('data', handleIncoming);
  port.on('error', (error) => publishConnection(false, null, error.message));
  port.on('close', () => {
    stopHeartbeat();
    publishConnection(false, null, '串口已断开');
  });

  publishConnection(true, portPath, '串口已连接');
  startHeartbeat();
  await sendFrame(CMD.PING);
  await sendFrame(CMD.SET_MODE, Buffer.from([MODE.REMOTE]));
  await sendRemote(0, 0);
  return { ok: true };
});

ipcMain.handle('serial-disconnect', async () => {
  disconnecting = true;
  publishConnection(false, null, '正在断开...');
  try {
    if (port && port.isOpen) {
      try { await sendRemote(0, 0); } catch (_) {}
      try { await sendFrame(CMD.SET_MODE, Buffer.from([MODE.STOP])); } catch (_) {}
    }
    await closePort();
    publishConnection(false, null, '已主动断开');
    return { ok: true };
  } finally {
    disconnecting = false;
  }
});

function clampControl(value) {
  return Math.max(-1000, Math.min(1000, Math.round(Number(value) || 0)));
}

async function sendRemote(move, rotate) {
  const payload = Buffer.alloc(4);
  payload.writeInt16LE(clampControl(move), 0);
  payload.writeInt16LE(clampControl(rotate), 2);
  await sendFrame(CMD.SET_REMOTE, payload);
}

async function sendAutoSpeed(trackRpm, armorRpm) {
  const track = Math.round(Number(trackRpm));
  const armor = Math.round(Number(armorRpm));
  if (!Number.isFinite(track) || track < 1500 || track > 5500 ||
      !Number.isFinite(armor) || armor < 50 || armor > 400) {
    throw new Error('自动速度超出安全范围');
  }
  const payload = Buffer.alloc(4);
  payload.writeUInt16LE(track, 0);
  payload.writeUInt16LE(armor, 2);
  await sendFrame(CMD.SET_AUTO_SPEED, payload);
}

async function sendRemoteMapping(trackRpm, armorRpm) {
  const track = Number(trackRpm);
  const armor = Number(armorRpm);
  if (!Number.isInteger(track) || track < 1500 || track > 5500 ||
      !Number.isInteger(armor) || armor < 20 || armor > 400) {
    throw new Error('遥控映射值超出安全范围');
  }
  const payload = Buffer.alloc(4);
  payload.writeUInt16LE(track, 0);
  payload.writeUInt16LE(armor, 2);
  await sendFrame(CMD.SET_REMOTE_MAPPING, payload);
}

ipcMain.handle('send-mode', (_event, mode) => sendFrame(CMD.SET_MODE, Buffer.from([mode])));
ipcMain.handle('send-remote', (_event, move, rotate) => sendRemote(move, rotate));
ipcMain.handle('send-auto-speed', (_event, trackRpm, armorRpm) => sendAutoSpeed(trackRpm, armorRpm));
ipcMain.handle('send-remote-mapping', (_event, trackRpm, armorRpm) => sendRemoteMapping(trackRpm, armorRpm));
ipcMain.handle('send-estop', () => sendFrame(CMD.ESTOP));
ipcMain.handle('send-clear-estop', () => sendFrame(CMD.CLEAR_ESTOP));
ipcMain.handle('send-ping', () => sendFrame(CMD.PING));

app.whenReady().then(createWindow);
app.on('window-all-closed', () => {
  stopHeartbeat();
  if (port && port.isOpen) port.close();
  app.quit();
});
