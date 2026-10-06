const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('targetSerial', {
  list: () => ipcRenderer.invoke('serial-list'),
  connect: (path) => ipcRenderer.invoke('serial-connect', path),
  disconnect: () => ipcRenderer.invoke('serial-disconnect'),
  setMode: (mode) => ipcRenderer.invoke('send-mode', mode),
  setRemote: (move, rotate) => ipcRenderer.invoke('send-remote', move, rotate),
  setAutoSpeed: (trackRpm, armorRpm) => ipcRenderer.invoke('send-auto-speed', trackRpm, armorRpm),
  setRemoteMapping: (trackRpm, armorRpm) => ipcRenderer.invoke('send-remote-mapping', trackRpm, armorRpm),
  emergencyStop: () => ipcRenderer.invoke('send-estop'),
  clearEmergencyStop: () => ipcRenderer.invoke('send-clear-estop'),
  ping: () => ipcRenderer.invoke('send-ping'),
  onConnection: (callback) => ipcRenderer.on('serial-connection', (_event, data) => callback(data)),
  onDeviceFrame: (callback) => ipcRenderer.on('device-frame', (_event, data) => callback(data))
});
