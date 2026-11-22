/*
 * main.js - Mac-App (Electron) fuer den Configurator
 * Laedt index.html und erlaubt Web Serial (USB) nur fuer die eigene Firmware.
 */
'use strict';
const { app, BrowserWindow, dialog, Menu, shell } = require('electron');
const path = require('path');
const fs = require('fs');

const FC_VID = 0x1209, FC_PID = 0x0001;            /* eigene Firmware */
const BF_VID = 0x0483, BF_PID = 0x5740;            /* Betaflight (STM32 Virtual COM Port) */
const DFU_VID = 0x0483, DFU_PID = 0xdf11;          /* STM32-Bootloader */
const hex = v => (v === undefined || v === null) ? NaN : (typeof v === 'number' ? v : (/^0x/i.test(v) ? parseInt(v, 16) : parseInt(v, 10)));
const isFc = p => hex(p.vendorId) === FC_VID && hex(p.productId) === FC_PID;
const isBf = p => hex(p.vendorId) === BF_VID && hex(p.productId) === BF_PID;

function uniquePath(dir, name) {
  let p = path.join(dir, name);
  const ext = path.extname(name), base = name.slice(0, name.length - ext.length);
  for (let i = 2; fs.existsSync(p); i++) p = path.join(dir, `${base} (${i})${ext}`);
  return p;
}

function createWindow() {
  const win = new BrowserWindow({
    width: 1400, height: 900, minWidth: 1000, minHeight: 640,
    title: 'DIY Drone Configurator', backgroundColor: '#121314',
    webPreferences: { contextIsolation: true, nodeIntegration: false, sandbox: true },
  });
  const ses = win.webContents.session;

  /* USB-Geraete (Web Serial): eigene Firmware automatisch waehlen, sonst Liste anzeigen */
  ses.on('select-serial-port', (event, portList, webContents, callback) => {
    event.preventDefault();
    const fc = portList.filter(isFc), bf = portList.filter(isBf);
    if (fc.length === 1) { callback(fc[0].portId); return; }
    if (!fc.length && bf.length === 1) { callback(bf[0].portId); return; }
    const list = fc.length ? fc : bf.length ? bf : portList;
    if (!list.length) {
      dialog.showMessageBox(win, { type: 'info', message: 'Kein Flightcontroller gefunden',
        detail: 'USB-Kabel eingesteckt? Läuft die eigene Firmware (nicht Betaflight)?' });
      callback(''); return;
    }
    const labels = list.map(p => `${p.displayName || p.portName}${p.vendorId ? ` (${Number(hex(p.vendorId)).toString(16)}:${Number(hex(p.productId)).toString(16)})` : ''}`);
    const r = dialog.showMessageBoxSync(win, { type: 'question', message: 'Gerät wählen', buttons: [...labels, 'Abbrechen'], cancelId: labels.length });
    callback(r < list.length ? list[r].portId : '');
  });
  /* WebUSB: Bootloader (DFU) zum Flashen automatisch waehlen */
  ses.on('select-usb-device', (event, details, callback) => {
    event.preventDefault();
    const dfu = details.deviceList.filter(d => d.vendorId === DFU_VID && d.productId === DFU_PID);
    if (dfu.length) { callback(dfu[0].deviceId); return; }
    dialog.showMessageBox(win, { type: 'info', message: 'Kein Flightcontroller im Bootloader gefunden',
      detail: 'Zuerst «Bootloader starten». Falls er nicht erscheint: Boot-Taste gedrückt halten und USB einstecken.' });
    callback();
  });
  ses.setPermissionCheckHandler((wc, permission) => permission === 'serial' || permission === 'usb');
  ses.setDevicePermissionHandler(details => details.deviceType === 'serial' || details.deviceType === 'usb');

  /* Downloads (CSV, Protokoll, diff) direkt in den Ordner Downloads */
  ses.on('will-download', (e, item) => {
    const dir = app.getPath('downloads');
    fs.mkdirSync(dir, { recursive: true });
    const target = uniquePath(dir, item.getFilename());
    item.setSavePath(target);
    if (process.env.FC_DEBUG) console.log('download ->', target);
    item.once('done', (ev, state) => { if (state === 'completed') app.addRecentDocument(target); });
  });

  /* Ungespeicherte Aenderungen: beim Schliessen nachfragen */
  win.webContents.on('will-prevent-unload', e => {
    const r = dialog.showMessageBoxSync(win, { type: 'warning', buttons: ['Schliessen', 'Abbrechen'], defaultId: 1, cancelId: 1,
      message: 'Ungespeicherte Änderungen', detail: 'Es gibt Einstellungen, die noch nicht im Flightcontroller gespeichert sind. Trotzdem schliessen?' });
    if (r === 0) e.preventDefault();
  });

  /* Links nach aussen im normalen Browser */
  win.webContents.setWindowOpenHandler(({ url }) => { shell.openExternal(url); return { action: 'deny' }; });
  win.loadFile(path.join(__dirname, 'index.html'));
}

Menu.setApplicationMenu(Menu.buildFromTemplate([
  { role: 'appMenu' },
  { role: 'editMenu' },
  { label: 'Ansicht', submenu: [{ role: 'reload', label: 'Neu laden' }, { role: 'togglefullscreen', label: 'Vollbild' },
    { type: 'separator' }, { role: 'zoomIn', label: 'Grösser' }, { role: 'zoomOut', label: 'Kleiner' }, { role: 'resetZoom', label: 'Originalgrösse' },
    { type: 'separator' }, { role: 'toggleDevTools', label: 'Entwicklerwerkzeuge' }] },
  { role: 'windowMenu' },
]));

app.whenReady().then(createWindow);
app.on('window-all-closed', () => app.quit());
