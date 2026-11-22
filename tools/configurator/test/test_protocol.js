/*
 * test_protocol.js - Tests des Configurator-Protokolls gegen die Ausgaben der Firmware
 *
 *   cmake -S tests -B build-tests && cmake --build build-tests && ./build-tests/test_core build-tests/sample_log.bin
 *   node tools/configurator/test/test_protocol.js
 *
 * Verwendet Dateien, die die Host-Tests der Firmware schreiben (build-tests/):
 *   tlm_sample.txt     Live-Daten-Zeilen aus src/core/telemetry.c
 *   params_sample.txt  Parameterliste aus src/core/params.c
 *   sample_log.bin     Flugdaten aus src/core/logrec.c
 */
'use strict';
const fs = require('fs');
const path = require('path');
const { execFileSync } = require('child_process');
const FC = require('../protocol.js');
const MockFC = require('../mock.js');

const ROOT = path.resolve(__dirname, '../../..');
const BT = path.join(ROOT, 'build-tests');
let checks = 0, failures = 0;
const ok = (cond, msg) => { checks++; if (!cond) { failures++; console.log('  FEHLER: ' + msg); } };
const near = (a, b, tol, msg) => ok(Math.abs(a - b) <= tol, `${msg}: ${a} != ${b}`);

function testTelemetry() {
  console.log('Live-Daten (telemetry.c)');
  const [h, t] = fs.readFileSync(path.join(BT, 'tlm_sample.txt'), 'utf8').trim().split('\n');
  const names = FC.parseHeader(h);
  ok(names && names.length === 44, 'Anzahl Felder ' + (names && names.length));
  const v = FC.parseTelemetry(t, names);
  ok(v, 'Zeile lesbar');
  near(v.t_ms, 12345, 0, 't_ms'); ok(v.armed === true, 'armed');
  near(v.att_r, -12.3, 1e-9, 'att_r'); near(v.gyro_y, 250.1, 1e-9, 'gyro_y'); near(v.acc_z, 0.998, 1e-9, 'acc_z');
  near(v.thr, 0.5, 1e-9, 'thr'); near(v.motors[3], 2047, 0, 'm4'); near(v.ch[4], 1811, 0, 'ch5');
  near(v.vbat, 16.8, 1e-9, 'vbat'); near(v.rx_uart, 2, 0, 'rx_uart'); near(v.outputs[1], 99, 0, 'out2');
  const blocks = FC.armBlockList(v.arm_block).map(b => b.key);
  ok(blocks.length === 1 && blocks[0] === 'THROTTLE', 'arm_block ' + blocks);
  ok(FC.parseTelemetry(t.slice(0, -3), names) === null, 'abgeschnittene Zeile wird verworfen');
}

function testParams() {
  console.log('Parameterliste (params.c) und Demo-Modus');
  const lines = fs.readFileSync(path.join(BT, 'params_sample.txt'), 'utf8').trim().split('\n');
  const ps = lines.map(FC.parseParam);
  ok(ps.every(Boolean), 'alle Zeilen lesbar');
  const p = ps.find(x => x.name === 'p_roll');
  ok(p.value === 52 && p.def === 45 && p.min === 0 && p.max === 250 && p.decimals === 0, 'p_roll');
  /* Demo-Liste muss gleich wie die Firmware sein */
  ok(MockFC.PARAMS.length === ps.length, `Demo hat ${MockFC.PARAMS.length}, Firmware ${ps.length} Parameter`);
  ps.forEach((x, i) => {
    const m = MockFC.PARAMS[i] || [];
    ok(m[0] === x.name && m[1] === x.min && m[2] === x.max && m[3] === x.decimals && m[4] === x.def, 'Demo-Parameter ' + x.name);
  });
  for (const name of ['motor1_out', 'rx_uart', 'vbat_scale', 'crsf_telemetry', 'gyro_sign_roll']) ok(ps.some(x => x.name === name), 'fehlt: ' + name);
}

function testLog() {
  console.log('Flugdaten (logrec.c) gegen buildb_log.py');
  const bin = fs.readFileSync(path.join(BT, 'sample_log.bin'));
  const res = FC.decodeLog(new Uint8Array(bin));
  ok(res.flights.length === 1 && res.crcErrors === 0, `Fluege ${res.flights.length}, CRC ${res.crcErrors}`);
  const f = res.flights[0];
  ok(f.samples === 20, 'Anzahl ' + f.samples);
  const csvJs = FC.flightToCsv(f);
  /* Python-Decoder als Referenz */
  const py = `
import sys; sys.path.insert(0, ${JSON.stringify(path.join(ROOT, 'tools/analysis'))})
import buildb_log as b
r = b.decode(open(${JSON.stringify(path.join(BT, 'sample_log.bin'))}, 'rb').read())
print(r.flights[0].to_dataframe().to_csv(index=False), end='')
`;
  let csvPy = null;
  try { csvPy = execFileSync('python3', ['-c', py], { encoding: 'utf8' }); } catch (e) { console.log('  (Python/pandas nicht verfuegbar, Vergleich uebersprungen)'); }
  if (csvPy) {
    const a = csvJs.trim().split('\n'), b = csvPy.trim().split('\n');
    ok(a[0] === b[0], `Spalten ${a[0]} != ${b[0]}`);
    ok(a.length === b.length, 'Zeilen');
    for (let i = 1; i < Math.min(a.length, b.length); i++) {
      const x = a[i].split(',').map(Number), y = b[i].split(',').map(Number);
      ok(x.length === y.length && x.every((v, k) => Math.abs(v - y[k]) < 1e-6), `Zeile ${i}: ${a[i]} != ${b[i]}`);
    }
  }
  /* eigene Datensaetze: Kopf + Daten + Fuellsatz + Fehler + geloescht */
  const parts = [FC.encodeHeader({ git_hash: 'abcdef12' }),
    FC.encodeData({ t_us: 100, flags: 1, sp: [10, -20.5, 0], gyro: [9.9, 0, -1], motor: [48, 100, 200, 2047], throttle: 0.25, vbat: 15.5, lq: 99 }),
    FC.encodePad(), FC.encodeData({ t_us: 1100, flags: 1, sp: [0, 0, 0], gyro: [0, 0, 0], motor: [0, 0, 0, 0], throttle: 0, vbat: 0, lq: 0 })];
  const bad = FC.encodeData({ t_us: 2100, flags: 1, sp: [0, 0, 0], gyro: [0, 0, 0], motor: [0, 0, 0, 0], throttle: 0, vbat: 0, lq: 0 });
  bad[5] ^= 1; parts.push(bad);
  parts.push(new Uint8Array(32).fill(0xFF), FC.encodeData({ t_us: 9, flags: 1, sp: [0, 0, 0], gyro: [0, 0, 0], motor: [0, 0, 0, 0], throttle: 0, vbat: 0, lq: 0 }));
  const all = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let o = 0; for (const p of parts) { all.set(p, o); o += p.length; }
  const r2 = FC.decodeLog(all);
  ok(r2.flights.length === 1 && r2.flights[0].samples === 2 && r2.crcErrors === 1, 'eigene Datensaetze');
  ok(r2.flights[0].header.git_hash === 'abcdef12', 'git_hash');
  near(r2.flights[0].rows[0][2], -20.5, 1e-9, 'sp_pitch');
  near(r2.flights[0].rows[0][7], 0.25, 1e-9, 'throttle');
}

function testSplitter() {
  console.log('Zeilen + Binaerblock (dump)');
  const lines = []; let bin = null;
  const sp = new FC.StreamSplitter(l => lines.push(l), b => { bin = b; });
  const payload = new Uint8Array(1000).map((_, i) => (i * 7) & 0xFF);
  payload[10] = 10; payload[11] = 13;                    /* Zeilenende-Bytes in den Binaerdaten */
  const enc = s => new TextEncoder().encode(s);
  const stream = new Uint8Array([...enc('$T,1,2\r\nDUMP 1000\r\n'), ...payload, ...enc('\r\nOK\r\n#7\r\n')]);
  sp.expectBinary = true;
  for (let i = 0; i < stream.length;) { const n = 1 + Math.floor(Math.random() * 97); sp.push(stream.subarray(i, i + n)); i += n; }
  ok(bin && bin.length === 1000 && bin.every((v, i) => v === payload[i]), 'Binaerdaten unveraendert');
  ok(JSON.stringify(lines) === JSON.stringify(['$T,1,2', 'DUMP 1000', '', 'OK', '#7']), 'Zeilen ' + JSON.stringify(lines));
  /* ohne expectBinary wird "DUMP" nicht als Binaer behandelt (z. B. Text im CLI) */
  const l2 = []; const sp2 = new FC.StreamSplitter(l => l2.push(l), () => ok(false, 'kein Binaer erwartet'));
  sp2.push(enc('DUMP 3\r\nabc\r\n'));
  ok(l2.length === 2, 'DUMP ohne Anforderung');
}

function testInfoAndRates() {
  console.log('info, Rates');
  const i = FC.parseInfo(['fw 1234ABCD', 'flash jedec=EF4018 ok=1 capacity=16711680 used=320 percent=0 full=0',
    'rx uart=2 frames=1500 crc_errors=3 lq=100', 'loop avg_us=41.2 max_us=61 period_us min=996 max=1004', 'vbat 16.40 cells=4', 'arm_block 0x0012']);
  ok(i.fw === '1234ABCD' && i.flash_used === 320 && i.rx_uart === 2 && i.loop_avg_us === 41.2 && i.vbat === 16.4 && i.vbat_cells === 4, 'info');
  ok(i.arm_block === 0x12, 'arm_block hex');
  /* Actual Rates wie rates.c: linear 400 -> 0.5 Stick = 200 dps; mit Expo bleibt Vollausschlag = max */
  near(FC.actualRate(0.5, 400, 400, 0), 200, 1e-9, 'linear');
  near(FC.actualRate(1, 200, 670, 0.54), 670, 1e-9, 'Vollausschlag');
  near(FC.actualRate(-1, 200, 670, 0.54), -670, 1e-9, 'Vollausschlag negativ');
  near(FC.crsfToUs(172), 988, 0, '172 -> 988 us'); near(FC.crsfToUs(1811), 2012, 0, '1811 -> 2012 us');
}

async function testMock() {
  console.log('Demo-Flightcontroller');
  const m = new MockFC();
  const lines = [];
  let bin = null;
  const sp = new FC.StreamSplitter(l => lines.push(l), b => { bin = b; });
  m.onData = b => sp.push(b);
  await m.open();
  m.write('version\nset p_roll 60\nset p_roll 999\nstream 50\n');
  await new Promise(r => setTimeout(r, 250));
  ok(lines.some(l => l.startsWith('$V,')), 'version');
  ok(lines.includes('p_roll = 60'), 'set');
  ok(lines.some(l => l.startsWith('ERR Bereich')), 'Bereich');
  const names = FC.parseHeader(lines.find(l => l.startsWith('$H,')));
  const tl = lines.filter(l => l.startsWith('$T,')).map(l => FC.parseTelemetry(l, names));
  ok(tl.length >= 5 && tl.every(Boolean), 'Live-Daten ' + tl.length);
  m.write('stream 0\n'); await new Promise(r => setTimeout(r, 30));
  sp.expectBinary = true; m.write('dump\necho #1\n');
  await new Promise(r => setTimeout(r, 400));
  ok(bin && FC.decodeLog(bin).flights.length === 2, 'Demo-Flugdaten');
  ok(lines[lines.length - 1] === '#1', 'Echo nach dump');
  await m.close();
}

(async () => {
  testTelemetry(); testParams(); testLog(); testSplitter(); testInfoAndRates();
  await testMock();
  console.log(`\n${checks} Pruefungen, ${failures} Fehler`);
  process.exit(failures ? 1 : 0);
})();
