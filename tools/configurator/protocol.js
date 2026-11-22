/*
 * protocol.js - Protokoll der eigenen Firmware (USB-Befehlszeile) fuer den Configurator
 *
 * Laeuft im Browser (index.html) und in Node (test/test_protocol.js).
 * Gegenstueck in der Firmware: src/app/usb_cli.c, src/core/telemetry.c, src/core/logrec.c
 */
(function (root) {
  'use strict';
  const FC = {};

  /* ---------- Arming-Sperrgruende (src/core/flight.h, ARM_BLOCK_...) ---------- */
  FC.ARM_BLOCKS = [
    { bit: 1 << 0, key: 'NO_RX', text: 'Kein Empfänger / Failsafe', hint: 'Fernsteuerung einschalten, Empfänger gebunden?' },
    { bit: 1 << 1, key: 'SWITCH', text: 'Arm-Schalter zuerst aus', hint: 'Arm-Schalter (Kanal 5) aus- und wieder einschalten' },
    { bit: 1 << 2, key: 'GYRO_CAL', text: 'Gyro kalibriert', hint: 'Drohne ca. 1 s still hinstellen' },
    { bit: 1 << 3, key: 'ANGLE', text: 'Zu schräg', hint: 'Drohne flach hinstellen (arm_angle_max)' },
    { bit: 1 << 4, key: 'THROTTLE', text: 'Gas nicht unten', hint: 'Gas-Stick ganz nach unten' },
    { bit: 1 << 5, key: 'USB', text: 'USB-Download läuft', hint: 'Warten, bis Download/Löschen fertig ist' },
    { bit: 1 << 6, key: 'ACC_CAL', text: 'ACC-Kalibrierung läuft', hint: 'Nicht bewegen' },
    { bit: 1 << 7, key: 'MOTOR_TEST', text: 'Motortest aktiv', hint: 'Motortest beenden' },
    { bit: 1 << 8, key: 'MOTOR_MAP', text: 'Motor-Zuordnung ungültig', hint: 'motor1_out..motor4_out: jeder Ausgang genau einmal' },
    { bit: 1 << 9, key: 'GYRO_FAIL', text: 'Gyro antwortet nicht', hint: 'Hardwarefehler – neu starten' },
  ];
  FC.armBlockList = function (mask) {
    return FC.ARM_BLOCKS.filter(b => (mask & b.bit) !== 0);
  };

  /* ---------- CRSF-Kanaele ---------- */
  FC.CRSF_MIN = 172; FC.CRSF_MID = 992; FC.CRSF_MAX = 1811;
  /* wie Betaflight: 172 -> 988 us, 992 -> 1500 us, 1811 -> 2012 us */
  FC.crsfToUs = v => Math.round((v - 992) * 5 / 8 + 1500);
  FC.crsfToBipolar = v => Math.max(-1, Math.min(1, (v - FC.CRSF_MID) / (0.5 * (FC.CRSF_MAX - FC.CRSF_MIN))));
  FC.crsfToUnipolar = v => Math.max(0, Math.min(1, (v - FC.CRSF_MIN) / (FC.CRSF_MAX - FC.CRSF_MIN)));

  /* ---------- Live-Daten "$H,..." / "$T,..." ---------- */
  /* Skalierung der ganzzahligen Felder (src/core/telemetry.h) */
  const SCALE = {
    att_r: 0.1, att_p: 0.1, att_y: 0.1, gyro_r: 0.1, gyro_p: 0.1, gyro_y: 0.1,
    acc_x: 0.001, acc_y: 0.001, acc_z: 0.001, sp_r: 0.1, sp_p: 0.1, sp_y: 0.1,
    thr: 0.001, vbat_mv: 0.001, loop_avg_x10: 0.1,
  };
  const RENAME = { vbat_mv: 'vbat', loop_avg_x10: 'loop_avg_us' };

  FC.parseHeader = function (line) {
    if (!line.startsWith('$H,')) return null;
    return line.slice(3).split(',');
  };

  FC.parseTelemetry = function (line, names) {
    if (!line.startsWith('$T,') || !names) return null;
    const parts = line.slice(3).split(',');
    if (parts.length !== names.length) return null;
    const t = {};
    for (let i = 0; i < names.length; i++) {
      const n = names[i];
      const v = Number(parts[i]);
      if (!Number.isFinite(v)) return null;
      t[RENAME[n] || n] = SCALE[n] ? v * SCALE[n] : v;
    }
    t.armed = !!t.armed; t.failsafe = !!t.failsafe; t.airmode = !!t.airmode;
    t.calibrated = !!t.calibrated; t.motor_test = !!t.motor_test;
    t.angle_mode = t.mode === 1;
    t.ch = [t.ch1, t.ch2, t.ch3, t.ch4, t.ch5, t.ch6, t.ch7, t.ch8];
    t.motors = [t.m1, t.m2, t.m3, t.m4];
    t.outputs = [t.out1, t.out2, t.out3, t.out4];
    return t;
  };

  /* ---------- Parameter "$P,name,wert,min,max,nachkomma,standard" ---------- */
  FC.parseParam = function (line) {
    if (!line.startsWith('$P,')) return null;
    const p = line.slice(3).split(',');
    if (p.length < 6) return null;
    return {
      name: p[0], value: Number(p[1]), min: Number(p[2]), max: Number(p[3]),
      decimals: Number(p[4]), def: Number(p[5]),
    };
  };
  FC.formatValue = (v, dec) => Number(v).toFixed(dec);

  FC.parseVersion = function (line) {
    if (!line.startsWith('$V,')) return null;
    const out = {};
    for (const kv of line.slice(3).split(',')) {
      const i = kv.indexOf('=');
      if (i > 0) out[kv.slice(0, i)] = kv.slice(i + 1);
    }
    return out;
  };

  /* "info" -> Objekt (key=value-Paare aller Zeilen, mit Abschnittsname als Praefix) */
  FC.parseInfo = function (lines) {
    const o = {};
    for (const raw of lines) {
      const line = raw.trim();
      const sp = line.indexOf(' ');
      if (sp < 0) continue;
      const section = line.slice(0, sp);
      const rest = line.slice(sp + 1);
      if (section === 'fw') { o.fw = rest; continue; }
      if (section === 'clock') { o.clock = rest; continue; }
      if (section === 'config') { o.config = rest; continue; }
      if (section === 'vbat') { o.vbat = parseFloat(rest); }
      if (section === 'arm_block') { o.arm_block = parseInt(rest, 16); continue; }
      const re = /([a-z_]+)=([^\s]+)/g;
      let m;
      while ((m = re.exec(rest)) !== null) {
        const key = section + '_' + m[1];
        const num = Number(m[2]);
        o[key] = Number.isFinite(num) ? num : m[2];
      }
    }
    return o;
  };

  /* ---------- Zeilen aus dem Bytestrom, mit Binaerblock fuer "dump" ---------- */
  class StreamSplitter {
    constructor(onLine, onBinary) {
      this.onLine = onLine;
      this.onBinary = onBinary;       /* (Uint8Array komplett) */
      this.buf = [];
      this.binLeft = 0;
      this.bin = null; this.binPos = 0;
      this.onProgress = null;
      this.expectBinary = false;      /* nur nach einem "dump"-Befehl auf "DUMP n" reagieren */
    }
    push(bytes) {
      let i = 0;
      while (i < bytes.length) {
        if (this.binLeft > 0) {
          const n = Math.min(this.binLeft, bytes.length - i);
          this.bin.set(bytes.subarray(i, i + n), this.binPos);
          this.binPos += n; this.binLeft -= n; i += n;
          if (this.onProgress) this.onProgress(this.binPos, this.bin.length);
          if (this.binLeft === 0) {
            const b = this.bin; this.bin = null;
            this.onBinary && this.onBinary(b);
          }
          continue;
        }
        const c = bytes[i++];
        if (c === 10) {
          let s = '';
          for (const ch of this.buf) s += String.fromCharCode(ch);
          this.buf = [];
          s = s.replace(/\r$/, '');
          const m = this.expectBinary && /^DUMP (\d+)$/.exec(s);
          if (m) {
            this.expectBinary = false;
            const n = Number(m[1]);
            this.onLine(s);
            if (n > 0) { this.bin = new Uint8Array(n); this.binPos = 0; this.binLeft = n; }
            else this.onBinary && this.onBinary(new Uint8Array(0));
            continue;
          }
          this.onLine(s);
        } else if (this.buf.length < 4096) {
          this.buf.push(c);
        }
      }
    }
  }
  FC.StreamSplitter = StreamSplitter;

  /* ---------- Flugdaten (src/core/logrec.h) ---------- */
  const REC = 32, T_DATA = 0xA5, T_HEADER = 0x5A, T_PAD = 0x00;

  const CRC_TABLE = (() => {
    const t = new Uint8Array(256);
    for (let i = 0; i < 256; i++) {
      let c = i;
      for (let b = 0; b < 8; b++) c = (c & 0x80) ? ((c << 1) ^ 0xD5) & 0xFF : (c << 1) & 0xFF;
      t[i] = c;
    }
    return t;
  })();
  FC.crc8 = function (bytes, start = 0, len = bytes.length - start) {
    let c = 0;
    for (let i = start; i < start + len; i++) c = CRC_TABLE[c ^ bytes[i]];
    return c;
  };

  FC.decodeLog = function (data) {
    const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
    const flights = [];
    let current = null, crcErrors = 0, unknown = 0;
    const end = data.length - (data.length % REC);
    for (let pos = 0; pos < end; pos += REC) {
      const t = data[pos];
      if (t === 0xFF) {
        let erased = true;
        for (let k = 0; k < REC; k++) if (data[pos + k] !== 0xFF) { erased = false; break; }
        if (erased) break;
      }
      if (FC.crc8(data, pos, 31) !== data[pos + 31]) { crcErrors++; continue; }
      if (t === T_PAD) continue;
      if (t === T_HEADER) {
        const v = data[pos + 1];
        const h = {
          format_version: v,
          git_hash: dv.getUint32(pos + 2, true).toString(16).padStart(8, '0'),
          loop_hz: dv.getUint16(pos + 6, true),
          log_hz: dv.getUint16(pos + 8, true),
          pid: {
            roll: [data[pos + 10], data[pos + 11], data[pos + 12]],
            pitch: [data[pos + 13], data[pos + 14], data[pos + 15]],
            yaw: [data[pos + 16], data[pos + 17], data[pos + 18]],
          },
          max_rate_dps: dv.getUint16(pos + 19, true),
          gyro_lpf_hz: data[pos + 21],
          dterm_lpf_hz: data[pos + 22],
        };
        if (v >= 2) {
          h.ff = { roll: dv.getUint16(pos + 23, true), pitch: dv.getUint16(pos + 25, true), yaw: dv.getUint16(pos + 27, true) };
          const f = data[pos + 29];
          h.iterm_relax = !!(f & 1); h.tpa = !!(f & 2); h.runaway_prevention = !!(f & 4);
        }
        current = { index: flights.length + 1, header: h, rows: [] };
        flights.push(current);
        continue;
      }
      if (t === T_DATA) {
        if (!current) { current = { index: flights.length + 1, header: {}, rows: [] }; flights.push(current); }
        current.rows.push([
          dv.getUint32(pos + 2, true),                                   // t_us
          dv.getInt16(pos + 6, true) / 10, dv.getInt16(pos + 8, true) / 10, dv.getInt16(pos + 10, true) / 10,
          dv.getInt16(pos + 12, true) / 10, dv.getInt16(pos + 14, true) / 10, dv.getInt16(pos + 16, true) / 10,
          dv.getUint16(pos + 26, true) / 10000,                         // throttle
          dv.getUint16(pos + 18, true), dv.getUint16(pos + 20, true),
          dv.getUint16(pos + 22, true), dv.getUint16(pos + 24, true),   // m1..m4
          data[pos + 1],                                                 // flags
          dv.getUint16(pos + 28, true) / 1000,                          // vbat
          data[pos + 30],                                                // lq
        ]);
        continue;
      }
      unknown++;
    }
    for (const f of flights) {
      const n = f.rows.length;
      f.samples = n;
      f.duration_s = n > 1 ? (((f.rows[n - 1][0] - f.rows[0][0]) >>> 0) / 1e6) : 0;
    }
    return { flights: flights.filter(f => f.rows.length > 0), crcErrors, unknown };
  };

  /* gleiche Spalten wie tools/analysis/buildb_log.py (fc_analysis.load_buildb_csv) */
  FC.CSV_COLUMNS = ['t_us', 'sp_roll', 'sp_pitch', 'sp_yaw', 'gyro_roll', 'gyro_pitch', 'gyro_yaw',
    'throttle', 'm1', 'm2', 'm3', 'm4', 'flags', 'vbat', 'lq'];
  FC.flightToCsv = function (f) {
    const lines = [FC.CSV_COLUMNS.join(',')];
    for (const r of f.rows) lines.push(r.map(v => (Number.isInteger(v) ? String(v) : String(+v.toFixed(4)))).join(','));
    return lines.join('\n') + '\n';
  };
  FC.flightMeta = function (f) {
    return Object.assign({}, f.header, { samples: f.samples, duration_s: f.duration_s });
  };

  /* Datensaetze erzeugen (Demo-Modus und Tests), wie logrec_encode_* */
  FC.encodeHeader = function (h) {
    const b = new Uint8Array(REC), dv = new DataView(b.buffer);
    b[0] = T_HEADER; b[1] = 2;
    dv.setUint32(2, parseInt(h.git_hash || '0', 16) >>> 0, true);
    dv.setUint16(6, h.loop_hz || 1000, true); dv.setUint16(8, h.log_hz || 1000, true);
    const pid = h.pid || { roll: [45, 80, 30], pitch: [47, 84, 34], yaw: [45, 80, 0] };
    [...pid.roll, ...pid.pitch, ...pid.yaw].forEach((v, i) => { b[10 + i] = v; });
    dv.setUint16(19, h.max_rate_dps || 400, true);
    b[21] = h.gyro_lpf_hz || 150; b[22] = h.dterm_lpf_hz || 75;
    b[29] = 4;
    b[31] = FC.crc8(b, 0, 31);
    return b;
  };
  FC.encodeData = function (d) {
    const b = new Uint8Array(REC), dv = new DataView(b.buffer);
    const i16 = v => Math.max(-32768, Math.min(32767, Math.round(v * 10)));
    b[0] = T_DATA; b[1] = d.flags;
    dv.setUint32(2, d.t_us >>> 0, true);
    for (let a = 0; a < 3; a++) dv.setInt16(6 + 2 * a, i16(d.sp[a]), true);
    for (let a = 0; a < 3; a++) dv.setInt16(12 + 2 * a, i16(d.gyro[a]), true);
    for (let m = 0; m < 4; m++) dv.setUint16(18 + 2 * m, d.motor[m], true);
    dv.setUint16(26, Math.round(d.throttle * 10000), true);
    dv.setUint16(28, Math.round(d.vbat * 1000), true);
    b[30] = d.lq;
    b[31] = FC.crc8(b, 0, 31);
    return b;
  };
  FC.encodePad = function () {
    const b = new Uint8Array(REC);
    b[31] = FC.crc8(b, 0, 31);
    return b;
  };

  /* ---------- Stick-Kennlinie (src/core/rates.c, Betaflight "Actual Rates") ---------- */
  FC.actualRate = function (stick, center, max, expo) {
    const x = Math.max(-1, Math.min(1, stick));
    const ax = Math.abs(x);
    const expof = ax * (Math.pow(x, 5) * expo + x * (1 - expo));
    const stickMovement = Math.max(0, max - center);
    return x * center + stickMovement * expof;
  };

  if (typeof module !== 'undefined' && module.exports) module.exports = FC;
  else root.FC = FC;
})(typeof window !== 'undefined' ? window : globalThis);
