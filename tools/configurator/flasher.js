/*
 * flasher.js - Firmware flashen ohne Zusatzprogramme (wie der Betaflight Configurator)
 *
 *   1. Betaflight sichern: CLI ueber USB (Web Serial, 0483:5740), "diff all" mitschreiben
 *   2. Bootloader: Betaflight-CLI "bl" bzw. eigene Firmware "dfu"
 *   3. STM32-Bootloader (ROM, USB-DFU 0483:df11) ueber WebUSB: Sektoren loeschen,
 *      schreiben, zuruecklesen und vergleichen, starten
 *
 * Der ROM-Bootloader des STM32 laesst sich nicht ueberschreiben: auch nach einem
 * Fehler kann man immer wieder flashen (notfalls Boot-Taste halten und USB einstecken).
 *
 * Laeuft im Browser/Electron und in Node (test/test_flasher.js mit simuliertem Geraet).
 */
(function (root) {
  'use strict';
  const FL = {};

  /* ---------- Intel-HEX -> Speicherabschnitte ---------- */
  FL.parseHex = function (text) {
    const mem = new Map();               /* Adresse -> Byte */
    let base = 0, start = null, sawEof = false;
    const lines = text.split(/\r?\n/);
    for (let n = 0; n < lines.length; n++) {
      const l = lines[n].trim();
      if (!l) continue;
      if (l[0] !== ':') throw new Error(`HEX Zeile ${n + 1}: kein ':'`);
      const b = [];
      for (let i = 1; i < l.length; i += 2) b.push(parseInt(l.substr(i, 2), 16));
      if (b.some(x => Number.isNaN(x))) throw new Error(`HEX Zeile ${n + 1}: ungueltig`);
      const len = b[0], addr = (b[1] << 8) | b[2], type = b[3];
      if (b.length !== len + 5) throw new Error(`HEX Zeile ${n + 1}: Laenge`);
      const sum = b.reduce((s, x) => s + x, 0) & 0xFF;
      if (sum !== 0) throw new Error(`HEX Zeile ${n + 1}: Pruefsumme`);
      const data = b.slice(4, 4 + len);
      if (type === 0) data.forEach((x, i) => mem.set(base + addr + i, x));
      else if (type === 1) { sawEof = true; break; }
      else if (type === 2) base = ((data[0] << 8) | data[1]) << 4;
      else if (type === 4) base = (((data[0] << 8) | data[1]) << 16) >>> 0;
      else if (type === 5) start = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]) >>> 0;
    }
    if (!sawEof) throw new Error('HEX ohne Ende-Datensatz (Datei unvollstaendig?)');
    const addrs = [...mem.keys()].sort((a, b) => a - b);
    const blocks = [];
    for (const a of addrs) {
      const last = blocks[blocks.length - 1];
      if (last && a === last.address + last.bytes.length) last.bytes.push(mem.get(a));
      else blocks.push({ address: a, bytes: [mem.get(a)] });
    }
    const out = blocks.map(b => ({ address: b.address, data: Uint8Array.from(b.bytes) }));
    const total = out.reduce((s, b) => s + b.data.length, 0);
    return { blocks: out, start, bytes: total };
  };

  /* Sektoren des STM32F722 (512 KB): 4 x 16 KB, 1 x 64 KB, 3 x 128 KB */
  FL.F722_SECTORS = (() => {
    const s = [], sizes = [16, 16, 16, 16, 64, 128, 128, 128];
    let a = 0x08000000;
    for (const k of sizes) { s.push({ start: a, size: k * 1024 }); a += k * 1024; }
    return s;
  })();
  /* Sektoren aus dem DfuSe-Text "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg" */
  FL.parseDfuseLayout = function (name) {
    const m = /\/0x([0-9a-fA-F]+)\/(.+)$/.exec(name || '');
    if (!m) return null;
    let addr = parseInt(m[1], 16);
    const sectors = [];
    for (const part of m[2].split(',')) {
      const p = /(\d+)\*(\d+)([BKM ])([a-g])/.exec(part.trim());
      if (!p) return null;
      const size = Number(p[2]) * ({ B: 1, ' ': 1, K: 1024, M: 1048576 })[p[3]];
      for (let i = 0; i < Number(p[1]); i++) { sectors.push({ start: addr, size, mode: p[4] }); addr += size; }
    }
    return sectors;
  };
  FL.sectorsFor = function (blocks, sectors) {
    const need = [];
    for (const s of sectors) {
      const end = s.start + s.size;
      if (blocks.some(b => b.address < end && b.address + b.data.length > s.start)) need.push(s);
    }
    return need;
  };

  /* ---------- Betaflight-CLI ueber eine serielle Verbindung ---------- */
  /* io: { write(text): Promise, read(): Promise<string>  (wartet auf neue Daten, '' bei Zeitablauf) } */
  FL.bfCollect = async function (io, cmd, opt = {}) {
    const idleMs = opt.idleMs || 800, maxMs = opt.maxMs || 20000;
    if (cmd !== null) await io.write(cmd + '\r\n');
    let out = '', t0 = Date.now(), lastData = Date.now();
    for (;;) {
      const chunk = await io.read(150);
      if (chunk) { out += chunk; lastData = Date.now(); }
      const prompt = /\n# $/.test(out.replace(/\r/g, ''));
      if (prompt && Date.now() - lastData > 150) break;
      if (!chunk && Date.now() - lastData > idleMs && out) break;
      if (Date.now() - t0 > maxMs) break;
    }
    return out.replace(/\r/g, '');
  };
  FL.bfEnterCli = async function (io) {
    await io.write('#');
    const r = await FL.bfCollect(io, null, { idleMs: 1200, maxMs: 4000 });
    if (!/CLI/i.test(r) && !/# $/.test(r)) throw new Error('Betaflight-CLI antwortet nicht');
    return r;
  };
  FL.parseBfVersion = function (text) {
    const m = /# (Betaflight|INAV|Emuflight)\s*\/\s*(\S+)\s+\(([^)]+)\)\s+(\S+)/.exec(text);
    if (!m) return null;
    const target = /\b(GEPRCF722\w*)\b/.exec(text);
    return { fw: m[1], mcu: m[2], target: m[3], version: m[4], board: target ? target[1] : m[3] };
  };

  /* ---------- USB-DFU (DfuSe, STM32-Bootloader) ---------- */
  const DNLOAD = 1, UPLOAD = 2, GETSTATUS = 3, CLRSTATUS = 4, ABORT = 6;
  const ST = { appIDLE: 0, dfuIDLE: 2, dnBUSY: 4, dnIDLE: 5, manifest: 7, upIDLE: 9, ERROR: 10 };
  const sleep = ms => new Promise(r => setTimeout(r, ms));

  class Dfu {
    /* dev: WebUSB USBDevice (oder Test-Attrappe mit gleicher Schnittstelle) */
    constructor(dev) { this.dev = dev; this.intf = 0; this.alt = 0; this.xfer = 2048; this.layout = null; this.log = () => {}; }

    async open() {
      const d = this.dev;
      await d.open();
      if (d.configuration === null) await d.selectConfiguration(1);
      /* Schnittstelle mit Klasse DFU (0xFE/0x01), alt-Setting "Internal Flash" */
      let found = null;
      for (const i of d.configuration.interfaces) {
        for (const a of i.alternates) {
          if (a.interfaceClass === 0xFE && a.interfaceSubclass === 0x01) {
            const name = a.interfaceName || '';
            if (!found || /internal flash/i.test(name)) found = { intf: i.interfaceNumber, alt: a.alternateSetting, name };
          }
        }
      }
      if (!found) throw new Error('Kein DFU-Interface gefunden');
      this.intf = found.intf; this.alt = found.alt;
      await d.claimInterface(this.intf);
      await d.selectAlternateInterface(this.intf, this.alt);
      this.layout = FL.parseDfuseLayout(found.name) || FL.F722_SECTORS;
      this.name = found.name;
      await this.toIdle();
    }
    async close() { try { await this.dev.releaseInterface(this.intf); } catch (e) { /* egal */ } try { await this.dev.close(); } catch (e) { /* egal */ } }

    _setup(request, value) { return { requestType: 'class', recipient: 'interface', request, value, index: this.intf }; }
    async _out(request, value, data) {
      const r = await this.dev.controlTransferOut(this._setup(request, value), data || new Uint8Array(0));
      if (r.status !== 'ok') throw new Error('USB-Fehler (out) ' + r.status);
    }
    async _in(request, value, len) {
      const r = await this.dev.controlTransferIn(this._setup(request, value), len);
      if (r.status !== 'ok') throw new Error('USB-Fehler (in) ' + r.status);
      return new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength);
    }
    async status() {
      const b = await this._in(GETSTATUS, 0, 6);
      return { status: b[0], poll: b[1] | (b[2] << 8) | (b[3] << 16), state: b[4] };
    }
    async toIdle() {
      for (let i = 0; i < 6; i++) {
        const s = await this.status();
        if (s.state === ST.dfuIDLE) return;
        if (s.state === ST.ERROR) await this._out(CLRSTATUS, 0);
        else await this._out(ABORT, 0);
      }
      throw new Error('Bootloader kommt nicht in den Ruhezustand');
    }
    /* nach DNLOAD: warten, bis fertig */
    async waitDone(expect) {
      for (let i = 0; i < 2000; i++) {
        const s = await this.status();
        if (s.status !== 0) throw new Error('DFU-Fehler, Status ' + s.status);
        if (s.state === ST.dnBUSY) { await sleep(Math.max(1, s.poll)); continue; }
        if (expect === undefined || s.state === expect) return s;
        if (s.state === ST.dnIDLE || s.state === ST.dfuIDLE) return s;
        await sleep(Math.max(1, s.poll));
      }
      throw new Error('DFU: Zeitueberschreitung');
    }
    async cmd(code, address) {
      const b = new Uint8Array(address === undefined ? 1 : 5);
      b[0] = code;
      if (address !== undefined) { b[1] = address & 0xFF; b[2] = (address >>> 8) & 0xFF; b[3] = (address >>> 16) & 0xFF; b[4] = (address >>> 24) & 0xFF; }
      await this._out(DNLOAD, 0, b);
      await this.waitDone(ST.dnIDLE);
    }
    async setAddress(a) { await this.cmd(0x21, a); }
    async eraseSector(a) { await this.cmd(0x41, a); }
    async writeBlock(address, data) {
      await this.setAddress(address);
      await this._out(DNLOAD, 2, data);
      await this.waitDone(ST.dnIDLE);
    }
    async readBlock(address, len) {
      await this.setAddress(address);
      await this.toIdle();
      const d = await this._in(UPLOAD, 2, len);
      await this.toIdle();
      return d;
    }
    /* Firmware starten: Adresszeiger setzen, leeres DNLOAD -> Manifest -> Reset */
    async leave(address) {
      await this.setAddress(address);
      await this._out(DNLOAD, 0, new Uint8Array(0));
      try { await this.status(); } catch (e) { /* Geraet trennt sich: normal */ }
    }

    /* Ganzer Ablauf. progress(phase, done, total) */
    async flash(image, progress = () => {}) {
      const sectors = FL.sectorsFor(image.blocks, this.layout);
      for (const b of image.blocks) {
        const end = b.address + b.data.length;
        const last = this.layout[this.layout.length - 1];
        if (b.address < this.layout[0].start || end > last.start + last.size) throw new Error('Firmware passt nicht in den Flash');
      }
      let i = 0;
      for (const s of sectors) { progress('Löschen', i++, sectors.length); await this.eraseSector(s.start); }
      progress('Löschen', sectors.length, sectors.length);
      const total = image.bytes; let done = 0;
      for (const b of image.blocks) {
        for (let off = 0; off < b.data.length; off += this.xfer) {
          const chunk = b.data.subarray(off, Math.min(b.data.length, off + this.xfer));
          await this.writeBlock(b.address + off, chunk);
          done += chunk.length; progress('Schreiben', done, total);
        }
      }
      done = 0;
      for (const b of image.blocks) {
        for (let off = 0; off < b.data.length; off += this.xfer) {
          const len = Math.min(this.xfer, b.data.length - off);
          const r = await this.readBlock(b.address + off, len);
          for (let k = 0; k < len; k++) {
            if (r[k] !== b.data[off + k]) throw new Error(`Prüfen fehlgeschlagen bei 0x${(b.address + off + k).toString(16)}`);
          }
          done += len; progress('Prüfen', done, total);
        }
      }
      return { sectors: sectors.length, bytes: total };
    }
  }
  FL.Dfu = Dfu;
  FL.DFU_FILTER = { vendorId: 0x0483, productId: 0xdf11 };
  FL.BF_FILTER = { usbVendorId: 0x0483, usbProductId: 0x5740 };

  if (typeof module !== 'undefined' && module.exports) module.exports = FL;
  else root.FL = FL;
})(typeof window !== 'undefined' ? window : globalThis);
