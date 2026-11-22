/*
 * test_flasher.js - Flasher gegen ein simuliertes STM32-DFU-Geraet (DfuSe, wie AN3156)
 *   node tools/configurator/test/test_flasher.js
 */
'use strict';
const fs = require('fs');
const path = require('path');
const FL = require('../flasher.js');
const ROOT = path.resolve(__dirname, '../../..');
let checks = 0, failures = 0;
const ok = (c, m) => { checks++; if (!c) { failures++; console.log('  FEHLER: ' + m); } };

/* Simulierter ROM-Bootloader: 512 KB Flash, Sektoren wie F722, Zustaende dfuIDLE/dnBUSY/dnIDLE/ERROR */
class FakeDfu {
  constructor() {
    this.flash = new Uint8Array(512 * 1024).fill(0xAB);       /* "alte Firmware" */
    this.ptr = 0x08000000; this.state = 2; this.status = 0; this.pending = null; this.left = null;
    this.erased = []; this.configuration = null;
    this.layoutName = '@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg';
  }
  async open() {}
  async close() {}
  async selectConfiguration() {
    this.configuration = { interfaces: [{ interfaceNumber: 0, alternates: [
      { alternateSetting: 0, interfaceClass: 0xFE, interfaceSubclass: 1, interfaceName: this.layoutName },
      { alternateSetting: 1, interfaceClass: 0xFE, interfaceSubclass: 1, interfaceName: '@Option Bytes  /0x1FFF0000/01*32 e' }] }] };
  }
  async claimInterface() {} async releaseInterface() {} async selectAlternateInterface(i, a) { this.alt = a; }
  idx(a) { if (a < 0x08000000 || a >= 0x08080000) throw new Error('ausserhalb'); return a - 0x08000000; }
  async controlTransferOut(s, data) {
    if (s.request === 1) {                                   /* DNLOAD */
      if (this.state !== 2 && this.state !== 5) { this.state = 10; this.status = 0x0F; return { status: 'ok' }; }
      this.pending = { block: s.value, data: Uint8Array.from(data) }; this.state = 3; return { status: 'ok' };
    }
    if (s.request === 4) { this.state = 2; this.status = 0; return { status: 'ok' }; }   /* CLRSTATUS */
    if (s.request === 6) { this.state = 2; return { status: 'ok' }; }                  /* ABORT */
    return { status: 'stall' };
  }
  async controlTransferIn(s, len) {
    if (s.request === 3) {                                   /* GETSTATUS: fuehrt das Ausstehende aus */
      if (this.pending) {
        const p = this.pending; this.pending = null;
        try {
          if (p.block === 0 && p.data.length === 0) { this.left = this.ptr; this.state = 8; }
          else if (p.block === 0 && p.data[0] === 0x21) { this.ptr = p.data[1] | (p.data[2] << 8) | (p.data[3] << 16) | (p.data[4] << 24) >>> 0; this.ptr >>>= 0; this.state = 5; }
          else if (p.block === 0 && p.data[0] === 0x41) {
            const a = (p.data[1] | (p.data[2] << 8) | (p.data[3] << 16) | (p.data[4] << 24)) >>> 0;
            const sec = FL.F722_SECTORS.find(x => a >= x.start && a < x.start + x.size);
            this.flash.fill(0xFF, this.idx(sec.start), this.idx(sec.start) + sec.size); this.erased.push(sec.start); this.state = 5;
          } else if (p.block >= 2) {
            const a = this.ptr + (p.block - 2) * 2048;
            for (let i = 0; i < p.data.length; i++) {
              if (this.flash[this.idx(a + i)] !== 0xFF) throw new Error('nicht geloescht');
              this.flash[this.idx(a + i)] = p.data[i];
            }
            this.state = 5;
          }
        } catch (e) { this.state = 10; this.status = 0x0A; }
        return { status: 'ok', data: new DataView(Uint8Array.from([this.status, 2, 0, 0, this.state === 5 ? 4 : this.state, 0]).buffer) };
      }
      return { status: 'ok', data: new DataView(Uint8Array.from([this.status, 0, 0, 0, this.state, 0]).buffer) };
    }
    if (s.request === 2) {                                   /* UPLOAD */
      if (this.state !== 2 && this.state !== 9) return { status: 'stall' };
      const a = this.ptr + (s.value - 2) * 2048;
      const d = this.flash.slice(this.idx(a), this.idx(a) + len);
      this.state = 9;
      return { status: 'ok', data: new DataView(d.buffer) };
    }
    return { status: 'stall' };
  }
}
/* Zustand 4 (dnBUSY) einmal zurueckgeben und danach dnIDLE, wie das echte Geraet */
const origIn = FakeDfu.prototype.controlTransferIn;
FakeDfu.prototype.controlTransferIn = async function (s, len) {
  if (s.request === 3 && this.busyOnce) { this.busyOnce = false; return { status: 'ok', data: new DataView(Uint8Array.from([0, 1, 0, 0, 5, 0]).buffer) }; }
  const r = await origIn.call(this, s, len);
  if (s.request === 3) { const b = new Uint8Array(r.data.buffer); if (b[4] === 4) this.busyOnce = true; }
  return r;
};

(async () => {
  console.log('HEX lesen');
  const hexPath = path.join(ROOT, 'build/firmware.hex'), binPath = path.join(ROOT, 'build/firmware.bin');
  const img = FL.parseHex(fs.readFileSync(hexPath, 'utf8'));
  const bin = fs.readFileSync(binPath);
  ok(img.blocks[0].address === 0x08000000, 'beginnt bei 0x08000000');
  const sameAsBin = blocks => blocks.every(b => b.data.every((v, i) => v === bin[b.address - 0x08000000 + i]));
  ok(sameAsBin(img.blocks), 'HEX == BIN (Bloecke: ' + img.blocks.map(b => b.data.length).join('+') + ')');
  ok(img.start !== null, 'Startadresse');
  let threw = false; try { FL.parseHex(':0100000000FE\n'); } catch (e) { threw = true; } ok(threw, 'Pruefsumme falsch erkannt');
  threw = false; try { FL.parseHex(':020000040800F2\n'); } catch (e) { threw = true; } ok(threw, 'fehlendes Ende erkannt');

  console.log('Sektoren');
  const lay = FL.parseDfuseLayout('@Internal Flash  /0x08000000/04*016Kg,01*064Kg,03*128Kg');
  ok(lay.length === 8 && lay[4].start === 0x08010000 && lay[4].size === 65536 && lay[7].start === 0x08060000, 'DfuSe-Layout');
  const need = FL.sectorsFor(img.blocks, lay);
  ok(need.length === (img.bytes > 65536 ? 5 : 4), 'nur benoetigte Sektoren: ' + need.length);

  console.log('Flashen gegen simuliertes Geraet');
  const dev = new FakeDfu();
  const dfu = new FL.Dfu(dev);
  await dfu.open();
  ok(dfu.alt === 0 && dfu.layout.length === 8, 'Internal Flash gewaehlt');
  const phases = new Set();
  const r = await dfu.flash(img, ph => phases.add(ph));
  ok(r.bytes === img.bytes, 'Bytes');
  ok(['Löschen', 'Schreiben', 'Prüfen'].every(p => phases.has(p)), 'alle Phasen');
  ok(img.blocks.every(b => b.data.every((v, i) => dev.flash[b.address - 0x08000000 + i] === v)), 'Flash-Inhalt == Firmware');
  ok(dev.flash[0x08020000 - 0x08000000] === 0xAB, 'Sektoren dahinter unberuehrt');
  await dfu.leave(0x08000000);
  ok(dev.left === 0x08000000, 'Start bei 0x08000000');

  console.log('Pruefen erkennt Fehler');
  const dev2 = new FakeDfu(); const d2 = new FL.Dfu(dev2); await d2.open();
  const origWrite = d2.writeBlock.bind(d2);
  d2.writeBlock = async (a, data) => { const c = Uint8Array.from(data); if (a === img.blocks[1].address + 4096) c[10] ^= 1; return origWrite(a, c); };
  threw = false; try { await d2.flash(img); } catch (e) { threw = /Prüfen/.test(e.message); } ok(threw, 'Verify-Fehler erkannt');

  console.log('Betaflight-CLI');
  const ver = FL.parseBfVersion('# version\n# Betaflight / STM32F7X2 (S7X2) 4.5.1 Jul 16 2024 / 13:44:31 (77d01ba3b) MSP API: 1.46\n# config rev: 1a2b3c\n# board: manufacturer_id: GEPR, board_name: GEPRCF722\n');
  ok(ver && ver.mcu === 'STM32F7X2' && ver.version === '4.5.1' && ver.board === 'GEPRCF722', 'version: ' + JSON.stringify(ver));
  /* simulierte serielle Verbindung */
  const script = { '#': '\r\nEntering CLI Mode, type \'exit\' to return, or \'help\'\r\n# ', 'diff all\r\n': 'diff all\r\n\r\n# version\r\n# Betaflight / STM32F7X2 (S7X2) 4.5.1\r\n\r\nset motor_pwm_protocol = DSHOT300\r\nserial 1 64 115200 57600 0 115200\r\n\r\n# ' };
  let buf = '';
  const io = { write: async t => { buf += script[t] || ''; }, read: async ms => { const c = buf; buf = ''; if (!c) await new Promise(r => setTimeout(r, Math.min(ms, 20))); return c; } };
  const e = await FL.bfEnterCli(io); ok(/CLI/.test(e), 'CLI betreten');
  const d = await FL.bfCollect(io, 'diff all', { idleMs: 300 });
  ok(/serial 1 64/.test(d) && /set motor_pwm_protocol/.test(d), 'diff all mitgeschrieben');

  console.log(`\n${checks} Pruefungen, ${failures} Fehler`);
  process.exit(failures ? 1 : 0);
})().catch(e => { console.log('ABBRUCH', e); process.exit(1); });
