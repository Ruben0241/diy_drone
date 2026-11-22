/*
 * mock.js - Demo-Flightcontroller im Browser (ohne Hardware)
 *
 * Antwortet auf dieselben Befehle wie die Firmware (src/app/usb_cli.c) mit denselben
 * Zeilenformaten. Damit laesst sich der Configurator ohne Drohne ausprobieren und testen.
 * Die Parameterliste stammt aus der Firmware (build-tests/params_sample.txt, siehe
 * test/test_protocol.js, der prueft, dass beide gleich sind).
 */
(function (root) {
  'use strict';
  const FC = (typeof module !== 'undefined' && module.exports) ? require('./protocol.js') : root.FC;

  /* [name, min, max, nachkommastellen, standard] */
  const PARAMS = [["p_roll",0,250,0,45],["i_roll",0,250,0,80],["d_roll",0,250,0,30],["f_roll",0,2000,0,0],["p_pitch",0,250,0,47],["i_pitch",0,250,0,84],["d_pitch",0,250,0,34],["f_pitch",0,2000,0,0],["p_yaw",0,250,0,45],["i_yaw",0,250,0,80],["d_yaw",0,250,0,0],["f_yaw",0,2000,0,0],["iterm_limit",0,1000,0,400],["pidsum_limit",100,1000,0,500],["pidsum_limit_pitch",100,1000,0,500],["pidsum_limit_yaw",100,1000,0,400],["iterm_relax",0,2,0,0],["iterm_relax_cutoff",1,50,0,15],["iterm_windup",20,100,0,80],["iterm_mix_windup",0.3,1,2,0.85],["yaw_lowpass_hz",0,500,0,100],["tpa_rate",0,1,2,0],["tpa_breakpoint",0,1,2,0.35],["gyro_lpf1_type",0,2,0,1],["gyro_lpf1_hz",0,500,0,150],["gyro_lpf2_type",0,2,0,0],["gyro_lpf2_hz",0,500,0,0],["gyro_notch_hz",0,500,0,0],["gyro_notch_cutoff",0,500,0,0],["dterm_lpf1_type",0,2,0,1],["dterm_lpf1_hz",0,500,0,75],["dterm_lpf2_type",0,2,0,1],["dterm_lpf2_hz",0,500,0,150],["acc_lpf_hz",0,200,0,25],["rc_smoothing_hz",0,200,0,40],["ff_smoothing_hz",0,200,0,30],["roll_center",0,2000,0,400],["roll_max",0,2000,0,400],["roll_expo",0,1,2,0],["pitch_center",0,2000,0,400],["pitch_max",0,2000,0,400],["pitch_expo",0,1,2,0],["yaw_center",0,2000,0,400],["yaw_max",0,2000,0,400],["yaw_expo",0,1,2,0],["deadband",0,0.2,3,0.01],["angle_limit",10,90,0,60],["angle_p",0,20,2,5],["imu_kp",0,10,3,0.25],["imu_ki",0,1,3,0],["motor_idle",0,0.2,3,0.055],["arm_throttle_max",0,0.3,2,0.05],["arm_angle_max",5,180,0,25],["low_throttle",0,0.3,2,0.05],["airmode_start",0,1,2,0.25],["runaway_prevention",0,1,0,1],["gyro_sign_roll",-1,1,0,1],["gyro_sign_pitch",-1,1,0,1],["gyro_sign_yaw",-1,1,0,1],["stick_sign_roll",-1,1,0,1],["stick_sign_pitch",-1,1,0,1],["stick_sign_yaw",-1,1,0,-1],["yaw_motors_reversed",0,1,0,0],["acc_bias_x",-0.5,0.5,4,0],["acc_bias_y",-0.5,0.5,4,0],["acc_bias_z",-0.5,0.5,4,0],["motor1_out",1,4,0,1],["motor2_out",1,4,0,2],["motor3_out",1,4,0,3],["motor4_out",1,4,0,4],["vbat_scale",10,255,0,110],["vbat_warn_cell",3,4,2,3.5],["rx_uart",0,6,0,0],["crsf_telemetry",0,1,0,1]];

  const TLM_FIELDS = 't_ms,armed,failsafe,mode,airmode,calibrated,motor_test,arm_block,att_r,att_p,att_y,gyro_r,gyro_p,gyro_y,acc_x,acc_y,acc_z,sp_r,sp_p,sp_y,thr,m1,m2,m3,m4,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,lq,vbat_mv,loop_avg_x10,loop_max_us,rx_uart,rx_frames,rx_crc_err,out1,out2,out3,out4';

  class MockFC {
    constructor() {
      this.onData = null;
      this.t0 = Date.now();
      this.values = {};
      this.saved = {};
      for (const p of PARAMS) { this.values[p[0]] = p[4]; this.saved[p[0]] = p[4]; }
      this.streamMs = 0; this.nextStream = 0;
      this.motorTest = [0, 0, 0, 0]; this.motorTestUntil = 0;
      this.calUntil = Date.now() + 1200;
      this.accCalUntil = 0;
      this.armed = false; this.armSeenOff = false;
      this.radioOn = true;            /* Demo: Fernsteuerung an */
      this.handMotion = true;         /* Demo: Drohne wird langsam gekippt */
      this.flash = this._makeLog();
      this.frames = 0;
      this.att = [0, 0, 0]; this.gyro = [0, 0, 0];
      this.timer = null;
    }
    open() {
      this.timer = setInterval(() => this._tick(), 10);
      return Promise.resolve();
    }
    close() {
      clearInterval(this.timer); this.timer = null;
      this.motorTest = [0, 0, 0, 0]; this.streamMs = 0;
      return Promise.resolve();
    }
    write(text) {
      /* wie die Firmware: Befehle der Reihe nach, ein langer Befehl (dump, acccal) blockiert */
      this.queue = this.queue || Promise.resolve();
      for (const line of String(text).split(/\r?\n/)) {
        if (line.trim()) this.queue = this.queue.then(() => new Promise(r => setTimeout(r, 1))).then(() => this._handle(line.trim()));
      }
      return Promise.resolve();
    }
    _emit(s) {
      if (!this.onData) return;
      const bytes = typeof s === 'string' ? new TextEncoder().encode(s) : s;
      this.onData(bytes);
    }
    _say(s) { this._emit(s + '\r\n'); }
    _ms() { return Date.now() - this.t0; }

    _channels() {
      const t = this._ms() / 1000;
      if (!this.radioOn) return [992, 992, 172, 992, 172, 172, 172, 172];
      const s = v => Math.round(992 + v * 819);
      const arm = this.demoArm ? 1811 : 172;
      return [s(0.6 * Math.sin(t * 0.9)), s(0.5 * Math.sin(t * 0.7 + 1)), 172 + Math.round(60 * (1 + Math.sin(t * 0.3))),
        s(0.4 * Math.sin(t * 0.5 + 2)), arm, 172, 992, 992];
    }
    _armBlock(ch) {
      let b = 0;
      const now = Date.now();
      if (!this.radioOn) b |= 1;
      if (!this.armSeenOff) b |= 2;
      if (now < this.calUntil) b |= 4;
      if (Math.abs(this.att[0]) > this.values.arm_angle_max || Math.abs(this.att[1]) > this.values.arm_angle_max) b |= 8;
      if (FC.crsfToUnipolar(ch[2]) >= this.values.arm_throttle_max) b |= 16;
      if (now < this.accCalUntil) b |= 64;
      if (now < this.motorTestUntil) b |= 128;
      const map = [1, 2, 3, 4].map(i => this.values['motor' + i + '_out']);
      if (new Set(map).size !== 4) b |= 256;
      return b;
    }
    _tick() {
      const t = this._ms() / 1000;
      const prev = this._prev || this.att.slice();
      if (this.handMotion) {
        this.att = [18 * Math.sin(t * 0.8), 12 * Math.sin(t * 0.6 + 0.5), (40 * t) % 360 - 180];
      }
      for (let a = 0; a < 3; a++) {
        let d = this.att[a] - prev[a];
        if (d > 180) d -= 360; if (d < -180) d += 360;
        this.gyro[a] = d / 0.01 * this.values['gyro_sign_' + ['roll', 'pitch', 'yaw'][a]];
      }
      this._prev = this.att.slice();
      if (this.radioOn) this.frames += 2;
      const ch = this._channels();
      if (FC.crsfToUnipolar(ch[4]) < 0.5) this.armSeenOff = true;
      if (Date.now() >= this.motorTestUntil) this.motorTest = [0, 0, 0, 0];
      if (this.streamMs && Date.now() >= this.nextStream) {
        this.nextStream = Date.now() + this.streamMs;
        this._say(this._tlmLine(ch));
      }
    }
    _tlmLine(ch) {
      const v = this.values;
      const stick = [FC.crsfToBipolar(ch[0]), FC.crsfToBipolar(ch[1]), FC.crsfToBipolar(ch[3])];
      const sp = ['roll', 'pitch', 'yaw'].map((ax, i) =>
        FC.actualRate(stick[i] * v['stick_sign_' + ax], v[ax + '_center'], v[ax + '_max'], v[ax + '_expo']));
      const testing = Date.now() < this.motorTestUntil;
      const motor = this.motorTest.map(p => (testing && p > 0 ? Math.round(48 + p / 100 * 1999) : 0));
      const map = [1, 2, 3, 4].map(i => v['motor' + i + '_out']);
      const out = [0, 0, 0, 0];
      if (new Set(map).size === 4) map.forEach((o, i) => { out[o - 1] = motor[i]; });
      const r = Math.PI / 180;
      const acc = [-Math.sin(this.att[1] * r) * 0 + Math.sin(this.att[0] * r) * 0, 0, 1];
      acc[0] = -Math.sin(this.att[1] * r); acc[1] = Math.sin(this.att[0] * r) * Math.cos(this.att[1] * r);
      acc[2] = Math.cos(this.att[0] * r) * Math.cos(this.att[1] * r);
      const vbat = 16.4 - 0.0005 * this._ms() / 1000;
      const f = [this._ms(), 0, this.radioOn ? 0 : 1, 0, 0, Date.now() >= this.calUntil ? 1 : 0, testing ? 1 : 0, this._armBlock(ch),
        ...this.att.map(x => Math.round(x * 10)), ...this.gyro.map(x => Math.round(x * 10)),
        ...acc.map(x => Math.round(x * 1000)), ...sp.map(x => Math.round(x * 10)),
        Math.round(FC.crsfToUnipolar(ch[2]) * 1000), ...motor, ...ch, this.radioOn ? 100 : 0,
        Math.round(vbat * 1000), 412, 61, this.radioOn ? (v.rx_uart || 2) : 0, this.frames, 0, ...out];
      return '$T,' + f.join(',');
    }
    _fmt(v, dec) { return Number(v).toFixed(dec); }
    _param(name) { return PARAMS.find(p => p[0] === name); }

    _makeLog() {
      /* zwei kurze Demo-Fluege (3 s und 2 s) mit einer Roll-Sprungantwort */
      const parts = [];
      for (const [dur, seed] of [[3000, 1], [2000, 2]]) {
        parts.push(FC.encodeHeader({ git_hash: 'd3m0d3m0', pid: { roll: [45, 80, 30], pitch: [47, 84, 34], yaw: [45, 80, 0] } }));
        let g = 0;
        for (let i = 0; i < dur; i++) {
          const sp = (i % 1000) < 500 ? 200 * seed : -200 * seed;
          g += (sp - g) * 0.03;
          parts.push(FC.encodeData({ t_us: 5e6 + i * 1000, flags: 1, sp: [sp, 0, 0], gyro: [g + 3 * Math.sin(i * 0.9), 0, 0],
            motor: [800, 820, 790, 810], throttle: 0.35, vbat: 15.6 - i * 0.0001, lq: 100 }));
        }
        parts.push(FC.encodePad());
      }
      const total = parts.reduce((n, p) => n + p.length, 0);
      const out = new Uint8Array(total);
      let o = 0;
      for (const p of parts) { out.set(p, o); o += p.length; }
      return out;
    }

    _wait(ms) { return new Promise(r => setTimeout(r, ms)); }

    async _handle(line) {
      const [cmd, ...args] = line.split(/\s+/);
      const v = this.values;
      switch (cmd) {
        case 'help': this._say('Befehle: (Demo) help, info, status, version, params, stream, get, set, save, diff, defaults, motor, motors, acccal, gyrocal, dump, erase, reboot, dfu, echo'); break;
        case 'version': this._say('$V,fw=d3m0d3m0,proto=1,board=DEMO,params=' + PARAMS.length); break;
        case 'echo': this._say(args.join(' ')); break;
        case 'params':
          for (const p of PARAMS) {
            const d2 = Math.max(2, p[3]);
            this._say('$P,' + [p[0], this._fmt(v[p[0]], p[3]), this._fmt(p[1], d2), this._fmt(p[2], d2), p[3], this._fmt(p[4], p[3])].join(','));
          }
          break;
        case 'get':
          for (const p of PARAMS) if (!args[0] || p[0].startsWith(args[0])) this._say(p[0] + ' = ' + this._fmt(v[p[0]], p[3]));
          break;
        case 'set': {
          let name = args[0] || '', val = args[1];
          if (name.includes('=')) [name, val] = name.split('=');
          const p = this._param(name);
          if (!p) { this._say('ERR unbekannt: ' + name); break; }
          const x = Number(val);
          if (!Number.isFinite(x) || x < p[1] || x > p[2]) { this._say('ERR Bereich ' + this._fmt(p[1], p[3]) + ' .. ' + this._fmt(p[2], p[3])); break; }
          if (this.armed) { this._say('ERR armed'); break; }
          v[name] = x;
          this._say(name + ' = ' + this._fmt(x, p[3]));
          this._say('OK (dauerhaft erst mit save)');
          break;
        }
        case 'save': Object.assign(this.saved, v); this._say('OK gespeichert'); break;
        case 'defaults': for (const p of PARAMS) v[p[0]] = p[4]; this._say('OK Standardwerte aktiv (dauerhaft erst mit save)'); break;
        case 'diff':
          this._say('# Abweichungen vom Standard');
          for (const p of PARAMS) if (Math.abs(v[p[0]] - p[4]) > 1e-5) this._say('set ' + p[0] + ' = ' + this._fmt(v[p[0]], p[3]));
          break;
        case 'info': {
          const used = this.flash.length;
          this._say('fw d3m0d3m0 (Demo-Modus, keine Hardware)');
          this._say('clock HSE 8MHz');
          this._say('flash jedec=EF4018 ok=1 capacity=16711680 used=' + used + ' percent=' + Math.floor(used * 100 / 16711680) + ' full=0');
          this._say('log records=0 dropped=0 queue_drops=0');
          this._say('state armed=0 failsafe=' + (this.radioOn ? 0 : 1) + ' calibrated=1 gyro_ok=1 mode=RATE loop_overruns=0');
          this._say('rx uart=' + (v.rx_uart || 2) + ' frames=' + this.frames + ' crc_errors=0 lq=100');
          this._say('loop avg_us=41.2 max_us=61 period_us min=996 max=1004');
          this._say('config Standardwerte');
          this._say('airmode=0 runaway_triggered=0');
          this._say('vbat 16.40 cells=4');
          this._say('arm_block 0x' + this._armBlock(this._channels()).toString(16).padStart(4, '0').toUpperCase());
          break;
        }
        case 'stream': {
          const hz = Number(args[0]);
          if (!(hz >= 0 && hz <= 100)) { this._say('ERR Aufruf: stream <0-100>'); break; }
          if (hz < 0.5) { this.streamMs = 0; this._say('OK'); break; }
          this.streamMs = Math.max(10, Math.round(1000 / hz));
          this._say('$H,' + TLM_FIELDS);
          this.nextStream = Date.now();
          break;
        }
        case 'motor':
          if (args[0] === 'stop') { this.motorTest = [0, 0, 0, 0]; this.motorTestUntil = 0; this._say('OK'); break; }
          {
            const p = Number(args[1]); const ms = args[2] ? Number(args[2]) : 3000;
            if (!(p >= 0 && p <= 40)) { this._say('ERR Aufruf: motor <1-4|all> <0-40> [ms]  (ohne Propeller!)'); break; }
            this.motorTest = [1, 2, 3, 4].map(i => (args[0] === 'all' || Number(args[0]) === i ? p : 0));
            this.motorTestUntil = Date.now() + ms;
            this._say('OK (ohne Propeller!)');
          }
          break;
        case 'motors': {
          const p = args.slice(0, 4).map(Number); const ms = args[4] ? Number(args[4]) : 3000;
          if (p.length < 4 || p.some(x => !(x >= 0 && x <= 40))) { this._say('ERR Aufruf: motors <0-40> <0-40> <0-40> <0-40> [ms]  (ohne Propeller!)'); break; }
          this.motorTest = p; this.motorTestUntil = Date.now() + ms;
          this._say('OK');
          break;
        }
        case 'acccal':
          this._say('ACC-Kalibrierung: Drohne waagrecht und still halten ...');
          this.accCalUntil = Date.now() + 600;
          await this._wait(600);
          v.acc_bias_x = 0.0123; v.acc_bias_y = -0.0071; v.acc_bias_z = 0.0214;
          this._say('acc_bias x=0.0123 y=-0.0071 z=0.0214'); this._say('OK (dauerhaft erst mit save)');
          break;
        case 'gyrocal':
          this._say('Gyro-Kalibrierung: Drohne still halten ...');
          this.calUntil = Date.now() + 1100;
          await this._wait(1150);
          this._say('OK');
          break;
        case 'status': this._say('att r=0.0 p=0.0 y=0.0 (Demo)'); break;
        case 'dump': {
          const data = this.flash;
          this._say('DUMP ' + data.length);
          for (let pos = 0; pos < data.length; pos += 16384) {
            this._emit(data.slice(pos, Math.min(data.length, pos + 16384)));
            await this._wait(5);
          }
          this._emit('\r\n'); this._say('OK');
          break;
        }
        case 'erase':
          await this._wait(200); this._say('ERASE 50%');
          await this._wait(200); this.flash = new Uint8Array(0); this._say('ERASE 100%'); this._say('OK');
          break;
        case 'reboot': case 'dfu': this._say('OK ' + cmd); break;
        default: this._say('ERR unbekannt: ' + line);
      }
    }
  }

  MockFC.PARAMS = PARAMS;
  if (typeof module !== 'undefined' && module.exports) module.exports = MockFC;
  else root.MockFC = MockFC;
})(typeof window !== 'undefined' ? window : globalThis);
