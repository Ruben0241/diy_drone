"""
run_sim.py - Software-in-the-Loop: der echte Firmware-Kern fliegt im Physikmodell

    cmake -S tests -B build-tests && cmake --build build-tests     # baut libfc_core.so
    python tools/sim/run_sim.py --out sim_ergebnisse

Ablauf pro Szenario:
  1. Gyro-Kalibrierung (Drohne steht), Armen mit Schalter, Gas auf Schwebe-Gas
  2. Ein "Pilot" haelt die Hoehe (nur Gas) und gibt Stick-Spruenge auf Roll/Pitch/Yaw
  3. Aufzeichnung wie auf dem Flightcontroller (1 kHz), Auswertung mit tools/analysis

Ergebnis: Sprungantworten, Kennzahlen und Kenngroessen des Modells (Schwebe-Gas,
Schub-Gewichts-Verhaeltnis, Traegheitsmomente, Motorzeitkonstante).
"""
from __future__ import annotations

import argparse
import ctypes as C
import json
import math
from pathlib import Path
import sys

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "analysis"))
import quad_model as qm      # noqa: E402
import fc_analysis as fa     # noqa: E402

LOOP_HZ = 1000
SUB = 8                      # Physikschritte pro Regelzyklus (8 kHz)


class FC:
    """Firmware-Kern ueber ctypes"""

    def __init__(self, lib_path: Path | None = None):
        lib_path = lib_path or (ROOT / "build-tests" / "libfc_core.so")
        if not lib_path.exists():
            raise FileNotFoundError(f"{lib_path} fehlt: zuerst 'cmake --build build-tests'")
        self.lib = C.CDLL(str(lib_path))
        f3 = C.c_float * 3
        self._f3, self._f4, self._ch = f3, C.c_float * 4, C.c_ushort * 16
        self.lib.fc_step.argtypes = [f3, f3, self._ch, C.c_int, self._f4, f3, f3, f3]
        self.lib.fc_step.restype = C.c_int
        self.lib.fc_set.argtypes = [C.c_char_p, C.c_char_p]
        self.lib.fc_gyro_filtered.restype = C.c_float
        self.lib.fc_init()

    def set(self, name: str, value) -> None:
        if not self.lib.fc_set(name.encode(), str(value).encode()):
            raise ValueError(f"Einstellung {name}={value} ungueltig")

    def reset(self) -> None:
        self.lib.fc_reset()

    def step(self, gyro, acc, ch, rc_valid=True):
        m, sp, att, ps = self._f4(), self._f3(), self._f3(), self._f3()
        flags = self.lib.fc_step(self._f3(*gyro), self._f3(*acc), self._ch(*ch), int(rc_valid), m, sp, att, ps)
        gf = [self.lib.fc_gyro_filtered(i) for i in range(3)]
        return flags, np.array(m[:]), np.array(sp[:]), np.array(att[:]), np.array(gf), np.array(ps[:])


def stick_to_crsf(s: float) -> int:
    return int(round(992 + max(-1, min(1, s)) * 819.5))


def thr_to_crsf(t: float) -> int:
    return int(round(172 + max(0, min(1, t)) * 1639))


def step_pattern(t: float, axis: int, amp: float, t0: float = 3.0, period: float = 0.6) -> np.ndarray:
    """Stick-Spruenge wie Manoever M2-M4: +amp, 0, -amp, 0 ... (je period)"""
    s = np.zeros(3)
    if t >= t0:
        k = int((t - t0) / period) % 4
        s[axis] = [amp, 0.0, -amp, 0.0][k]
    return s


def fly(fc: FC, params: qm.DroneParams | None = None, seconds: float = 12.0, axis: int | None = 0,
        amp: float = 0.4, settings: dict | None = None, wind_gust: tuple | None = None, seed: int = 1):
    """Ein Flug. Rueckgabe: DataFrame im Log-Format von Build B + Ereignisse"""
    sim = qm.QuadSim(params, dt_phys=1 / (LOOP_HZ * SUB), seed=seed)
    for k, v in (settings or {}).items():
        fc.set(k, v)
    fc.reset()
    hover = sim.hover_command()
    idle = 0.055
    thr_hover = (hover - idle) / (1 - idle)          # Gasstellung, die Schwebe-Befehl ergibt
    rows, events = [], {"armed_at": None, "runaway": False, "disarmed_at": None, "crash": False}
    alt_i = 0.0
    n = int(seconds * LOOP_HZ)
    for k in range(n):
        t = k / LOOP_HZ
        # Phasen: 0-1.5 s kalibrieren (Boden, Arm aus), 1.5 s armen, ab 2 s fliegen
        ch = [992] * 16
        ch[5] = 172                                   # Rate-Mode
        arm = t >= 1.5
        ch[4] = 1811 if arm else 172
        on_ground = t < 2.0
        if on_ground:
            thr = 0.0
        else:
            # Pilot haelt die Hoehe: PI auf die Hoehe, um das Schwebe-Gas herum
            err = 1.0 - sim.pos[2]
            alt_i += err / LOOP_HZ
            thr = thr_hover + 0.08 * err - 0.06 * sim.vel[2] + 0.02 * alt_i
        ch[2] = thr_to_crsf(thr)
        sticks = step_pattern(t, axis, amp) if axis is not None else np.zeros(3)
        for i, c in enumerate((0, 1, 3)):            # AETR: Roll=0, Pitch=1, Yaw=3
            ch[c] = stick_to_crsf(sticks[i])
        if wind_gust and wind_gust[0] <= t < wind_gust[0] + wind_gust[1]:
            sim.ext_torque = np.array(wind_gust[2])
        else:
            sim.ext_torque = np.zeros(3)

        flags, motors, sp, att, gf, ps = fc.step(sim.gyro_reading(), sim.acc_reading(), ch)
        armed = bool(flags & 1)
        if armed and events["armed_at"] is None:
            events["armed_at"] = t
        if flags & 2:
            events["runaway"] = True
        if not armed and events["armed_at"] is not None and events["disarmed_at"] is None:
            events["disarmed_at"] = t
        sim.u = motors if armed else np.zeros(4)
        if on_ground:                                 # am Boden: keine Bewegung
            sim.Omega[:] = 0
        if np.linalg.norm(sim.w) > 150.0:            # > 8600 dps: Drohne unkontrolliert
            events["crash"] = True
            break
        for _ in range(SUB):
            sim.step()
            if on_ground:
                sim.pos[:] = 0; sim.vel[:] = 0; sim.w[:] = 0; sim.R = np.eye(3)
                sim.acc_world[:] = 0
        roll, pitch = sim.euler_deg()
        rows.append((int(t * 1e6), *sp, *gf, thr, *motors, int(flags & 1), roll, pitch,
                     *np.degrees(sim.w), sim.pos[2]))
    cols = ["t_us", "sp_roll", "sp_pitch", "sp_yaw", "gyro_roll", "gyro_pitch", "gyro_yaw", "throttle",
            "m1", "m2", "m3", "m4", "flags", "true_roll", "true_pitch", "true_wx", "true_wy", "true_wz", "alt"]
    return pd.DataFrame(rows, columns=cols), events


def model_summary(p: qm.DroneParams) -> dict:
    rot = qm.rotor_from_data()
    T_max = p.k_T * p.omega_max**2
    hover = math.sqrt(p.mass * qm.G / 4 / p.k_T)
    return {
        "masse_kg": round(p.mass, 4),
        "J_kgm2": [float(f"{x:.3e}") for x in p.J],
        "k_T_Ns2": p.k_T, "k_Q_Nms2": p.k_Q,
        "omega_max_rad_s": p.omega_max, "drehzahl_max_rpm": round(p.omega_max * 60 / (2 * math.pi)),
        "schub_max_pro_motor_g": round(T_max / qm.G * 1000),
        "schub_gewicht_verhaeltnis": round(4 * T_max / (p.mass * qm.G), 2),
        "schwebe_drehzahl_rpm": round(hover * 60 / (2 * math.pi)),
        "schwebe_befehl": round(hover / p.omega_max, 3),
        "motor_tau_s": round(qm.motor_time_constant(), 4),
        "rotor_aus_daten": {k: float(f"{v:.4g}") for k, v in rot.items()},
    }


def analyse(df: pd.DataFrame, t_from: float = 3.0) -> dict:
    d = df[df.t_us >= t_from * 1e6].copy()
    d["t"] = (d.t_us - d.t_us.iloc[0]) * 1e-6
    d = d[["t", "sp_roll", "sp_pitch", "sp_yaw", "gyro_roll", "gyro_pitch", "gyro_yaw", "throttle"]]
    return fa.analyze(d, band=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim_ergebnisse")
    ap.add_argument("--seconds", type=float, default=15.0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    p = qm.DroneParams()
    summary = model_summary(p)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    fc = FC()

    results = {}
    for ax, name in enumerate(("roll", "pitch", "yaw")):
        df, ev = fly(fc, p, seconds=a.seconds, axis=ax, amp=0.4)
        df.to_csv(out / f"sim_{name}.csv", index=False)
        res = analyse(df)
        results[name] = res[name]
        m = res[name]["step"]
        print(f"{name:5s}: Anstieg {m['rise_time_ms']:.1f} ms, Ueberschwingen {m['overshoot_pct']:.1f} %, "
              f"Ausregel {m['settling_time_ms']:.0f} ms, RMS {res[name]['rms_error_dps']:.1f} dps, "
              f"armed bis Ende: {ev['disarmed_at'] is None}")

    fa.plot_steps({"Simulation (Firmware-Kern)": {k: v for k, v in results.items()}},
                  out / "sim_sprungantwort.png")
    (out / "modell.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    (out / "kennzahlen.json").write_text(json.dumps(
        {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")} for k, v in results.items()}, indent=2))
    print(f"Ergebnisse in {out}/")


if __name__ == "__main__":
    main()
