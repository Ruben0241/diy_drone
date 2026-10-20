"""
aliasing_demo.py - Warum die Abtastrate des Gyros wichtig ist (Abtasttheorem)

Im Schwebeflug dreht jeder Rotor mit f_rot = Omega/(2 pi). Unwucht erzeugt Vibration bei
f_rot, ein Dreiblatt-Propeller zusaetzlich bei 3 f_rot (Blattfrequenz). Wird das Gyro mit
f_s = 1 kHz abgetastet, erscheinen Anteile ueber f_s/2 = 500 Hz an falscher Stelle:
    f_alias = | f - k * f_s |   (k so, dass 0 <= f_alias <= f_s/2)
(Shannon 1949). Das Skript simuliert das Gyrosignal mit 8 kHz, tastet es mit 1 kHz ab
(ohne und mit dem Anti-Alias-Filter des ICM-42688-P bei 258 Hz) und zeigt beide Spektren.

    python tools/sim/aliasing_demo.py --out sim_ergebnisse
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import numpy as np
from scipy import signal

sys.path.insert(0, str(Path(__file__).parent))
import quad_model as qm   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="sim_ergebnisse")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    p = qm.DroneParams(vibration_dps=5.0, gyro_noise_dps=0.05)
    sim = qm.QuadSim(p, dt_phys=1 / 8000)
    sim.Omega[:] = math.sqrt(p.mass * qm.G / 4 / p.k_T)      # Schwebedrehzahl
    sim.u[:] = sim.Omega / p.omega_max
    f_rot = sim.Omega[0] / (2 * math.pi)
    n = 8000 * 2
    x = np.empty(n)
    for i in range(n):
        sim.phase += sim.Omega * sim.dt
        x[i] = sim.gyro_reading()[0]

    fs_hi, fs = 8000, 1000
    # Anti-Alias-Filter des Sensors (2. Ordnung, 258 Hz) vor der Abtastung
    b, a_ = signal.butter(2, 258 / (fs_hi / 2))
    x_aaf = signal.lfilter(b, a_, x)
    raw_1k = x[:: fs_hi // fs]
    aaf_1k = x_aaf[:: fs_hi // fs]

    def spec(v, f):
        w = np.hanning(len(v))
        S = np.abs(np.fft.rfft(v * w)) / np.sum(w) * 2
        return np.fft.rfftfreq(len(v), 1 / f), S

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.6), sharey=True)
    f8, S8 = spec(x, fs_hi)
    ax[0].plot(f8, S8, lw=0.8)
    ax[0].set_xlim(0, 1500)
    ax[0].set_title("Gyro-Signal mit 8 kHz (\"wahr\")")
    for f, lab in ((f_rot, "f_rot"), (3 * f_rot, "3 f_rot")):
        ax[0].axvline(f, color="grey", ls="--", lw=0.8)
        ax[0].text(f, ax[0].get_ylim()[1] * 0.9 if ax[0].get_ylim()[1] else 1, lab, fontsize=8)
    ax[0].axvline(500, color="red", lw=0.8)
    ax[0].text(505, 0.5, "f_s/2 = 500 Hz", color="red", fontsize=8)
    f1, S1 = spec(raw_1k, fs)
    f2, S2 = spec(aaf_1k, fs)
    ax[1].plot(f1, S1, lw=0.8, label="ohne Anti-Alias-Filter")
    ax[1].plot(f2, S2, lw=0.8, label="mit Sensorfilter 258 Hz")
    alias = abs(3 * f_rot - fs)
    ax[1].axvline(alias, color="grey", ls="--", lw=0.8)
    ax[1].text(alias, 0.5, f"Alias von 3 f_rot: {alias:.0f} Hz", fontsize=8)
    ax[1].set_xlim(0, 500)
    ax[1].set_title("abgetastet mit 1 kHz")
    ax[1].legend(fontsize=8)
    for a2 in ax:
        a2.set_xlabel("Frequenz [Hz]")
        a2.grid(alpha=0.3)
    ax[0].set_ylabel("Amplitude [dps]")
    fig.tight_layout()
    fig.savefig(out / "aliasing.png", dpi=200)
    print(f"Rotorfrequenz {f_rot:.0f} Hz, Blattfrequenz {3 * f_rot:.0f} Hz -> Alias bei {alias:.0f} Hz")
    print(f"Spitze bei Alias ohne Filter: {S1[np.argmin(abs(f1 - alias))]:.2f} dps, "
          f"mit Filter: {S2[np.argmin(abs(f2 - alias))]:.2f} dps")


if __name__ == "__main__":
    main()
