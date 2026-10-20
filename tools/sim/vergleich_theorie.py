"""
vergleich_theorie.py - Simulation gegen das vereinfachte Modell "gedaempfter Schwinger"

Physik-Kapitel: Der Roll-Regelkreis mit P (und D) und Motorverzoegerung tau_m ist ein
Schwinger 2. Ordnung  tau_m J w'' + (J + K_D) w' + K_P w = K_P w_soll.
physik_rechnungen.py berechnet daraus Daempfung und Ueberschwingen; hier fliegt der
Firmware-Kern dieselben Faelle im vollstaendigen Modell (mit Filtern, Rauschen, I-Anteil).

    python tools/sim/vergleich_theorie.py --out physik
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
import quad_model as qm   # noqa: E402
import run_sim as rs      # noqa: E402

FAELLE = {
    "nur P": {"i_roll": 0, "d_roll": 0},
    "P + D": {"i_roll": 0},
    "P + D ohne Filter": {"i_roll": 0, "gyro_lpf1_hz": 0, "dterm_lpf1_hz": 0, "dterm_lpf2_hz": 0},
    "PID (Standard)": {},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="physik")
    ap.add_argument("--seconds", type=float, default=12.0)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    fc, p = rs.FC(), qm.DroneParams()
    res, curves = {}, {}
    for name, s in FAELLE.items():
        fc.lib.fc_init()                       # Standardwerte, dann Fall einstellen
        df, ev = rs.fly(fc, p, seconds=a.seconds, axis=0, amp=0.4, settings=s)
        an = rs.analyse(df)["roll"]
        m = an["step"]
        curves[name] = an["_curve"]
        res[name] = {k: round(float(m[k]), 1) for k in
                     ("rise_time_ms", "overshoot_pct", "settling_time_ms", "steady_state_error_pct")}
        print(f"{name:20s} {res[name]}")
    (out / "vergleich_theorie.json").write_text(json.dumps(res, indent=2, ensure_ascii=False))

    # Modell 2. Ordnung (Werte wie physik_rechnungen.py)
    import math
    import numpy as np
    from scipy import signal
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    J = p.J[0]
    Oh = math.sqrt(p.mass * qm.G / 4 / p.k_T)
    dtau = 4 * p.motor_dy * 2 * p.k_T * Oh * p.omega_max
    kp = 45 * 0.032029 / 1000 * dtau * 180 / math.pi
    kd = 30 * 0.000529 / 1000 * dtau * 180 / math.pi
    t = np.linspace(0, 0.3, 3001)
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6), sharey=True)
    for i, (name, kd_) in enumerate((("nur P", 0.0), ("P + D", kd))):
        _, y = signal.step(([kp], [p.tau_motor * J, J + kd_, kp]), T=t)
        ax[i].plot(t * 1e3, y, "--", label="Modell 2. Ordnung (Formel)")
        c = curves[name]
        if c is not None:
            ax[i].plot(c.time * 1e3, c.response, label="Simulation (Firmware-Kern)")
        ax[i].axhline(1, color="grey", lw=0.8, ls=":")
        ax[i].set_title(f"Roll, {name}")
        ax[i].set_xlabel("Zeit [ms]")
        ax[i].set_xlim(0, 300)
        ax[i].grid(alpha=0.3)
    ax[0].set_ylabel("Sprungantwort [-]")
    ax[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "theorie_vs_simulation.png", dpi=200)


if __name__ == "__main__":
    main()
