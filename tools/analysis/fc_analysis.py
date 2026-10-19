"""
fc_analysis.py - Auswertung der Flugdaten von Build A (Betaflight) und Build B (eigene Firmware)

Begleitarbeit Kap. 3.4.5 / Kap. 6:
  * Sprungantwort pro Achse, geschaetzt per Wiener-Dekonvolution aus Soll- und Ist-Drehrate
    (gleiches Prinzip wie PID-Analyzer von Plasmatree; hier Fenster von 2 s, Antwort von 0.5 s,
    ueberlappende Fenster, Mittelung ueber gut angeregte Fenster)
  * Kennzahlen: Anstiegszeit (10-90 %), Ueberschwingen, Ausregelzeit, bleibende Abweichung
  * RMS der Regelabweichung (Soll - Ist) pro Achse
  * Vergleich A/B mit dem vorab festgelegten Kriterium (Standard: +-20 %)

Eingabe:
  Build A: CSV aus blackbox_decode (Spalten "time (us)", "setpoint[0..2]", "gyroADC[0..2]",
           "rcCommand[3]" bzw. "setpoint[3]", "motor[0..3]")
  Build B: CSV der eigenen Firmware (Spalten t_us, sp_roll, sp_pitch, sp_yaw,
           gyro_roll, gyro_pitch, gyro_yaw, throttle, m1..m4, flags)

Aufruf:
  python fc_analysis.py --a flugA.csv --b flugB.csv --out ergebnisse/
"""
from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np
import pandas as pd

AXES = ("roll", "pitch", "yaw")


# --------------------------------------------------------------------------- Einlesen

def _clean_columns(df: pd.DataFrame) -> pd.DataFrame:
    df.columns = [re.sub(r"\s+", " ", str(c)).strip() for c in df.columns]
    return df


def load_betaflight_csv(path: str | Path) -> pd.DataFrame:
    """CSV aus blackbox_decode bzw. Blackbox Explorer -> einheitliches Format."""
    df = _clean_columns(pd.read_csv(path, skipinitialspace=True, low_memory=False))
    time_col = next(c for c in df.columns if c.startswith("time"))
    if "setpoint[0]" not in df.columns:
        raise ValueError(
            f"{path}: keine Spalten setpoint[0..2]. Das Log stammt vermutlich aus einer alten "
            "Betaflight-Version (< 4.0). Mit aktueller Betaflight-Version neu aufzeichnen.")
    out = pd.DataFrame({"t": (df[time_col].astype(float) - df[time_col].iloc[0]) * 1e-6})
    for i, ax in enumerate(AXES):
        out[f"sp_{ax}"] = df[f"setpoint[{i}]"].astype(float)
        out[f"gyro_{ax}"] = df[f"gyroADC[{i}]"].astype(float)
    # Gas: setpoint[3] (0..1000) in neueren Logs, sonst rcCommand[3] (1000..2000)
    if "setpoint[3]" in df.columns:
        out["throttle"] = df["setpoint[3]"].astype(float) / 1000.0
    else:
        out["throttle"] = (df["rcCommand[3]"].astype(float) - 1000.0) / 1000.0
    for m in range(4):
        col = f"motor[{m}]"
        if col in df.columns:
            out[f"m{m + 1}"] = df[col].astype(float)
    return out


def load_buildb_csv(path: str | Path) -> pd.DataFrame:
    """CSV der eigenen Firmware -> einheitliches Format."""
    df = _clean_columns(pd.read_csv(path))
    out = pd.DataFrame({"t": _unwrap_us(df["t_us"])})
    for ax in AXES:
        out[f"sp_{ax}"] = df[f"sp_{ax}"].astype(float)
        out[f"gyro_{ax}"] = df[f"gyro_{ax}"].astype(float)
    out["throttle"] = df["throttle"].astype(float)
    for m in range(1, 5):
        if f"m{m}" in df.columns:
            out[f"m{m}"] = df[f"m{m}"].astype(float)
    if "flags" in df.columns:
        out["armed"] = (df["flags"].astype(int) & 1) == 1
    return out


def _unwrap_us(t_us: pd.Series) -> np.ndarray:
    """32-Bit-Mikrosekunden (Ueberlauf nach ~71 min) -> Sekunden ab 0."""
    t = t_us.astype(np.int64).to_numpy()
    wraps = np.concatenate([[0], np.cumsum(np.diff(t) < 0)])
    t = t + wraps * (1 << 32)
    return (t - t[0]) * 1e-6


def detect_rate(df: pd.DataFrame) -> float:
    dt = np.median(np.diff(df["t"].to_numpy()))
    return 1.0 / dt


def resample(df: pd.DataFrame, rate_hz: float) -> pd.DataFrame:
    """Auf ein gleichmaessiges Zeitraster bringen (fuer FFT noetig)."""
    t = df["t"].to_numpy()
    tn = np.arange(t[0], t[-1], 1.0 / rate_hz)
    out = {"t": tn}
    for c in df.columns:
        if c == "t":
            continue
        y = df[c].to_numpy()
        if y.dtype == bool:
            out[c] = np.interp(tn, t, y.astype(float)) > 0.5
        else:
            out[c] = np.interp(tn, t, y.astype(float))
    return pd.DataFrame(out)


# --------------------------------------------------------------------------- Sprungantwort

@dataclass
class StepConfig:
    frame_s: float = 2.0          # Fensterlaenge (PID-Analyzer: 1 s; 2 s ergab im Test weniger Endwertfehler)
    resp_s: float = 0.5           # Laenge der Antwort
    superpos: int = 16            # Ueberlappung: Verschiebung = frame / superpos
    cutfreq_hz: float = 25.0      # darueber gilt das Signal als Rauschen
    min_input_dps: float = 20.0   # Fenster ohne genug Anregung werden ignoriert
    throttle_range: tuple = (0.05, 0.9)  # extreme Gasstellungen auslassen


def _wiener(inp: np.ndarray, out: np.ndarray, dt: float, cutfreq: float) -> np.ndarray:
    """Wiener-Dekonvolution je Zeile: Impulsantwort h mit out = inp * h."""
    n = inp.shape[1]
    nfft = int(2 ** np.ceil(np.log2(2 * n)))
    X = np.fft.rfft(inp, nfft, axis=1)
    Y = np.fft.rfft(out, nfft, axis=1)
    f = np.fft.rfftfreq(nfft, dt)
    # Signal-Rausch-Verhaeltnis: hoch unterhalb cutfreq, sanft abfallend darueber
    snr = 10.0 / (1.0 + (f / cutfreq) ** 4)
    H = Y * np.conj(X) / (np.abs(X) ** 2 + 1.0 / snr)
    return np.fft.irfft(H, nfft, axis=1)[:, :n]


@dataclass
class StepResult:
    time: np.ndarray       # s
    response: np.ndarray   # mittlere Sprungantwort (1.0 = Sollwert)
    n_windows: int


def step_response(df: pd.DataFrame, axis: str, cfg: StepConfig = StepConfig()) -> StepResult | None:
    rate = detect_rate(df)
    dt = 1.0 / rate
    sp = df[f"sp_{axis}"].to_numpy(dtype=float)
    gy = df[f"gyro_{axis}"].to_numpy(dtype=float)
    thr = df["throttle"].to_numpy(dtype=float)

    flen = int(round(cfg.frame_s * rate))
    rlen = int(round(cfg.resp_s * rate))
    shift = max(1, flen // cfg.superpos)
    if len(sp) < flen:
        return None

    starts = np.arange(0, len(sp) - flen, shift)
    win = np.hanning(flen)
    inp, outp = [], []
    for s in starts:
        seg_sp = sp[s:s + flen]
        seg_thr = thr[s:s + flen]
        if np.max(np.abs(seg_sp)) < cfg.min_input_dps:
            continue
        if seg_thr.min() < cfg.throttle_range[0] or seg_thr.max() > cfg.throttle_range[1]:
            continue
        inp.append(seg_sp * win)
        outp.append(gy[s:s + flen] * win)
    if not inp:
        return None

    h = _wiener(np.array(inp), np.array(outp), dt, cfg.cutfreq_hz)[:, :rlen]
    steps = np.cumsum(h, axis=1)
    # robuste Mittelung: Median ueber alle Fenster, deren Endwert plausibel ist
    final = steps[:, -rlen // 5:].mean(axis=1)
    good = (final > 0.5) & (final < 1.5)
    if good.sum() >= 3:
        steps = steps[good]
    resp = np.median(steps, axis=0)
    return StepResult(time=np.arange(rlen) * dt, response=resp, n_windows=len(steps))


# --------------------------------------------------------------------------- Kennzahlen

@dataclass
class StepMetrics:
    rise_time_ms: float
    overshoot_pct: float
    settling_time_ms: float
    steady_state_error_pct: float
    n_windows: int


def step_metrics(res: StepResult, band: float = 0.05) -> StepMetrics:
    """Kennzahlen nach Lehrbuch (Lunze 2020): Anstieg, Ueberschwingen und Ausregelzeit beziehen
    sich auf den Endwert y_inf; die bleibende Regelabweichung ist y_inf - 1 (Sollwert = 1)."""
    t, y = res.time, res.response
    tail = y[-max(1, len(y) // 5):].mean()
    final = tail

    def first_cross(level):
        idx = np.argmax(y >= level)
        if y[idx] < level:
            return np.nan
        if idx == 0:
            return t[0]
        # lineare Interpolation zwischen den Abtastwerten
        y0, y1 = y[idx - 1], y[idx]
        return t[idx - 1] + (level - y0) / (y1 - y0) * (t[idx] - t[idx - 1])

    rise = first_cross(0.9 * final) - first_cross(0.1 * final)
    overshoot = max(0.0, (y.max() - final) / final * 100.0)
    outside = np.where(np.abs(y - final) > band * final)[0]
    if len(outside) == 0:
        settling = 0.0
    elif outside[-1] == len(y) - 1:
        settling = np.nan  # verlaesst das Band bis zum Ende nicht dauerhaft
    else:
        settling = t[outside[-1] + 1]
    return StepMetrics(
        rise_time_ms=float(rise * 1e3),
        overshoot_pct=float(overshoot),
        settling_time_ms=float(settling * 1e3) if not np.isnan(settling) else float("nan"),
        steady_state_error_pct=float((tail - 1.0) * 100.0),
        n_windows=res.n_windows,
    )


def tracking_rms(df: pd.DataFrame, axis: str) -> float:
    e = df[f"sp_{axis}"].to_numpy() - df[f"gyro_{axis}"].to_numpy()
    return float(np.sqrt(np.mean(e ** 2)))


def analyze(df: pd.DataFrame, band: float = 0.05, cfg: StepConfig = StepConfig()) -> dict:
    out = {}
    for ax in AXES:
        res = step_response(df, ax, cfg)
        m = step_metrics(res, band) if res is not None else None
        out[ax] = {
            "step": asdict(m) if m else None,
            "rms_error_dps": tracking_rms(df, ax),
            "_curve": res,
        }
    return out


# --------------------------------------------------------------------------- Vergleich

def compare(a: dict, b: dict, tolerance: float = 0.20, sse_tolerance_pp: float = 5.0) -> pd.DataFrame:
    """Beurteilung pro Kennzahl:
      * relativ: |B - A| / |A| <= tolerance (Anstieg, Ueberschwingen, Ausregelzeit, RMS)
      * bleibende Abweichung liegt nahe 0, darum absolut: |B - A| <= sse_tolerance_pp Prozentpunkte
    """
    rows = []
    for ax in AXES:
        pairs = [("rms_error_dps", a[ax]["rms_error_dps"], b[ax]["rms_error_dps"])]
        if a[ax]["step"] and b[ax]["step"]:
            for k in ("rise_time_ms", "overshoot_pct", "settling_time_ms", "steady_state_error_pct"):
                pairs.append((k, a[ax]["step"][k], b[ax]["step"][k]))
        for name, va, vb in pairs:
            if va is None or vb is None or np.isnan(va) or np.isnan(vb):
                dev, ok = np.nan, None
            elif name == "steady_state_error_pct":
                dev = abs(vb - va)                      # Prozentpunkte
                ok = bool(dev <= sse_tolerance_pp)
            else:
                dev = abs(vb - va) / abs(va) if va != 0 else np.nan
                ok = bool(dev <= tolerance) if not np.isnan(dev) else None
            rows.append({"achse": ax, "kennzahl": name, "build_a": va, "build_b": vb,
                         "abweichung": dev, "vergleichbar": ok})
    return pd.DataFrame(rows)


def plot_steps(results: dict[str, dict], path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True)
    for i, ax in enumerate(AXES):
        for label, res in results.items():
            curve = res[ax]["_curve"]
            if curve is not None:
                axs[i].plot(curve.time * 1e3, curve.response, label=label)
        axs[i].axhline(1.0, color="grey", lw=0.8, ls="--")
        axs[i].set_title(ax.capitalize())
        axs[i].set_xlabel("Zeit [ms]")
        axs[i].grid(alpha=0.3)
    axs[0].set_ylabel("Sprungantwort [-]")
    axs[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def _strip(res: dict) -> dict:
    return {ax: {k: v for k, v in d.items() if not k.startswith("_")} for ax, d in res.items()}


def main() -> None:
    ap = argparse.ArgumentParser(description="Vergleich Build A (Betaflight) / Build B")
    ap.add_argument("--a", nargs="+", help="Betaflight-CSV (blackbox_decode)")
    ap.add_argument("--b", nargs="+", help="CSV der eigenen Firmware")
    ap.add_argument("--out", default="ergebnisse", help="Ausgabeordner")
    ap.add_argument("--band", type=float, default=0.05, help="Toleranzband Ausregelzeit (0.05 = +-5 %)")
    ap.add_argument("--tolerance", type=float, default=0.20, help="Kriterium 'vergleichbar' (0.20 = 20 %)")
    ap.add_argument("--rate", type=float, default=1000.0, help="Auswerterate [Hz]")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    if args.a:
        df = pd.concat([resample(load_betaflight_csv(p), args.rate) for p in args.a], ignore_index=True)
        results["Build A (Betaflight)"] = analyze(df, args.band)
    if args.b:
        df = pd.concat([resample(load_buildb_csv(p), args.rate) for p in args.b], ignore_index=True)
        results["Build B (eigene FC)"] = analyze(df, args.band)

    (out / "kennzahlen.json").write_text(
        json.dumps({k: _strip(v) for k, v in results.items()}, indent=2, ensure_ascii=False))
    plot_steps(results, out / "sprungantwort.png")

    if args.a and args.b:
        table = compare(results["Build A (Betaflight)"], results["Build B (eigene FC)"], args.tolerance)
        table.to_csv(out / "vergleich.csv", index=False)
        print(table.to_string(index=False))
    else:
        for name, res in results.items():
            print(name)
            print(json.dumps(_strip(res), indent=2, ensure_ascii=False))
    print(f"\nErgebnisse in {out}/")


if __name__ == "__main__":
    main()
