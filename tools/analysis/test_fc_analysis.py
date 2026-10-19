"""
Test der Auswertung mit synthetischen Flugdaten.

Ein Regelkreis wird als System 2. Ordnung (Eigenfrequenz wn, Daempfung zeta) simuliert.
Die geschaetzte Sprungantwort muss mit der wahren Sprungantwort uebereinstimmen.

    python -m pytest tools/analysis -q      oder      python tools/analysis/test_fc_analysis.py
"""
from pathlib import Path
import sys
import tempfile

import numpy as np
import pandas as pd
from scipy import signal

sys.path.insert(0, str(Path(__file__).parent))
import fc_analysis as fa  # noqa: E402

RATE = 1000.0


def closed_loop(wn: float, zeta: float):
    sys_c = signal.TransferFunction([wn ** 2], [1, 2 * zeta * wn, wn ** 2])
    return signal.cont2discrete((sys_c.num, sys_c.den), 1 / RATE, method="bilinear")


def stick_input(seconds: float, seed: int) -> np.ndarray:
    """Handsteuerung nachbilden: zufaellige Spruenge, leicht geglaettet, Pausen dazwischen."""
    rng = np.random.default_rng(seed)
    n = int(seconds * RATE)
    sp = np.zeros(n)
    i = 0
    while i < n:
        hold = int(rng.uniform(0.15, 0.6) * RATE)
        level = rng.choice([0.0, rng.uniform(-300, 300)])
        sp[i:i + hold] = level
        i += hold
    b, a = signal.butter(2, 30 / (RATE / 2))   # Daumen bewegt sich nicht unendlich schnell
    return signal.lfilter(b, a, sp)


def simulate(wn, zeta, seconds=60, noise=3.0, seed=1) -> pd.DataFrame:
    num, den, _ = closed_loop(wn, zeta)
    rng = np.random.default_rng(seed + 100)
    t = np.arange(int(seconds * RATE)) / RATE
    data = {"t": t, "throttle": np.full_like(t, 0.4)}
    for k, ax in enumerate(fa.AXES):
        sp = stick_input(seconds, seed + k)
        gy = signal.lfilter(np.squeeze(num), np.squeeze(den), sp) + rng.normal(0, noise, len(sp))
        data[f"sp_{ax}"] = sp
        data[f"gyro_{ax}"] = gy
    return pd.DataFrame(data)


def true_step(wn, zeta, n):
    num, den, _ = closed_loop(wn, zeta)
    return signal.lfilter(np.squeeze(num), np.squeeze(den), np.ones(n))


def test_step_response_matches_truth():
    wn, zeta = 2 * np.pi * 12, 0.5
    df = simulate(wn, zeta)
    res = fa.step_response(df, "roll")
    assert res is not None and res.n_windows > 50
    truth = true_step(wn, zeta, len(res.response))
    err = np.max(np.abs(res.response[:300] - truth[:300]))
    assert err < 0.08, f"max. Abweichung {err:.3f}"

    m = fa.step_metrics(res)
    m_true = fa.step_metrics(fa.StepResult(res.time, truth, 1))
    expected_os = 100 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta ** 2))   # 16.3 %
    assert abs(m_true.overshoot_pct - expected_os) < 1.0
    assert abs(m.overshoot_pct - m_true.overshoot_pct) < 2.5
    assert abs(m.rise_time_ms - m_true.rise_time_ms) < 3.0
    assert abs(m.settling_time_ms - m_true.settling_time_ms) < 15.0
    assert abs(m.steady_state_error_pct) < 1.5


def test_compare_detects_difference():
    a = fa.analyze(simulate(2 * np.pi * 12, 0.5, seed=1))
    same = fa.analyze(simulate(2 * np.pi * 12, 0.5, seed=7))
    slow = fa.analyze(simulate(2 * np.pi * 6, 0.3, seed=7))
    t_same = fa.compare(a, same)
    t_slow = fa.compare(a, slow)
    rise_same = t_same[(t_same.achse == "roll") & (t_same.kennzahl == "rise_time_ms")].iloc[0]
    rise_slow = t_slow[(t_slow.achse == "roll") & (t_slow.kennzahl == "rise_time_ms")].iloc[0]
    assert rise_same.vergleichbar
    assert not rise_slow.vergleichbar
    os_slow = t_slow[(t_slow.achse == "roll") & (t_slow.kennzahl == "overshoot_pct")].iloc[0]
    assert not os_slow.vergleichbar      # deutlich mehr Ueberschwingen darf nicht durchgehen


def test_loaders_roundtrip():
    df = simulate(2 * np.pi * 12, 0.5, seconds=5)
    with tempfile.TemporaryDirectory() as d:
        # Format der eigenen Firmware, mit Zeitstempel-Ueberlauf in der Mitte
        t_us = (np.arange(len(df)) * 1000 + (2 ** 32 - 2_000_000)) % (2 ** 32)
        b = pd.DataFrame({"t_us": t_us, "throttle": df.throttle, "flags": 1})
        for ax in fa.AXES:
            b[f"sp_{ax}"] = df[f"sp_{ax}"]
            b[f"gyro_{ax}"] = df[f"gyro_{ax}"]
        pb = Path(d) / "b.csv"
        b.to_csv(pb, index=False)
        lb = fa.load_buildb_csv(pb)
        assert np.allclose(np.diff(lb.t), 1e-3)

        # Format von blackbox_decode (Leerzeichen nach den Kommas wie im Original)
        a = pd.DataFrame({"loopIteration": np.arange(len(df)), "time (us)": np.arange(len(df)) * 1000})
        for i, ax in enumerate(fa.AXES):
            a[f"setpoint[{i}]"] = df[f"sp_{ax}"].round()
            a[f"gyroADC[{i}]"] = df[f"gyro_{ax}"].round()
        a["setpoint[3]"] = 400
        for m in range(4):
            a[f"motor[{m}]"] = 1000
        pa = Path(d) / "a.csv"
        pa.write_text(a.to_csv(index=False).replace(",", ", "))
        la = fa.load_betaflight_csv(pa)
        assert np.allclose(la.throttle, 0.4)
        assert np.allclose(np.diff(la.t), 1e-3)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
