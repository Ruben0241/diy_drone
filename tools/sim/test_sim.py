"""
Tests mit der Physik-Simulation (Software-in-the-Loop). Benoetigt build-tests/libfc_core.so.

    python tools/sim/test_sim.py
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
import quad_model as qm   # noqa: E402
import run_sim as rs      # noqa: E402

FC = rs.FC()
P = qm.DroneParams()


def test_model_plausible():
    s = rs.model_summary(P)
    assert 0.35 < s["masse_kg"] < 0.5
    assert 3 < s["schub_gewicht_verhaeltnis"] < 15          # typisch fuer FPV: 5-12
    assert 0.2 < s["schwebe_befehl"] < 0.5
    assert 0.01 < s["motor_tau_s"] < 0.05
    J = P.J
    assert J[2] > J[1] > J[0]                                 # Gieren > Nicken > Rollen (gestreckter Frame)


def test_hover_stable():
    df, ev = rs.fly(FC, P, seconds=6, axis=None)
    d = df[df.t_us >= 3e6]
    assert not ev["crash"] and ev["disarmed_at"] is None
    assert d.true_roll.abs().max() < 2 and d.true_pitch.abs().max() < 2


def test_roll_step_response():
    df, ev = rs.fly(FC, P, seconds=8, axis=0, amp=0.4)
    assert not ev["crash"] and ev["disarmed_at"] is None
    m = rs.analyse(df)["roll"]["step"]
    assert m["rise_time_ms"] < 60
    assert m["overshoot_pct"] < 50
    assert abs(m["steady_state_error_pct"]) < 5


def test_yaw_step_response():
    df, ev = rs.fly(FC, P, seconds=8, axis=2, amp=0.4)
    assert not ev["crash"] and ev["disarmed_at"] is None
    m = rs.analyse(df)["yaw"]["step"]
    assert m["overshoot_pct"] < 30          # ohne Rotor-Reaktionsmoment im Modell: ca. 70 %


def test_rotor_inertia_estimate():
    assert 1.5e-6 < qm.rotor_inertia() < 3e-6 and abs(qm.rotor_inertia() - P.J_rotor) / P.J_rotor < 0.3


def test_gust_rejected():
    df, ev = rs.fly(FC, P, seconds=6, axis=None, wind_gust=(4.0, 0.2, (0.05, 0, 0)))
    assert df[df.t_us >= 3.9e6].true_roll.abs().max() < 10
    assert df[df.t_us >= 5.2e6].true_roll.abs().max() < 1


def test_runaway_protection_disarms_on_wrong_gyro_sign():
    df, ev = rs.fly(FC, P, seconds=4, axis=None, settings={"gyro_sign_roll": "-1"})
    FC.set("gyro_sign_roll", "1")
    assert ev["runaway"] and ev["disarmed_at"] is not None and ev["disarmed_at"] < 2.5


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
