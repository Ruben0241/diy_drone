"""
physik_rechnungen.py - alle Zahlen des Physik-Kapitels der Begleitarbeit, nachvollziehbar

    python tools/sim/physik_rechnungen.py --out physik

Erzeugt physik/rechnungen.json und die Abbildungen schub_drehzahl.png, motorgleichung.png.
Jede Rechnung nennt ihre Quelle. Messwerte, die noch fehlen, sind als SCHAETZUNG markiert
(siehe quad_model.DroneParams).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import quad_model as qm   # noqa: E402

G = 9.81

# Herstellerdaten T-Motor (Ligpower) F2004 KV3000, Propeller T76, 4S
# https://www.ligpower.com/product/f2004-kv3000-fpv-motor.html
# Spalten: Gas %, Schub g, Spannung V, Strom A, Drehzahl U/min, Leistung W
T76 = np.array([
    [50, 214.40, 15.90, 3.99, 24197, 63.40],
    [55, 261.45, 15.87, 5.26, 26703, 83.43],
    [60, 308.73, 15.84, 6.57, 28691, 104.01],
    [65, 355.65, 15.82, 7.84, 30906, 123.94],
    [70, 391.56, 15.79, 9.00, 32212, 142.00],
    [75, 450.25, 15.76, 10.64, 34389, 167.70],
    [80, 504.09, 15.72, 12.38, 36039, 194.74],
    [85, 548.46, 15.68, 14.17, 37332, 222.26],
    [90, 587.94, 15.64, 15.97, 38846, 249.85],
    [95, 627.19, 15.60, 18.29, 40088, 285.37],
    [100, 767.72, 15.56, 22.61, 43870, 351.87],
])
KV_RPM_PER_V = 3000
R_MOTOR = 0.090      # Ohm (Datenblatt)
I0 = 0.88            # A Leerlaufstrom bei 10 V (Datenblatt)


def rho_isa(h_m: float) -> float:
    """Luftdichte der Standardatmosphaere bis 11 km (U.S. Standard Atmosphere 1976):
    rho = rho0 (1 - L h / T0)^(g M/(R L) - 1)"""
    T0, L, rho0 = 288.15, 0.0065, 1.225
    return rho0 * (1 - L * h_m / T0) ** (G * 0.0289644 / (8.31446 * L) - 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="physik")
    ap.add_argument("--hoehe", type=float, default=420.0, help="Hoehe des Testgelaendes in m")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    p = qm.DroneParams()
    R = {}

    # ---------- 1. Masse und Gewicht ----------
    m = p.mass
    W = m * G
    R["masse"] = dict(m_kg=m, gewicht_N=W, schub_pro_motor_schweben_N=W / 4,
                      anteile_g=dict(motoren=4 * p.m_motor * 1e3, propeller=4 * p.m_prop * 1e3,
                                     akku=p.m_battery * 1e3, rest_SCHAETZUNG=p.m_rest * 1e3))

    # ---------- 2. Luftdichte ----------
    rho = rho_isa(a.hoehe)
    R["luft"] = dict(hoehe_m=a.hoehe, rho_kg_m3=rho, rho_meereshoehe=1.225)

    # ---------- 3. Schub ~ Drehzahl^2 (Herstellerdaten) ----------
    T = T76[:, 1] / 1000 * G           # N
    n = T76[:, 4] / 60                 # 1/s
    w = 2 * math.pi * n                # rad/s
    kT_pts = T / w**2
    # Exponent aus log-log-Regression: T = c * n^k
    k_exp, logc = np.polyfit(np.log(n), np.log(T), 1)
    D76 = 0.076
    CT = T / (rho_isa(0) * n**2 * D76**4)                     # Brandt & Selig 2011
    P_el = T76[:, 5]
    R["schub_drehzahl"] = dict(
        k_T_mittel_Ns2=float(kT_pts[:-1].mean()), k_T_streuung_pct=float(kT_pts[:-1].std() / kT_pts[:-1].mean() * 100),
        k_T_100pct=float(kT_pts[-1]), exponent_k=float(k_exp),
        C_T_mittel=float(CT[:-1].mean()),
        bemerkung="100-%-Zeile weicht ab (Sprung von 627 g auf 768 g), deshalb ohne sie gemittelt")

    # ---------- 4. Strahltheorie (Impulssatz) ----------
    D = 0.090
    A = math.pi * (D / 2) ** 2
    T_h = W / 4
    v_h = math.sqrt(T_h / (2 * rho * A))
    P_ideal = T_h * v_h
    # Guete aus Herstellerdaten (T76): Gesamtwirkungsgrad = P_ideal / P_el (Rotor * Motor * ESC)
    A76 = math.pi * (D76 / 2) ** 2
    P_ideal76 = T**1.5 / np.sqrt(2 * rho_isa(0) * A76)
    eta_ges = P_ideal76 / P_el
    eta_lo = float(eta_ges[0])          # tiefe Last, am naechsten am Schwebeflug
    P_el_hover = P_ideal / eta_lo
    R["strahltheorie"] = dict(D_m=D, A_m2=A, T_schweben_N=T_h, v_induziert_m_s=v_h,
                              P_ideal_pro_motor_W=P_ideal, P_ideal_gesamt_W=4 * P_ideal,
                              gesamtwirkungsgrad_T76=[float(x) for x in eta_ges],
                              P_el_schweben_geschaetzt_gesamt_W=4 * P_el_hover,
                              schub_pro_leistung_g_W=float(T_h / G * 1000 / P_el_hover))

    # ---------- 5. Schwebezeit ----------
    E_Wh = 1.8 * 14.8                    # 1800 mAh, 4S, Nennspannung 3.7 V/Zelle
    E_nutz = 0.8 * E_Wh                  # nur bis 80 % entladen (Akkuschonung)
    P_elek = 2.0                         # SCHAETZUNG: FC, Empfaenger, ESC-Ruhestrom
    t_h = E_nutz / (4 * P_el_hover + P_elek) * 60
    R["schwebezeit"] = dict(E_Wh=E_Wh, E_nutzbar_Wh=E_nutz, P_gesamt_W=4 * P_el_hover + P_elek,
                            t_min=t_h, strom_A=(4 * P_el_hover + P_elek) / 14.8)

    # ---------- 6. Motor: Kv, Kt, Gegen-EMK, Zeitkonstante (Drela 2005) ----------
    Kv = KV_RPM_PER_V * 2 * math.pi / 60        # rad/s pro V
    Kt = 1 / Kv                                   # N m / A
    # Drehzahl aus der Motorgleichung Omega = Kv (U_eff - I R), U_eff = Gas * U (ESC-PWM, Annahme)
    U_eff = T76[:, 0] / 100 * T76[:, 2]
    rpm_model = KV_RPM_PER_V * (U_eff - T76[:, 3] * R_MOTOR)
    # Drehmoment aus Strom: Q = Kt (I - I0); mechanische Leistung = Q * Omega
    Q = Kt * (T76[:, 3] - I0)
    P_mech = Q * w
    eta_motor = P_mech / P_el
    J_r = qm.rotor_inertia()
    tau_m = qm.motor_time_constant(J_rotor=J_r)
    R["motor"] = dict(Kv_rad_s_V=Kv, Kt_Nm_A=Kt,
                      drehzahl_modell_rpm=[float(x) for x in rpm_model],
                      drehzahl_mess_rpm=[float(x) for x in T76[:, 4]],
                      abweichung_pct=[float(x) for x in (rpm_model / T76[:, 4] - 1) * 100],
                      Q_Nm=[float(x) for x in Q], wirkungsgrad_motor=[float(x) for x in eta_motor],
                      J_rotor_kgm2=J_r, tau_motor_s=tau_m,
                      leerlauf_4S_rpm=KV_RPM_PER_V * 16.8, leerlauf_6S_rpm=KV_RPM_PER_V * 25.2)

    # ---------- 7. Blattspitzengeschwindigkeit ----------
    c_schall = 331.3 * math.sqrt(1 + 15 / 273.15)
    def tip(rpm):
        return math.pi * D * rpm / 60
    hover_rpm = math.sqrt(T_h / p.k_T) * 60 / (2 * math.pi)
    R["blattspitze"] = dict(c_schall_15C=c_schall,
                            schweben=dict(rpm=hover_rpm, v=tip(hover_rpm), mach=tip(hover_rpm) / c_schall),
                            vollgas_modell=dict(rpm=p.omega_max * 60 / (2 * math.pi),
                                                v=tip(p.omega_max * 60 / (2 * math.pi))),
                            leerlauf_4S=dict(rpm=50400, v=tip(50400), mach=tip(50400) / c_schall),
                            leerlauf_6S=dict(rpm=75600, v=tip(75600), mach=tip(75600) / c_schall))

    # ---------- 8. Traegheitsmomente (Steiner) und Bifilarpendel ----------
    J = p.J
    L_faden, d_faden = 1.0, 0.20
    T_bif = [2 * math.pi * math.sqrt(4 * Ji * L_faden / (m * G * d_faden**2)) for Ji in J]
    R["traegheit"] = dict(J_kgm2=[float(x) for x in J],
                          bifilar_L_m=L_faden, bifilar_d_m=d_faden,
                          bifilar_periode_erwartet_s=T_bif,
                          fehler=dict(dT_s=0.02, perioden=10,
                                      rel_fehler_J_pct=float(2 * 0.02 / (10 * T_bif[2]) * 100 + 2 * 0.001 / d_faden * 100
                                                             + 0.002 / L_faden * 100)))

    # ---------- 9. Dynamik: maximale Winkelbeschleunigung ----------
    T_max = p.k_T * p.omega_max**2
    tau_roll_max = 2 * p.motor_dy * T_max            # zwei Motoren voll, zwei aus
    tau_pitch_max = 2 * p.motor_dx * T_max
    tau_yaw_drag = 2 * p.k_Q * p.omega_max**2
    Omega_dot = p.omega_max / p.tau_motor
    tau_yaw_reakt = 4 * J_r * Omega_dot              # zwei beschleunigen, zwei bremsen: gleiches Vorzeichen
    R["dynamik"] = dict(T_max_N=T_max, schub_gewicht=4 * T_max / W,
                        alpha_roll_max_rad_s2=tau_roll_max / J[0], alpha_pitch_max_rad_s2=tau_pitch_max / J[1],
                        alpha_yaw_luft_max_rad_s2=tau_yaw_drag / J[2],
                        tau_yaw_luft_Nm=tau_yaw_drag, tau_yaw_reaktion_kurz_Nm=tau_yaw_reakt,
                        zeit_bis_400dps_roll_ms=math.radians(400) / (tau_roll_max / J[0]) * 1000,
                        kreisel_L_rotor_Nms=J_r * math.sqrt(T_h / p.k_T))

    # ---------- 10. Regelkreis als gedaempfter Schwinger (Roll) ----------
    # PID-Ausgang (Betaflight-Skalierung) -> Motorbefehl (/1000) -> Drehzahl -> Schub -> Moment
    Omega_h = math.sqrt(T_h / p.k_T)
    dT_du = 2 * p.k_T * Omega_h * p.omega_max          # N pro Einheit Motorbefehl (linearisiert)
    dtau_du = 4 * p.motor_dy * dT_du                   # Roll-Mischer: alle 4 Motoren mit +-1
    kp = 45 * 0.032029 / 1000 * dtau_du * 180 / math.pi     # N m pro rad/s
    kd = 30 * 0.000529 / 1000 * dtau_du * 180 / math.pi     # N m pro rad/s^2
    tm = p.tau_motor
    def zeta(kd_):
        wn = math.sqrt(kp / (tm * J[0]))
        return wn, (J[0] + kd_) / (2 * math.sqrt(kp * tm * J[0]))
    wn, z_p = zeta(0.0)
    _, z_pd = zeta(kd)
    def os(z):
        return 100 * math.exp(-math.pi * z / math.sqrt(1 - z * z)) if z < 1 else 0.0
    # Anstiegszeit 10-90 % aus der exakten Sprungantwort des Modells 2. Ordnung
    from scipy import signal
    def rise(kd_):
        t, y = signal.step(([kp], [tm * J[0], J[0] + kd_, kp]), T=np.linspace(0, 0.3, 30001))
        y = y / y[-1]
        return 1000 * (t[np.argmax(y >= 0.9)] - t[np.argmax(y >= 0.1)])
    R["regelkreis_roll"] = dict(anstieg_nur_P_ms=rise(0.0), anstieg_PD_ms=rise(kd),
                                dtau_du_Nm=dtau_du, K_P_Nm_s=kp, K_D_Nm_s2=kd, tau_motor_s=tm,
                                omega_n_rad_s=wn, f_n_Hz=wn / (2 * math.pi),
                                zeta_nur_P=z_p, ueberschwingen_nur_P_pct=os(z_p),
                                zeta_PD=z_pd, ueberschwingen_PD_pct=os(z_pd))

    # ---------- 11. Abtastung ----------
    f_rot = Omega_h / (2 * math.pi)
    R["abtastung"] = dict(f_rot_Hz=f_rot, f_blatt_Hz=3 * f_rot, fs_Hz=1000, nyquist_Hz=500,
                          alias_blatt_Hz=abs(3 * f_rot - 1000))

    (out / "rechnungen.json").write_text(json.dumps(R, indent=2, ensure_ascii=False))

    # ---------- Abbildungen ----------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    ax[0].plot(n**2 / 1e5, T, "o", label="Herstellerdaten T76")
    nn = np.linspace(0, n.max(), 50)
    ax[0].plot(nn**2 / 1e5, kT_pts[:-1].mean() * (2 * math.pi * nn) ** 2, "-",
               label=f"T = k_T ω², k_T = {kT_pts[:-1].mean():.3g} N s²")
    ax[0].set_xlabel("n² [10⁵ s⁻²]")
    ax[0].set_ylabel("Schub T [N]")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[0].set_title("Schub proportional zum Quadrat der Drehzahl")
    ax[1].plot(T76[:, 0], T76[:, 4], "o", label="gemessen")
    ax[1].plot(T76[:, 0], rpm_model, "s", mfc="none", label="Motorgleichung Kv (U − I R)")
    ax[1].set_xlabel("Gas [%]")
    ax[1].set_ylabel("Drehzahl [U/min]")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    ax[1].set_title("Drehzahl: Messung und Motormodell")
    fig.tight_layout()
    fig.savefig(out / "schub_drehzahl.png", dpi=200)
    print(json.dumps(R, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
