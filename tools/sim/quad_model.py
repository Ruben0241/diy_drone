"""
quad_model.py - Physikalisches Modell des Quadrocopters (Build A/B, gleiche Mechanik)

Starrkoerper mit 6 Freiheitsgraden:
  Translation:  m * a = R * (0, 0, sum T_i) + m * g - c_v * v          (Newton)
  Rotation:     J * w_dot = tau - w x (J * w)                          (Drallsatz / Euler-Gleichungen)
  Rotor i:      T_i = k_T * Omega_i^2,  Q_i = k_Q * Omega_i^2          (Schub und Widerstandsmoment)
                + J_r * dOmega_i/dt als Reaktionsmoment beim Beschleunigen (wichtig fuer Yaw)
  Motor i:      tau_m * Omega_i_dot = Omega_cmd,i - Omega_i            (Verzoegerung 1. Ordnung)
                Omega_cmd = u * Omega_max                              (u = DShot-Befehl 0..1)

Koordinaten (wie Firmware): x vorne, y links, z oben. Motoren wie Betaflight Quad-X:
  M1 hinten rechts, M2 vorne rechts, M3 hinten links, M4 vorne links; M1, M4 drehen im
  Uhrzeigersinn (von oben), M2, M3 dagegen ("props in").

Alle Zahlenwerte stehen in DroneParams mit Herkunft. "SCHAETZUNG" muss durch eine
Messung ersetzt werden (Kap. Physik: Waage, Schubmessung, Bifilarpendel).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

G = 9.81


@dataclass
class DroneParams:
    # --- Masse [kg] ---
    m_motor: float = 0.0164        # T-Hobby F2004 KV3000 Datenblatt: 16.4 g inkl. Kabel
    m_prop: float = 0.00235        # HQProp T90MMX3 (GetFPV: 2.35 g)
    m_battery: float = 0.209       # CNHL MiniStar 1800 mAh 4S (FPV24: 209 g)
    m_rest: float = 0.137          # SCHAETZUNG: Frame, Stack, Empfaenger, Kabel, Schrauben
    # --- Geometrie [m] ---
    motor_dx: float = 0.2211 / 2   # Motorabstand 221.1 mm (Frame v3) - Annahme: in Laengsrichtung
    motor_dy: float = 0.1542 / 2   # Motorabstand 154.2 mm (Frame v3) - Annahme: in Querrichtung
    battery_size: tuple = (0.086, 0.034, 0.0385)   # L x B x H (FPV24)
    battery_dz: float = 0.035      # SCHAETZUNG: Akkumitte ueber der Motorebene
    body_size: tuple = (0.10, 0.06, 0.02)          # SCHAETZUNG: Rumpf als Quader
    # --- Rotor/Motor ---
    k_T: float = 7.0e-7            # N s^2, aus T-Hobby-Testdaten (T76) auf 90 mm skaliert (siehe rotor_from_data)
    k_Q: float = 6.75e-9           # N m s^2, ebenso
    omega_max: float = 3620.0      # rad/s, aus maximaler mechanischer Leistung (siehe rotor_from_data)
    tau_motor: float = 0.020       # s, aus Motorgleichungen (siehe motor_time_constant)
    J_rotor: float = 2.4e-6        # kg m^2, Rotor (Glocke + Propeller), SCHAETZUNG siehe motor_time_constant
    # --- Sonstiges ---
    c_v: float = 0.10              # N s/m, SCHAETZUNG: Luftwiderstand (linear)
    gyro_noise_dps: float = 0.05   # ICM-42688-P: 2.8 mdps/sqrt(Hz) * sqrt(258 Hz)
    vibration_dps: float = 2.0     # SCHAETZUNG: Vibration pro Motor bei Drehfrequenz
    J: np.ndarray = field(default=None)

    def __post_init__(self):
        if self.J is None:
            self.J = inertia(self)

    @property
    def mass(self) -> float:
        return 4 * (self.m_motor + self.m_prop) + self.m_battery + self.m_rest

    def motor_positions(self) -> np.ndarray:
        a, b = self.motor_dx, self.motor_dy
        return np.array([[-a, -b], [a, -b], [-a, b], [a, b]])   # M1..M4 (x, y)

    @property
    def spin(self) -> np.ndarray:
        """+1: Propeller dreht im Uhrzeigersinn (Reaktionsmoment gegen den Uhrzeigersinn, +z)"""
        return np.array([1.0, -1.0, -1.0, 1.0])


def inertia(p: DroneParams) -> np.ndarray:
    """Traegheitstensor (Hauptachsen) aus Punktmassen und Quadern, Steiner-Satz.
    Bezugspunkt: gemeinsamer Schwerpunkt."""
    parts = []   # (Masse, Lage (x,y,z), Eigentraegheit diag)
    mp = p.m_motor + p.m_prop
    for (x, y) in p.motor_positions():
        parts.append((mp, np.array([x, y, 0.0]), np.zeros(3)))
    L, B, H = p.battery_size
    mb = p.m_battery
    parts.append((mb, np.array([0, 0, p.battery_dz]),
                  mb / 12 * np.array([B**2 + H**2, L**2 + H**2, L**2 + B**2])))
    L, B, H = p.body_size
    mr = p.m_rest
    parts.append((mr, np.zeros(3), mr / 12 * np.array([B**2 + H**2, L**2 + H**2, L**2 + B**2])))
    m = sum(q[0] for q in parts)
    cg = sum(q[0] * q[1] for q in parts) / m
    J = np.zeros(3)
    for mi, r, j0 in parts:
        d = r - cg
        J += j0 + mi * np.array([d[1]**2 + d[2]**2, d[0]**2 + d[2]**2, d[0]**2 + d[1]**2])
    return J


def rotor_from_data():
    """Rotorkonstanten aus den T-Hobby-Testdaten (F2004 KV3000, 4S, Propeller T76, 100 % Gas):
    T = 767.72 g, n = 43870 U/min, P_el = 351.87 W. Annahmen: Motorwirkungsgrad 0.8;
    gleiche Bauart -> k_T ~ D^4, k_Q ~ D^5 (aus C_T, C_P = konstant, Brandt & Selig 2011)."""
    T = 0.76772 * G
    w = 43870 * 2 * math.pi / 60
    kT76 = T / w**2
    kQ76 = 0.8 * 351.87 / w**3
    s = 90 / 76
    kT, kQ = kT76 * s**4, kQ76 * s**5
    P_mech_max = 0.8 * 400.0          # ca. 400 W elektrisch (Datenblatt max. 460 W / 60 s)
    w_max = (P_mech_max / kQ) ** (1 / 3)
    return dict(kT76=kT76, kQ76=kQ76, kT=kT, kQ=kQ, omega_max=w_max, T_max=kT * w_max**2)


def rotor_inertia(m_prop=0.00235, r_prop=0.045, m_bell=0.005, r_bell=0.0115):
    """Traegheitsmoment eines Rotors (SCHAETZUNG, Messung siehe Kap. Physik):
    Propellerblaetter als duenne Staebe um die Nabe: J = m R^2 / 3,
    Glocke des Motors als duenner Ring: J = m r^2 (obere Grenze).
    Standardwerte: 2.35 g / 45 mm (HQProp T90MMX3), Glocke ca. 5 g / 11.5 mm (Annahme)."""
    return m_prop * r_prop**2 / 3 + m_bell * r_bell**2


def motor_time_constant(J_rotor=2.4e-6, R=0.090, kv_rpm=3000, kQ=6.75e-9, omega=1200.0):
    """Zeitkonstante des Motors (Drela 2005: v = i R + Omega/Kv, Q = i/Kv):
    Bei Spannungsvorgabe wirkt die Gegen-EMK wie eine Daempfung K_t^2 / R,
    dazu die Luftreibung dQ/dOmega = 2 k_Q Omega.
      1/tau = K_t^2 / (J R) + 2 k_Q Omega / J,   K_t = 1/Kv [N m / A]"""
    Kt = 60 / (2 * math.pi * kv_rpm)
    return 1.0 / (Kt**2 / (J_rotor * R) + 2 * kQ * omega / J_rotor)


class QuadSim:
    """Numerische Integration (halbimplizites Euler-Verfahren, Schritt dt_phys)."""

    def __init__(self, p: DroneParams | None = None, dt_phys: float = 1 / 8000, seed: int = 1):
        self.p = p or DroneParams()
        self.dt = dt_phys
        self.rng = np.random.default_rng(seed)
        self.reset()

    def reset(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.R = np.eye(3)                 # Koerper -> Welt
        self.w = np.zeros(3)               # rad/s, Koerpersystem
        self.Omega = np.zeros(4)           # Rotordrehzahl rad/s
        self.Omega_dot = np.zeros(4)
        self.u = np.zeros(4)               # Motorbefehl
        self.t = 0.0
        self.phase = np.zeros(4)           # Rotorwinkel (fuer Vibration)
        self.ext_torque = np.zeros(3)
        self.acc_world = np.zeros(3)       # am Boden: Normalkraft hebt die Gewichtskraft auf

    def forces(self):
        p = self.p
        T = p.k_T * self.Omega**2
        Q = p.k_Q * self.Omega**2
        xy = p.motor_positions()
        tau = np.array([np.sum(xy[:, 1] * T), -np.sum(xy[:, 0] * T), np.sum(p.spin * Q)])
        return T, tau

    def step(self):
        p, dt = self.p, self.dt
        # Motoren (1. Ordnung)
        cmd = np.clip(self.u, 0, 1) * p.omega_max
        self.Omega_dot = (cmd - self.Omega) / p.tau_motor
        self.Omega += self.Omega_dot * dt
        self.phase += self.Omega * dt
        T, tau = self.forces()
        # Reaktionsmoment beim Beschleunigen der Rotoren (Drallsatz fuer den Rotor)
        tau[2] += np.sum(p.spin * p.J_rotor * self.Omega_dot)
        tau = tau + self.ext_torque
        # Rotation: J w_dot = tau - w x (J w)
        J = p.J
        w_dot = (tau - np.cross(self.w, J * self.w)) / J
        self.w += w_dot * dt
        # Lage: R <- R * exp([w]x dt) (Rodrigues)
        th = np.linalg.norm(self.w) * dt
        if th > 1e-12:
            k = self.w / np.linalg.norm(self.w)
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            self.R = self.R @ (np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K)
        # Translation
        F = self.R @ np.array([0, 0, T.sum()]) + np.array([0, 0, -p.mass * G]) - p.c_v * self.vel
        self.acc_world = F / p.mass
        self.vel += self.acc_world * dt
        self.pos += self.vel * dt
        self.t += dt

    def gyro_reading(self):
        """Gyro [dps] im Koerpersystem: Drehrate + Rauschen + Vibration (Rotorfrequenz, 3. Harmonische)"""
        p = self.p
        vib = np.zeros(3)
        if p.vibration_dps > 0:
            for i in range(4):
                s = math.sin(self.phase[i]) + 0.5 * math.sin(3 * self.phase[i])
                vib += p.vibration_dps * s * np.array([1.0, 0.7, 0.3]) * (self.Omega[i] / p.omega_max)
        return np.degrees(self.w) + self.rng.normal(0, p.gyro_noise_dps, 3) + vib

    def acc_reading(self):
        """Beschleunigungssensor misst die spezifische Kraft (a - g) im Koerpersystem, in g"""
        f_world = self.acc_world - np.array([0, 0, -G])
        return self.R.T @ f_world / G + self.rng.normal(0, 0.002, 3)

    def euler_deg(self):
        R = self.R
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(-math.asin(max(-1, min(1, R[2, 0]))))
        return roll, pitch

    def hover_command(self) -> float:
        p = self.p
        return math.sqrt(p.mass * G / 4 / p.k_T) / p.omega_max
