# Simulation (Software-in-the-Loop)

Der **unveränderte Firmware-Kern** (`src/core`) fliegt in einem physikalischen Modell der
Drohne. So lassen sich Regelung, Filter und Schutzfunktionen testen, bevor die Drohne
zum ersten Mal abhebt. Die Simulation ersetzt den Bodentest **nicht**: alle Zahlenwerte
mit «SCHAETZUNG» in `quad_model.py` müssen durch Messungen ersetzt werden.

```bash
cmake -S tests -B build-tests && cmake --build build-tests   # baut libfc_core.so
pip install -r tools/analysis/requirements.txt
python tools/sim/test_sim.py                                 # Tests (auch in der CI)
python tools/sim/run_sim.py --out sim_ergebnisse             # Sprungantworten + Kennzahlen
python tools/sim/aliasing_demo.py --out sim_ergebnisse       # Abtasttheorem am Gyro-Signal
```

## Modell (`quad_model.py`)

| Teil | Gleichung | Quelle |
|---|---|---|
| Schub, Widerstandsmoment | T = k_T Ω², Q = k_Q Ω² | Brandt & Selig 2011; Bouabdallah 2007 |
| Motor | τ_m dΩ/dt = Ω_cmd − Ω, τ_m aus K_t²/(J R) | Drela 2005 |
| Rotor-Reaktionsmoment | τ_z += Σ ± J_r dΩ/dt | Drallsatz |
| Drehung | J ω̇ = τ − ω × (J ω) (Euler-Gleichungen) | OpenStax Physics Vol. 1, Kap. 10–11 |
| Translation | m a = R (0,0,ΣT) + m g − c_v v | Newton |
| Trägheitsmomente | Punktmassen + Quader, Steiner-Satz | OpenStax Vol. 1, Kap. 10.5 |
| Beschleunigungssensor | misst spezifische Kraft (a − g) | Martin 2014 |
| Gyro | Drehrate + Rauschen + Vibration bei f_rot und 3 f_rot | ICM-42688-P Datenblatt |

Integration: halbimplizites Euler-Verfahren mit 8 kHz, Regelung mit 1 kHz (wie Firmware).

## Ergebnisse (Standardwerte, Betaflight-PIDs)

| Achse | Anstiegszeit | Überschwingen | Ausregelzeit |
|---|---|---|---|
| Roll | 16 ms | 17 % | 147 ms |
| Pitch | 17 ms | 22 % | 163 ms |
| Yaw | 60 ms | 6 % | 264 ms |

- Schweben: Lage stabil auf ±0.01°, Gasstellung ≈ 0.30 (Motorbefehl 0.34).
- Böe 0.05 N·m für 0.2 s: max. 1.4° Roll, nach 1 s ausgeregelt.
- Falsches Gyro-Vorzeichen: Runaway-Schutz schaltet 0.15 s nach dem Abheben ab – die
  Drohne hat sich bis dahin aber schon überschlagen. → Der Bodentest bleibt Pflicht.
- Ohne Rotor-Reaktionsmoment im Modell schwingt Yaw um ~70 % über: Bei Yaw trägt das
  Beschleunigen der Rotoren (J_r dΩ/dt) wesentlich zum Drehmoment bei.

## Physik-Kapitel der Begleitarbeit

```bash
python tools/sim/physik_rechnungen.py --out physik    # alle Zahlen + Abb. Schub/Drehzahl
python tools/sim/vergleich_theorie.py --out physik    # Formel (Schwinger 2. Ordnung) gegen Simulation
```

| Roll-Regler | Formel: Anstieg / Überschwingen | Simulation |
|---|---|---|
| nur P | 11.6 ms / 47 % | 10.8 ms / 54 % |
| P + D | 23.9 ms / 1 % | 20.6 ms / 3 % |
| PID (Standard) | – | 15.7 ms / 17 % |
