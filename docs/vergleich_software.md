# Abgleich mit bestehender Flugsoftware

Ziel: prüfen, dass im eigenen Regler **kein Teil fehlt**, der für einen sicheren Rate-Mode-Flug
nötig ist, und festhalten, was bewusst weggelassen wurde. Es wurde **kein Code übernommen**,
nur Aufbau und Zahlenwerte verglichen.

Verglichen (Stand Oktober 2026):

| Software | Was angeschaut | Link |
|---|---|---|
| Betaflight (master) | `src/main/flight/pid.c`, `pid_init.c`, `fc/rc.c`, `fc/core.c`, `fc/controlrate_profile.c` | https://github.com/betaflight/betaflight |
| INAV | Dokumentation «INAV PID Controller» | https://github.com/iNavFlight/inav/blob/master/docs/INAV%20PID%20Controller.md |
| PX4 | `src/lib/rate_control/rate_control.cpp`, Reglerdiagramme | https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/rate_control/rate_control.cpp, https://docs.px4.io/main/en/flight_stack/controller_diagrams.html |
| dRehmFlight (Beta 1.3) | `controlRATE()`, `controlMixer()` | https://github.com/nickrehm/dRehmFlight |

## 1. Regler (innere Schleife, Drehrate)

| Baustein | Build B | Betaflight | INAV | PX4 | dRehmFlight |
|---|---|---|---|---|---|
| P auf Fehler | ✔ | ✔ | ✔ | ✔ | ✔ |
| I mit fester Grenze | ✔ 80 % von pidsum (400 / 320) | ✔ `iterm_windup` 80 % | ✔ | ✔ | ✔ `i_limit` |
| I nicht am Boden / Gas unten | ✔ (bis Airmode aktiv) | ✔ | ✔ | ✔ `landed` | ✔ Gas < 1060 µs |
| Anti-Windup bei Motorsättigung | ✔ `iterm_mix_windup` | – (bis 4.5: ja) | ✔ Back-Calculation | ✔ Sättigungsflags | – |
| I-Term-Relax | ✔ (aus für Vergleich) | ✔ RP, 40 dps, 15 Hz | ✔ | ähnlich (i_factor) | – |
| D auf Messwert (kein D-Kick) | ✔ | ✔ | ✔ | ✔ (Winkelbeschl.) | – (D auf Fehler) |
| 2 Tiefpässe auf D | ✔ PT1/Biquad | ✔ (dynamisch) | ✔ | 1 | – |
| Feedforward | ✔ Ableitung Sollwert, PT1 | ✔ mit Mittelung, Jitter-Reduktion, Boost | ✔ «CD» | ✔ statisch (k_ff · Sollwert) | – |
| Tiefpass auf Yaw-P | ✔ 100 Hz | ✔ `yaw_lowpass_hz` 100 | – | – | – |
| TPA | ✔ Modus D (aus) | ✔ D, ab 1350, 65 % | ✔ | – | – |
| Summenbegrenzung | ✔ 500 / 500 / 400 | ✔ 500 / 400 Yaw | ✔ | über Mischer | – |
| Schutz gegen NaN | ✔ | – | – | ✔ `PX4_ISFINITE` | – |

## 2. Rund um den Regler

| Baustein | Build B | Betaflight |
|---|---|---|
| Gyro-Filter | 2 Stufen PT1/Biquad, statischer Notch | dynamische Tiefpässe, dynamischer Notch, RPM-Filter |
| Stick-Kennlinie | Actual Rates (Formel aus `rc.c`) | Actual Rates (Standard) |
| Glättung der Sticks | PT1 40 Hz | automatisch (PT3) |
| Mischer Quad-X mit Desaturierung | ✔ (Airmode) | ✔ |
| Airmode | ✔ ab 25 % Gas | ✔ `airmode_start_throttle_percent` 25 |
| Runaway-Schutz | ✔ Werte aus `core.c` | ✔ |
| Angle-Mode | ✔ P auf Winkel, 60° | ✔ (mit Feedforward, Erdbezug) |
| Arming-Sperren | Gas, Lage, Kalibrierung, Schalter, USB-Befehle | viele weitere (GPS, OSD, …) |
| Failsafe | sofort Motoren aus (250 ms) | 2 Stufen (Landen, dann aus) |
| Regelfrequenz | 1 kHz | 4–8 kHz |

## 3. Bewusst weggelassen

| Funktion | Grund |
|---|---|
| Dynamischer Notch, RPM-Filter, bidirektionales DShot | Aufwand; für den Vergleich in Betaflight ebenfalls aus |
| Anti-Gravity, D-Max, Throttle-Boost | Verbesserungen, keine Grundfunktion; im Vergleich aus |
| Schubkurve linearisieren, Spannungskompensation, Dynamic Idle | dito |
| Crash-Recovery, Horizon, Acro-Trainer, Launch-Control, EZ-Landing | für die Messflüge nicht nötig |
| 2-stufiges Failsafe | Drohne fliegt nur in Sichtweite über weichem Boden; sofort aus ist einfacher zu prüfen |
| GPS, OSD, MSP, Konfigurator | nicht Teil der Fragestellung |

## 4. Betaflight auf «reinen PID» stellen (Build A, Vergleich 1)

Damit sich Build A und Build B nur im **eigenen Code** unterscheiden, werden in Betaflight
die Zusatzfunktionen ausgeschaltet. Einstellungsnamen aus `src/main/fc/parameter_names.h`
(master). **Vor dem Einfügen mit `get <name>` prüfen**, ob es den Namen in der installierten
Version gibt (`version` im CLI), sonst Fehlermeldung.

```
# Zusatzfunktionen aus
set f_roll = 0
set f_pitch = 0
set f_yaw = 0
set iterm_relax = OFF
set anti_gravity_gain = 0
set tpa_rate = 0
set tpa_low_rate = 0
set d_max_roll = 0
set d_max_pitch = 0
set throttle_boost = 0
set thrust_linear = 0
set vbat_sag_compensation = 0
set dyn_idle_min_rpm = 0
# Filter wie Build B (statisch, PT1)
set simplified_pids_mode = OFF
set simplified_dterm_filter = OFF
set simplified_gyro_filter = OFF
set dyn_notch_count = 0
set dshot_bidir = OFF
set rpm_filter_harmonics = 0
set gyro_lpf1_dyn_min_hz = 0
set gyro_lpf1_type = PT1
set gyro_lpf1_static_hz = 150
set gyro_lpf2_static_hz = 0
set dterm_lpf1_dyn_min_hz = 0
set dterm_lpf1_type = PT1
set dterm_lpf1_static_hz = 75
set dterm_lpf2_type = PT1
set dterm_lpf2_static_hz = 150
# Raten: 400 °/s linear
set rates_type = ACTUAL
set roll_rc_rate = 40
set pitch_rc_rate = 40
set yaw_rc_rate = 40
set roll_srate = 40
set pitch_srate = 40
set yaw_srate = 40
set roll_expo = 0
set pitch_expo = 0
set yaw_expo = 0
# Motoren, Logging
set motor_pwm_protocol = DSHOT300
# bis Version 4.5 heisst es dshot_idle_value
set motor_idle = 550
save
```

Danach `diff all` speichern (Beleg für die Arbeit). `pid_process_denom` so setzen, dass die
PID-Schleife mit **1 kHz** läuft (CLI `status` zeigt die Gyro-Rate; bei 8 kHz → 8) und
`blackbox_sample_rate` auf jeden Zyklus.

Gleiche Werte in Build B: `defaults` (das sind genau diese Werte). Für **Vergleich 2**
(Betaflight-Standard gegen Build B mit allen Funktionen) in Build B:

```
set f_roll 120
set f_pitch 125
set f_yaw 120
set iterm_relax 1
set tpa_rate 0.65
set tpa_breakpoint 0.35
save
```

## 5. Ergebnis

Alle Bausteine, die in **allen** vier Programmen vorkommen (P, I mit Grenze, I-Sperre am Boden,
D, Mischer, Begrenzung), sind umgesetzt; ebenso die Schutzfunktionen von Betaflight (Airmode,
Runaway, Arming-Sperren). Getestet in `tests/test_core.c` (450 Prüfungen) und in der
Simulation (`tools/sim/test_sim.py`).
