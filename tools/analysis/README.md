# Auswertung Build A / Build B (Kap. 3.4.5, Kap. 6)

Schätzt aus den Flugdaten die Sprungantwort pro Achse und berechnet die Kennzahlen
für den Vergleich. Beide Builds werden mit **demselben** Verfahren ausgewertet.

## Installation

```bash
pip install -r tools/analysis/requirements.txt
```

## Ablauf

1. **Build A:** Blackbox-Log aus Betaflight auslesen (Tab «Dataflash» → *Save flash to file*),
   mit `blackbox_decode` (betaflight/blackbox-tools) in CSV umwandeln:
   `blackbox_decode LOG00001.BFL` → `LOG00001.01.csv`
   Benötigt ein Log aus Betaflight ≥ 4.0 (Spalten `setpoint[0..2]`).
2. **Build B:** `python tools/fc_cli.py dump --prefix JJJJMMTT` lädt alle Flüge über USB
   und schreibt pro Flug eine CSV mit den Spalten
   `t_us, sp_roll, sp_pitch, sp_yaw, gyro_roll, gyro_pitch, gyro_yaw, throttle, m1..m4, flags, vbat, lq`.
   Eine vorhandene `.bin` lässt sich mit `python tools/analysis/buildb_log.py datei.bin` umwandeln.
3. Auswerten:

```bash
python tools/analysis/fc_analysis.py \
    --a flug_A1.csv flug_A2.csv flug_A3.csv \
    --b flug_B1.csv flug_B2.csv flug_B3.csv \
    --out ergebnisse/ --band 0.05 --tolerance 0.20
```

Ausgabe in `ergebnisse/`: `kennzahlen.json`, `vergleich.csv`, `sprungantwort.png`.

## Verfahren

| Schritt | Umsetzung |
|---|---|
| Zeitraster | alle Daten auf 1 kHz interpoliert |
| Fenster | 2 s, Verschiebung 1/16 Fenster, Hanning-Fenster |
| Auswahl | nur Fenster mit max. Sollwert ≥ 20 °/s und Gas zwischen 5 % und 90 % |
| Dekonvolution | Wiener-Filter, Signal-Rausch-Verhältnis fällt oberhalb 25 Hz ab |
| Mittelung | Median über alle plausiblen Fenster |
| Kennzahlen | Anstiegszeit 10–90 %, Überschwingen, Ausregelzeit (± Band) bezogen auf den Endwert y(∞); bleibende Abweichung = y(∞) − 1; RMS von Soll − Ist |
| Vergleich | relativ \|B − A\| / \|A\| ≤ Toleranz; bleibende Abweichung absolut ≤ 5 Prozentpunkte |

Prinzip wie PID-Analyzer (Plasmatree), eigene Umsetzung. Abweichung: Fenster 2 s statt 1 s,
weil das im Test den Fehler beim Endwert von −2.4 % auf unter 0.1 % senkte.

## Genauigkeit (Test mit simuliertem Regelkreis, ζ = 0.5, f₀ = 12 Hz, Handsteuer-Eingaben, Rauschen)

| Kennzahl | wahr | geschätzt |
|---|---|---|
| Anstiegszeit | 21.7 ms | 20.8 ms |
| Überschwingen | 16.3 % | 17.4 % |
| Ausregelzeit (±5 %) | 70 ms | 72 ms |
| bleibende Abweichung | 0 % | −0.05 % |

Tests: `python tools/analysis/test_fc_analysis.py` und `python tools/analysis/test_buildb_log.py`
