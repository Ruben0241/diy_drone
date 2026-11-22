# Designrichtlinie Configurator

Ziel: ruhig, lesbar, Werkzeug statt Spielzeug. Farbe nur dort, wo sie etwas bedeutet.

## 1. Farben

| Token | Wert | Verwendung |
|---|---|---|
| `--bg` | `#121314` | Seitenhintergrund |
| `--surface` | `#1a1b1d` | Panels, Navigation, Kopf- und Fusszeile |
| `--surface-2` | `#222326` | Eingabefelder, Knöpfe, Balken-Hintergrund |
| `--border` | `#2d2e32` | alle Linien und Rahmen (1 px) |
| `--text` | `#e4e4e6` | Text, Werte |
| `--text-2` | `#9b9da3` | Beschriftungen, Erklärungen |
| `--text-3` | `#66686e` | Achsen, Gitter-Beschriftung, deaktiviert |
| `--accent` | `#d6a446` | **einzige** Akzentfarbe |
| `--danger` | `#d9534f` | **nur** Gefahr und Fehler |

Regeln:
- **Eine Akzentfarbe.** Nur für: Primärknopf (pro Ansicht höchstens einer), aktiver Tab, Fokus/geänderte Felder, Füllbalken, Hauptlinie im Diagramm.
- **Rot nur für Gefahr:** ARMED, Failsafe, Fehler, Motortest aktiv, Propeller-Warnung, Löschen/STOP.
- **Kein Grün, Blau, Orange.** «OK» ist neutral (grauer Text/Punkt) – der Normalzustand braucht keine Farbe.
- Keine farbig gefüllten Flächen. Hinweise: neutrale Fläche mit Rahmen. Gefahr: roter Rahmen + roter Text, Hintergrund neutral.
- Keine Emojis oder bunten Icons.

## 2. Diagramme

- Höchstens 3 Linien: **Text-Weiss**, **Akzent**, **Grau gestrichelt** – immer in dieser Reihenfolge (Roll, Pitch, Yaw bzw. X, Y, Z).
- Soll/Ist: Soll = Grau gestrichelt, Ist = Akzent.
- Gitter `--border`, Beschriftung `--text-3`, 11 px.
- Motor-Drehrichtung nur mit Pfeil, nicht mit Farbe.

## 3. Schrift

- Systemschrift, Basis 14 px; Titel 20 px / 600; Panel-Titel 13 px / 600 in `--text-2`, Grossbuchstaben, Sperrung 0.5 px.
- Zahlenwerte in Monospace (`ui-monospace`), damit sie beim Aktualisieren nicht springen.

## 4. Abstände und Formen

- Raster 4 px: 4 / 8 / 12 / 16 / 24.
- Radius 6 px (Panels), 4 px (Felder, Knöpfe).
- Rahmen statt Schatten (Ausnahme: Dialog, Toast).

## 5. Zustände

| Zustand | Darstellung |
|---|---|
| normal / OK | `--text-2`, grauer Punkt |
| geändert, nicht gespeichert | Akzent-Rahmen am Feld, Speicherleiste unten |
| ≠ Standardwert | Wert fett (keine Farbe) |
| Gefahr / Fehler | `--danger` als Rahmen und Text |
| deaktiviert | Deckkraft 40 % |
