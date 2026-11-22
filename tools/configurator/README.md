# Configurator für die eigene Firmware (Build B)

Bedienoberfläche wie der Betaflight Configurator, läuft im Browser und spricht über USB
(Web Serial) direkt mit der Firmware. Keine Installation nötig.

## Starten

- **macOS:** `start.command` im Finder doppelklicken (öffnet Chrome auf `http://localhost:8722`).
- **Alle Systeme:** `python tools/configurator/start_configurator.py`
- Nur **Google Chrome** oder **Microsoft Edge** (Safari/Firefox können kein Web Serial).
- **Demo** (ohne Drohne): Knopf «Demo» oder `index.html?demo`.

Dann «Verbinden» → Gerät `DIY Drone FC` (USB 1209:0001) wählen. Läuft noch Betaflight,
erscheint der Flightcontroller dort nicht – dann den Betaflight Configurator verwenden.

## Tabs

| Tab | Inhalt |
|---|---|
| Übersicht | 3D-Lage live, Arming-Sperrgründe (wie Betaflight), Gyro-/ACC-Kalibrierung, Neustart, Bootloader (DFU) |
| Empfänger | 8 Kanäle in µs, Sticks → Soll-Drehrate, Link-Qualität, gefundener UART, UART/Telemetrie einstellen |
| Motoren | Motortest bis 40 % (nur mit Häkchen «Propeller ab»), Drehrichtungs-Bild, Assistent für die Motor-Reihenfolge |
| PID & Rates | P/I/D/F je Achse, Actual Rates mit Kurve, I-Term-Relax, TPA, Airmode, Angle-Mode |
| Filter | Gyro-/D-Tiefpässe, Notch, Frequenzgang-Diagramm |
| Sensoren | Gyro, Soll/Ist, Beschleunigung, Lage als Live-Diagramme (50 Hz) |
| Bodentest | `docs/bodentest.md` mit automatischer Erkennung, korrigiert falsche Vorzeichen, Testprotokoll als .md |
| Alle Einstellungen | alle 74 Parameter mit Beschreibung, Suche, Sichern/Laden als `diff`-Datei |
| Flugdaten | Blackbox herunterladen → CSV pro Flug (gleiches Format wie `fc_cli.py dump`), Vorschau-Diagramm, Löschen |
| CLI | Befehlszeile der Firmware |

Änderungen werden erst mit **Speichern** (gelbe Leiste unten) in den Flightcontroller geschrieben
(`set …` + `save`). Kalibrierungen und Korrekturen aus dem Bodentest wirken sofort, bleiben aber
erst nach **Speichern** nach einem Neustart erhalten.

## Sicherheit

- Motortest nur disarmed, max. 40 %, nur mit Häkchen «Propeller ab».
- Der Configurator sendet alle 250 ms ein «weiter so» (`motors a b c d 600`). Bleibt es aus
  (Tab gewechselt, Fenster verlassen, Kabel ab, Browser hängt), stoppt die Firmware die Motoren
  nach 0.6 s von selbst. Trennt sich USB, stoppt sie sofort.
- <kbd>Esc</kbd> oder <kbd>Leertaste</kbd> stoppt jederzeit.
- Geflogen wird nur mit der Fernsteuerung – der Configurator kann die Drohne nicht armen.

## Protokoll

Textbefehle über den virtuellen COM-Port (`src/app/usb_cli.c`). Für den Configurator:
`version`, `params` (`$P,name,wert,min,max,nachkomma,standard`), `stream <hz>`
(`$H,…` Feldnamen, dann `$T,…`, siehe `src/core/telemetry.h`), `echo <text>` (Ende einer Antwort),
`motors a b c d [ms]`, `gyrocal`. Dateien:

| Datei | Inhalt |
|---|---|
| `index.html` | Oberfläche |
| `protocol.js` | Zeilen/Live-Daten/Parameter lesen, Blackbox-Decoder (wie `buildb_log.py`) |
| `mock.js` | Demo-Flightcontroller (gleiche Antworten wie die Firmware) |
| `test/test_protocol.js` | prüft `protocol.js` und `mock.js` gegen Ausgaben der Firmware (`build-tests/`) |
