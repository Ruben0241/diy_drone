# Bodentest (Manöver M0) – vor jedem ersten Flug mit neuer Firmware

**Immer ohne Propeller.** Akku über den Smoke Stopper anschliessen.

## 1. Start

| Prüfung | Erwartung | Wenn nicht |
|---|---|---|
| Kurzer Piepser beim Einschalten | Firmware läuft | Flash prüfen, Board-Target prüfen |
| Dauerndes kurzes Piepsen (alle 0.5 s) | **Fehler**: Gyro nicht gefunden oder Absturz | Board ist nicht GEPRCF722 oder Gyro-Pins anders |
| LED 0 blinkt schnell | Gyro-Kalibrierung läuft oder kein Empfänger | Drohne still hinstellen, Fernsteuerung einschalten |
| LED 0 blinkt langsam | bereit | – |

## 2. Arming

1. Arm-Schalter (Kanal 5) aus, Gas ganz unten.
2. Arm-Schalter ein → Piepser, LED 0 dauernd an, alle Motoren drehen langsam (Leerlauf).
3. Arm-Schalter beim Einschalten schon an → darf **nicht** armen.
4. Mit Gas oben → darf **nicht** armen.

## 3. Motor-Reihenfolge und Drehrichtung

Disarmed, **Propeller ab**, Akku am Smoke Stopper, USB verbunden: `python tools/fc_cli.py term`,
dann nacheinander `motor 1 8`, `motor 2 8`, `motor 3 8`, `motor 4 8` (8 % Gas, stoppt nach 3 s von selbst; sofort aus: `motor stop`).
Prüfen, welcher Motor dreht und in welche Richtung (Klebeband-Fähnchen auf der Glocke):

| Motor | Position | Drehrichtung (Betaflight-Standard «Props in») |
|---|---|---|
| M1 | hinten rechts | [aus Betaflight Motors-Tab übernehmen] |
| M2 | vorne rechts | |
| M3 | hinten links | |
| M4 | vorne links | |

Stimmt die Position nicht: in Betaflight wurde eventuell eine Motor-Neuzuordnung
(`resource MOTOR`) verwendet. In `diff all` nachsehen.

## 3b. Sensorrichtung ohne Motoren (über USB)

Akku **ab**, nur USB. `python tools/fc_cli.py live` starten und die Drohne bewegen:

| Bewegung | Erwartung in der Anzeige | Korrektur (in `fc_cli.py term`), falls falsch |
|---|---|---|
| rechte Seite nach unten kippen | `att r=` wird positiv, `gyro r=` beim Kippen positiv | `set gyro_sign_roll -1` |
| Nase nach unten kippen | `att p=` wird positiv, `gyro p=` beim Kippen positiv | `set gyro_sign_pitch -1` |
| von oben gesehen gegen den Uhrzeigersinn drehen | `gyro y=` positiv | `set gyro_sign_yaw -1` |
| Sticks bewegen (armed nicht nötig) | Roll rechts → `sp r=` positiv, Pitch vorne → `sp p=` positiv, **Yaw rechts → `sp y=` negativ** | `set stick_sign_… -1` (bzw. `1`) |

> Achtung: Die Lage (`att`) kann nur stimmen, wenn auch das Gyro-Vorzeichen stimmt.
> Zuerst `gyro`, dann `att` prüfen. Danach folgt Schritt 4 mit Motoren als Gegenprobe.

## 4. Regelrichtung (wichtigster Test)

Armed, Gas ca. 20 %, Drohne in der Hand. Schnell um eine Achse drehen und hören,
welche Motoren **schneller** werden:

| Bewegung | Diese Motoren müssen schneller werden | Korrektur (in `fc_cli.py term`), falls falsch |
|---|---|---|
| rechte Seite nach unten kippen | rechts (M1, M2) | `set gyro_sign_roll -1` |
| Nase nach unten kippen | vorne (M2, M4) | `set gyro_sign_pitch -1` |
| im Uhrzeigersinn drehen (von oben) | die im Uhrzeigersinn drehenden Motoren | `set gyro_sign_yaw -1` |

Danach dieselbe Prüfung mit den Sticks (Drohne still):

| Stick | Diese Motoren müssen schneller werden | Korrektur |
|---|---|---|
| Roll nach rechts | links (M3, M4) | `set stick_sign_roll -1` |
| Pitch nach vorne | hinten (M1, M3) | `set stick_sign_pitch -1` |
| Yaw nach rechts | die gegen den Uhrzeigersinn drehenden Motoren | `set stick_sign_yaw -1` |

> Für die Yaw-Achse gilt: Ein Propeller, der im Uhrzeigersinn dreht, erzeugt ein
> Reaktionsmoment gegen den Uhrzeigersinn (Kap. 2.1).

Alle Korrekturen danach mit `save` speichern und mit `diff` ins Testprotokoll übernehmen.
Die Simulation zeigt, was ein falsches Vorzeichen bewirkt: Die Drohne überschlägt sich in
< 0.2 s, bevor der Runaway-Schutz abschaltet (`tools/sim/README.md`).

## 4b. Beschleunigungssensor kalibrieren (für Angle-Mode)

Drohne auf eine **waagrechte** Fläche stellen (Wasserwaage), `acccal`, 1 s nicht berühren,
danach `save`. Kontrolle mit `live`: `att r=` und `att p=` unter 1°.

## 5. Failsafe

Armed, Gas ca. 20 %: Fernsteuerung ausschalten → Motoren müssen **innerhalb von 0.25 s**
stoppen. Fernsteuerung wieder ein → bleibt aus, bis der Arm-Schalter aus und wieder ein ist.

## 6. Logging

Armen, 10 s laufen lassen, disarmen. `python tools/fc_cli.py dump --prefix test` →
eine Datei `logs/test_B_01.csv` mit ca. 10 000 Zeilen muss entstehen. Danach `erase`.

## 7. Ergebnis ins Testprotokoll

Datum, Firmware-Version (Git-Commit), Ergebnis jeder Zeile, vorgenommene Korrekturen.
