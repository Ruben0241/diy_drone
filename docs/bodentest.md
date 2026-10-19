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

Gas leicht erhöhen. Jeden Motor einzeln kurz mit dem Finger antippen und prüfen:

| Motor | Position | Drehrichtung (Betaflight-Standard «Props in») |
|---|---|---|
| M1 | hinten rechts | [aus Betaflight Motors-Tab übernehmen] |
| M2 | vorne rechts | |
| M3 | hinten links | |
| M4 | vorne links | |

Stimmt die Position nicht: in Betaflight wurde eventuell eine Motor-Neuzuordnung
(`resource MOTOR`) verwendet. In `diff all` nachsehen.

## 4. Regelrichtung (wichtigster Test)

Armed, Gas ca. 20 %, Drohne in der Hand. Schnell um eine Achse drehen und hören,
welche Motoren **schneller** werden:

| Bewegung | Diese Motoren müssen schneller werden | Korrektur in `main.c`, falls falsch |
|---|---|---|
| rechte Seite nach unten kippen | rechts (M1, M2) | `cfg.gyro_sign[AXIS_ROLL] = -1.0f;` |
| Nase nach unten kippen | vorne (M2, M4) | `cfg.gyro_sign[AXIS_PITCH] = -1.0f;` |
| im Uhrzeigersinn drehen (von oben) | die im Uhrzeigersinn drehenden Motoren | `cfg.gyro_sign[AXIS_YAW] = -1.0f;` |

Danach dieselbe Prüfung mit den Sticks (Drohne still):

| Stick | Diese Motoren müssen schneller werden | Korrektur |
|---|---|---|
| Roll nach rechts | links (M3, M4) | `cfg.stick_sign[AXIS_ROLL] = -1.0f;` |
| Pitch nach vorne | hinten (M1, M3) | `cfg.stick_sign[AXIS_PITCH] = -1.0f;` |
| Yaw nach rechts | die gegen den Uhrzeigersinn drehenden Motoren | `cfg.stick_sign[AXIS_YAW] = -1.0f;` |

> Für die Yaw-Achse gilt: Ein Propeller, der im Uhrzeigersinn dreht, erzeugt ein
> Reaktionsmoment gegen den Uhrzeigersinn (Kap. 2.1).

## 5. Failsafe

Armed, Gas ca. 20 %: Fernsteuerung ausschalten → Motoren müssen **innerhalb von 0.25 s**
stoppen. Fernsteuerung wieder ein → bleibt aus, bis der Arm-Schalter aus und wieder ein ist.

## 6. Ergebnis ins Testprotokoll

Datum, Firmware-Version (Git-Commit), Ergebnis jeder Zeile, vorgenommene Korrekturen.
