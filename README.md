# DIY Drone FC – eigene Flugsteuerung für den GEPRC TAKER F722

Maturaarbeit «Entwicklung einer FPV-Drohne mit eigenem PID-Flightcontroller» (KZU, 6j).

Eigene Firmware (C, FreeRTOS) für den Flightcontroller **GEPRC TAKER F722 (GEP-F722-HD V2)**
mit STM32F722 und ICM-42688-P. Build A fliegt mit Betaflight, Build B mit dieser Firmware
auf **derselben Hardware**. Dadurch unterscheidet sich im Vergleich nur die Software.

> ⚠️ **Status: noch nie auf echter Hardware gelaufen.** Kompiliert ohne Warnungen und
> besteht die Host-Tests. Pinbelegung, Vorzeichen und Timing müssen am Board geprüft
> werden. Erste Tests **nur ohne Propeller** (siehe [docs/bodentest.md](docs/bodentest.md)).

## Funktionsumfang

| Modul | Datei | Stand |
|---|---|---|
| Takt 216 MHz (HSI → PLL, Over-Drive) | `src/board/system.c` | fertig |
| ICM-42688-P über SPI3, 1 kHz, Data-Ready-Interrupt | `src/board/icm42688.c` | fertig |
| CRSF-Empfang über UART2 (420 000 Baud) | `src/board/rx_uart.c`, `src/core/crsf.c` | fertig |
| DShot300 auf 4 Motoren (TIM3 + DMA-Burst) | `src/board/dshot_tim3.c`, `src/core/dshot.c` | fertig |
| Lageschätzung Mahony | `src/core/mahony.c` | fertig |
| PID je Achse (Betaflight-Skalierung, D auf Istwert, Anti-Windup) | `src/core/pid.c` | fertig |
| Quad-X-Mischer mit Desaturierung | `src/core/mixer.c` | fertig |
| Rate- und Angle-Mode, Arming, Failsafe, Gyro-Kalibrierung | `src/core/flight.c` | fertig |
| Watchdog, Fehler-Handler (Motoren aus) | `src/board/system.c`, `src/app/main.c` | fertig |
| Akkuspannung, LEDs, Summer | `src/board/peripherals.c` | fertig, Werte prüfen |
| **Logging in den 16-MB-Flash und Export als CSV** | `src/app/main.c` (`log_task`) | **offen** |
| USB (Konfiguration, Log-Download) | – | offen |
| Bidirektionales DShot / RPM-Filter | – | optional |

## Aufbau

```
src/core/    hardwareunabhängig, auf dem PC testbar (PID, Mahony, CRSF, Mischer, DShot, Flugsteuerung)
src/board/   Register-Treiber für den GEPRC F722 (Pins in board.h)
src/app/     FreeRTOS-Tasks (main.c) und FreeRTOSConfig.h
tests/       Host-Tests der Kernmodule
lib/         FreeRTOS-Kernel, CMSIS (Git-Submodule)
docs/        Bodentest
tools/analysis/  Python-Auswertung A/B (Sprungantwort, Kennzahlen, Vergleich)
```

### Tasks (Begleitarbeit Kap. 2.4)

| Task | Priorität | Takt | Aufgabe |
|---|---|---|---|
| control | 4 | Gyro Data-Ready (1 kHz) | IMU lesen, Lage, PID, Mischer, DShot |
| rx | 3 | 1 kHz | CRSF auswerten, Zeitstempel für Failsafe |
| log | 2 | Queue | Messdaten (Flash folgt) |
| status | 1 | 10 Hz | LEDs, Summer, Akkuspannung |

### Pinbelegung

Quelle: Betaflight-Target `GEPRCF722` (`betaflight/config`, `configs/GEPR/GEPRCF722/config.h`).

| Funktion | Pins |
|---|---|
| Gyro ICM-42688-P | SPI3: SCK PC10, MISO PC11, MOSI PC12; CS PA15; INT PC3; Ausrichtung CW180 |
| Empfänger | UART2: TX PA2, RX PA3 **[prüfen: UART des RP3 in Betaflight → Ports]** |
| Motoren | M1 PB4, M2 PB5, M3 PB0, M4 PB1 (TIM3 CH1–4), DMA1 Stream 2 Kanal 5 |
| Akkuspannung | PC0 (ADC1 IN10) |
| LEDs / Summer | PA13, PA14 / PC13 |
| Blackbox-Flash | W25Q128, SPI2, CS PB12 (noch ungenutzt) |

## Bauen

Benötigt: `arm-none-eabi-gcc` (Arm GNU Toolchain), CMake ≥ 3.16.

macOS: `brew install cmake` und die Arm GNU Toolchain von developer.arm.com
(oder `brew install --cask gcc-arm-embedded`).

```bash
git clone --recursive https://github.com/Ruben0241/diy_drone
cd diy_drone

# Tests der Kernmodule auf dem PC
cmake -S tests -B build-tests && cmake --build build-tests && ./build-tests/test_core

# Firmware
cmake -S . -B build && cmake --build build
# Ergebnis: build/firmware.hex und build/firmware.bin
```

## Flashen

1. **Betaflight sichern**: Betaflight Configurator → CLI → `diff all` → Text als Datei speichern.
2. **Board prüfen**: CLI → `version` muss `GEPRCF722` zeigen. Sonst nicht flashen.
3. Betaflight Configurator → *Firmware Flasher* → *Load Firmware [Local]* → `build/firmware.hex` → *Flash Firmware*.
   Falls der Configurator die Datei ablehnt: STM32CubeProgrammer (USB/DFU) oder
   Alternativ im DFU-Modus: `dfu-util -a 0 -s 0x08000000:leave -D build/firmware.bin`
4. Diese Firmware hat **kein USB**. Für das nächste Flashen die Boot-Taste gedrückt halten
   und dann USB einstecken (DFU-Modus) **[prüfen: Lage der Boot-Taste am Board]**.

**Zurück zu Betaflight:** Firmware Flasher → Board `GEPRCF722` → Flashen → im CLI die
gespeicherte `diff all` einfügen → `save`.

## Einstellungen

Alle Standardwerte stehen in `flight_default_config()` (`src/core/flight.c`).
Überschreiben in `control_task()` in `src/app/main.c`.

| Parameter | Standard | Bemerkung |
|---|---|---|
| PID Roll / Pitch / Yaw | 45/80/30, 47/84/34, 45/80/0 | Betaflight-Standardwerte, gleiche Skalierung |
| Max. Drehrate | 400 °/s, linear | wie in Kap. 3.4 vorgeschlagen; in Betaflight gleich einstellen |
| Gyro-Tiefpass / D-Tiefpass | 150 Hz / 75 Hz | PT1 |
| Leerlauf | 5.5 % | wie Betaflight `dshot_idle_value` |
| Failsafe | 250 ms ohne Paket | Motoren sofort aus |
| Kanäle | 1 Roll, 2 Pitch, 3 Gas, 4 Yaw, 5 Arm, 6 Angle-Mode | ELRS/EdgeTX «AETR» |

**Unterschiede zu Betaflight** (wichtig für die Diskussion in der Arbeit): kein Feedforward,
keine dynamischen Notch-/RPM-Filter, kein TPA, keine Anti-Gravity, keine I-Term-Relax,
Regelfrequenz 1 kHz statt 4–8 kHz.

## Sicherheit

- Erste Tests ohne Propeller, danach gefesselt (Kap. 3.4.7).
- Arming nur mit Gas unten, Schalter-Wechsel aus → ein, Drohne flach (< 25°) und Gyro kalibriert.
- Failsafe, Gyro-Ausfall, Absturz der Software (HardFault, Assert, Stack-Überlauf) und Watchdog
  schalten die Motoren aus.

## Hinweis zur Eigenleistung

Teile dieses Codes wurden mit KI-Unterstützung (Claude, Anthropic) erstellt.
Das ist in der Redlichkeitserklärung der Maturaarbeit zu deklarieren.

## Lizenzen

FreeRTOS-Kernel: MIT. CMSIS (Arm / STMicroelectronics): Apache-2.0.
Eigener Code: **[Lizenz festlegen]**. Betaflight (GPL-3.0) diente nur als Referenz: übernommen wurden Zahlenwerte
(Pinbelegung, Registerwerte, Mischertabelle, PID-Skalierungsfaktoren), kein Programmcode.
