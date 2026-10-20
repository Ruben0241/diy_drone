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
| Takt 216 MHz (Quarz 8 MHz → PLL, Over-Drive; Notbetrieb mit HSI) | `src/board/system.c` | fertig |
| ICM-42688-P über SPI3, 1 kHz, Data-Ready-Interrupt | `src/board/icm42688.c` | fertig |
| CRSF-Empfang über UART2 (420 000 Baud) | `src/board/rx_uart.c`, `src/core/crsf.c` | fertig |
| DShot300 auf 4 Motoren (TIM3 + DMA-Burst) | `src/board/dshot_tim3.c`, `src/core/dshot.c` | fertig |
| Lageschätzung Mahony | `src/core/mahony.c` | fertig |
| PID je Achse (Betaflight-Skalierung, D auf Istwert, Feedforward, I-Term-Relax, Anti-Windup, TPA) | `src/core/pid.c` | fertig |
| Filter: PT1, Biquad-Tiefpass, Notch | `src/core/biquad.c`, `src/core/pt1.c` | fertig |
| Stick-Kennlinie (Betaflight «Actual Rates») | `src/core/rates.c` | fertig |
| Airmode, Runaway-Schutz, Motortest, ACC-Kalibrierung | `src/core/flight.c` | fertig |
| Einstellungen im Flash (Text + CRC), Befehle get/set/save/diff | `src/core/params.c`, `src/app/config_store.c` | fertig |
| Simulation: Firmware-Kern im Physikmodell | `tools/sim` ([README](tools/sim/README.md)) | fertig |
| Quad-X-Mischer mit Desaturierung | `src/core/mixer.c` | fertig |
| Rate- und Angle-Mode, Arming, Failsafe, Gyro-Kalibrierung | `src/core/flight.c` | fertig |
| Watchdog, Fehler-Handler (Motoren aus) | `src/board/system.c`, `src/app/main.c` | fertig |
| Akkuspannung, LEDs, Summer | `src/board/peripherals.c` | fertig, Werte prüfen |
| Logging in den 16-MB-Flash (W25Q128, SPI2), 1 kHz, nur armed | `src/core/blackbox.c`, `src/core/logrec.c`, `src/board/flash_w25q.c` | fertig |
| USB (virtueller COM-Port): info, live, dump, erase, dfu | `src/app/usb_cli.c` (TinyUSB) | fertig |
| PC-Werkzeug: Download → CSV, Live-Anzeige, DFU | `tools/fc_cli.py`, `tools/analysis/buildb_log.py` | fertig |
| Bidirektionales DShot / RPM-Filter | – | optional |

## Aufbau

```
src/core/    hardwareunabhängig, auf dem PC testbar (PID, Mahony, CRSF, Mischer, DShot, Flugsteuerung)
src/board/   Register-Treiber für den GEPRC F722 (Pins in board.h)
src/app/     FreeRTOS-Tasks (main.c) und FreeRTOSConfig.h
tests/       Host-Tests der Kernmodule
lib/         FreeRTOS-Kernel, CMSIS, TinyUSB (Git-Submodule)
docs/        Bodentest
tools/fc_cli.py  USB-Werkzeug (Download, Löschen, Live, DFU)
tools/analysis/  Python-Auswertung A/B (Sprungantwort, Kennzahlen, Vergleich), Log-Decoder
tools/sim/       Simulation (Software-in-the-Loop) und Aliasing-Demo
docs/            Bodentest, Abgleich mit Betaflight/INAV/PX4/dRehmFlight
```

### Tasks (Begleitarbeit Kap. 2.4)

| Task | Priorität | Takt | Aufgabe |
|---|---|---|---|
| control | 4 | Gyro Data-Ready (1 kHz) | IMU lesen, Lage, PID, Mischer, DShot |
| rx | 3 | 1 kHz | CRSF auswerten, Zeitstempel für Failsafe |
| log | 2 | Queue | Flugdaten in den Flash (nur armed) |
| usbd | 2 | USB-Ereignisse | TinyUSB |
| cli | 1 | Eingabe | Befehle über USB |
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
| Blackbox-Flash | W25Q128, SPI2 (PB13/PB14/PB15), CS PB12 |
| USB | PA11/PA12 (OTG_FS), Takt aus Quarz 8 MHz |

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
4. Nächstes Mal ohne Boot-Taste: `python tools/fc_cli.py dfu` startet den Bootloader,
   danach wie oben flashen. Falls die Firmware nicht startet: Boot-Taste gedrückt halten
   und USB einstecken **[prüfen: Lage der Boot-Taste am Board]**.

**Zurück zu Betaflight:** Firmware Flasher → Board `GEPRCF722` → Flashen → im CLI die
gespeicherte `diff all` einfügen → `save`.

## Flugdaten herunterladen (Build B)

```bash
pip install -r tools/requirements.txt
python tools/fc_cli.py info                      # Belegung des Flash
python tools/fc_cli.py dump --prefix 20261102    # -> logs/20261102_B_01.csv, _02.csv, ... + .json
python tools/fc_cli.py erase                     # erst nach erfolgreichem dump!
python tools/analysis/fc_analysis.py --a flugA.csv --b logs/20261102_B_01.csv --out ergebnisse
```

- Aufgezeichnet wird nur **armed** (Kopfsatz mit Firmware-Version und PID-Werten pro Flug),
  mit 1 kHz, 32 Byte pro Zyklus → 16 MB reichen für **ca. 8.7 min** Flugzeit.
  Ist der Speicher voll, piepst es beim Armen 3×; geflogen werden kann trotzdem.
- Pro Flug entstehen eine CSV (Spalten wie `tools/analysis` erwartet) und eine JSON-Datei
  mit den Einstellungen. Die Rohdaten werden zusätzlich als `.bin` gesichert.
- Während `dump` und `erase` ist Armen gesperrt; im armed-Zustand verweigert die Firmware beides.
- `python tools/fc_cli.py live` zeigt Lage, Gyro, Sticks und Akku 20× pro Sekunde
  (hilfreich für den Bodentest, ganz ohne Motoren).

Format der Datensätze: `src/core/logrec.h`.

## Einstellungen

Standardwerte in `flight_default_config()` (`src/core/flight.c`), Namen wie in Betaflight
(Liste: `src/core/params.c`). Ändern **ohne neu zu kompilieren** über USB:

```
python tools/fc_cli.py term
> get pid          # alle Einstellungen, die mit "pid" beginnen (leer: alle)
> set p_roll 50
> diff             # nur Abweichungen vom Standard
> save             # in den Flash (letzte 64 KB), nur disarmed
> defaults         # zurück auf Standard (danach save)
```

| Parameter | Standard | Bemerkung |
|---|---|---|
| `p/i/d/f_roll` usw. | 45/80/30/0, 47/84/34/0, 45/80/0/0 | Betaflight-Werte, gleiche Skalierung; F aus |
| `iterm_relax`, `tpa_rate` | aus | Betaflight-Standard: an; für Vergleich 1 aus |
| `iterm_windup` | 80 % | I-Grenze 400 (Roll/Pitch), 320 (Yaw) wie Betaflight |
| `roll_center` / `roll_max` / `roll_expo` | 400 / 400 °/s / 0 | linear, wie Kap. 3.4 |
| `gyro_lpf1_hz` / `dterm_lpf1_hz` / `dterm_lpf2_hz` | 150 / 75 / 150 Hz | PT1, statisch |
| `yaw_lowpass_hz` | 100 Hz | wie Betaflight |
| `motor_idle` | 5.5 % | wie Betaflight `motor_idle` 550 |
| `airmode_start` | 25 % | wie Betaflight |
| `runaway_prevention` | an | Werte aus Betaflight `core.c` |
| `gyro_sign_*`, `stick_sign_*` | +1 (Yaw-Stick −1) | Korrektur nach Bodentest |
| Failsafe | 250 ms ohne Paket | Motoren sofort aus |
| Kanäle | 1 Roll, 2 Pitch, 3 Gas, 4 Yaw, 5 Arm, 6 Angle-Mode | ELRS/EdgeTX «AETR» |

Weitere Befehle: `motor 1 10` (Motor 1 mit 10 %, nur disarmed, Propeller ab!), `motor stop`,
`acccal` (Drohne waagrecht hinstellen).

**Unterschiede zu Betaflight** und was bewusst fehlt: [docs/vergleich_software.md](docs/vergleich_software.md)
(dort auch die CLI-Liste, um Betaflight für den Vergleich auf «reinen PID» zu stellen).

## Sicherheit

- Erste Tests ohne Propeller, danach gefesselt (Kap. 3.4.7).
- Arming nur mit Gas unten, Schalter-Wechsel aus → ein, Drohne flach (< 25°) und Gyro kalibriert.
- Failsafe, Gyro-Ausfall, Absturz der Software (HardFault, Assert, Stack-Überlauf) und Watchdog
  schalten die Motoren aus. Nach einem Gyro-Ausfall muss der Arm-Schalter neu betätigt werden.
- Piepsen beim Start: 2× = Quarz nicht angelaufen (USB unzuverlässig), 4× = kein Flash gefunden,
  dauernd kurz alle 0.5 s = Gyro nicht gefunden oder Software-Fehler.

## Hinweis zur Eigenleistung

Teile dieses Codes wurden mit KI-Unterstützung (Claude, Anthropic) erstellt.
Das ist in der Redlichkeitserklärung der Maturaarbeit zu deklarieren.

## Lizenzen

FreeRTOS-Kernel: MIT. TinyUSB: MIT. CMSIS (Arm / STMicroelectronics): Apache-2.0.
USB-ID 1209:0001 ist die Test-ID von pid.codes (nur für Prototypen).
Eigener Code: **[Lizenz festlegen]**. Betaflight (GPL-3.0) diente nur als Referenz: übernommen wurden Zahlenwerte
(Pinbelegung, Registerwerte, Mischertabelle, PID-Skalierungsfaktoren), kein Programmcode.
