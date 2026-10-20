#!/usr/bin/env python3
"""
fc_cli.py - Verbindung zur eigenen Firmware ueber USB

    python tools/fc_cli.py info                     Zustand, Flash-Belegung
    python tools/fc_cli.py live                     Lage/Gyro/Sticks live (Bodentest), Ctrl+C beendet
    python tools/fc_cli.py dump --prefix 20261102   Fluege herunterladen -> CSV (+ .bin als Sicherung)
    python tools/fc_cli.py erase                    beschriebenen Bereich loeschen
    python tools/fc_cli.py erase --all              ganzen Flash loeschen (bis 200 s)
    python tools/fc_cli.py dfu                      Neustart in den Bootloader (zum Flashen)
    python tools/fc_cli.py term                     einfaches Terminal

Optionen: --port /dev/cu.usbmodemXXXX (sonst automatisch per USB-ID 1209:0001)
Benoetigt: pip install pyserial pandas numpy
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial fehlt: pip install pyserial")

sys.path.insert(0, str(Path(__file__).parent / "analysis"))

USB_VID, USB_PID = 0x1209, 0x0001


def find_port() -> str:
    ports = [p for p in list_ports.comports() if p.vid == USB_VID and p.pid == USB_PID]
    if not ports:
        found = ", ".join(f"{p.device} ({p.vid:04x}:{p.pid:04x})" if p.vid else p.device
                          for p in list_ports.comports())
        sys.exit(f"Flightcontroller nicht gefunden (USB 1209:0001). Vorhanden: {found or 'keine'}")
    return ports[0].device


def open_fc(port: str | None) -> serial.Serial:
    s = serial.Serial(port or find_port(), 115200, timeout=2)
    time.sleep(0.1)
    s.reset_input_buffer()
    return s


def command(s: serial.Serial, cmd: str) -> None:
    s.write((cmd + "\n").encode())
    s.flush()


def read_until_ok(s: serial.Serial, timeout: float = 5.0, echo: bool = True) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        line = s.readline().decode(errors="replace").strip()
        if not line:
            continue
        if echo:
            print(line)
        if line.startswith("OK"):
            return True
        if line.startswith("ERR"):
            return False
        end = time.time() + timeout
    print("Zeitueberschreitung")
    return False


def cmd_info(s, a):
    command(s, "info")
    time.sleep(0.3)
    print(s.read(s.in_waiting or 1).decode(errors="replace"), end="")


def cmd_status(s, a):
    command(s, "status")
    print(s.readline().decode(errors="replace").strip())


def cmd_live(s, a):
    command(s, "live")
    try:
        while True:
            line = s.readline().decode(errors="replace").rstrip()
            if line:
                print(line)
    except KeyboardInterrupt:
        s.write(b"x")
        read_until_ok(s, echo=False)


def cmd_dump(s, a):
    command(s, "dump")
    head = s.readline().decode(errors="replace").strip()
    if not head.startswith("DUMP"):
        sys.exit(f"Unerwartete Antwort: {head}")
    total = int(head.split()[1])
    print(f"Lade {total / 1e6:.2f} MB ...")
    data = bytearray()
    t0 = time.time()
    s.timeout = 5
    while len(data) < total:
        chunk = s.read(min(65536, total - len(data)))
        if not chunk:
            sys.exit(f"Abbruch nach {len(data)} von {total} Byte")
        data += chunk
        rate = len(data) / max(time.time() - t0, 1e-3) / 1e3
        print(f"\r  {len(data) * 100 // max(total, 1):3d} %  ({rate:.0f} kB/s)", end="", flush=True)
    print()
    read_until_ok(s, echo=False)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    raw = out / f"{a.prefix}_B_roh.bin"
    raw.write_bytes(bytes(data))
    import buildb_log
    res = buildb_log.decode(bytes(data))
    paths = buildb_log.write_csvs(res, out, a.prefix)
    print(f"{len(res.flights)} Flug/Fluege, CRC-Fehler: {res.crc_errors}. Rohdaten: {raw}")
    for p in paths:
        print(" ", p)
    print("Danach loeschen mit: python tools/fc_cli.py erase")


def cmd_erase(s, a):
    if not a.yes:
        ans = input("Flugdaten auf dem Flightcontroller loeschen? Vorher 'dump' machen! [j/N] ")
        if ans.strip().lower() not in ("j", "ja", "y", "yes"):
            return
    command(s, "erase all" if a.all else "erase")
    read_until_ok(s, timeout=250 if a.all else 30)


def cmd_simple(name):
    def run(s, a):
        command(s, name)
        read_until_ok(s)
        if name == "dfu":
            print("Board ist jetzt im DFU-Modus: mit Betaflight Configurator oder dfu-util flashen.")
    return run


def cmd_term(s, a):
    import threading

    def reader():
        while True:
            d = s.read(s.in_waiting or 1)
            if d:
                sys.stdout.write(d.decode(errors="replace"))
                sys.stdout.flush()
    threading.Thread(target=reader, daemon=True).start()
    print("Terminal (Ctrl+C beendet). 'help' fuer Befehle.")
    try:
        for line in sys.stdin:
            command(s, line.strip())
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description="USB-Werkzeug fuer die eigene Flugsteuerung")
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("status")
    sub.add_parser("live")
    d = sub.add_parser("dump")
    d.add_argument("--out", default="logs")
    d.add_argument("--prefix", default=time.strftime("%Y%m%d"))
    e = sub.add_parser("erase")
    e.add_argument("--all", action="store_true")
    e.add_argument("--yes", action="store_true")
    sub.add_parser("dfu")
    sub.add_parser("reboot")
    sub.add_parser("term")
    a = ap.parse_args()

    handlers = {
        "info": cmd_info, "status": cmd_status,
        "live": cmd_live, "dump": cmd_dump, "erase": cmd_erase,
        "dfu": cmd_simple("dfu"), "reboot": cmd_simple("reboot"), "term": cmd_term,
    }
    with open_fc(a.port) as s:
        handlers[a.cmd](s, a)


if __name__ == "__main__":
    main()
