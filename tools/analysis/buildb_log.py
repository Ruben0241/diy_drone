"""
buildb_log.py - Binaerlog der eigenen Firmware (Build B) entschluesseln

Format: siehe src/core/logrec.h (32-Byte-Datensaetze, CRC8 Polynom 0xD5).
Jeder Kopfsatz (0x5A) beginnt einen neuen Flug. Ergebnis pro Flug: ein DataFrame
mit den Spalten, die fc_analysis.load_buildb_csv erwartet, plus Metadaten.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

REC = 32
TYPE_DATA, TYPE_HEADER, TYPE_PAD, TYPE_ERASED = 0xA5, 0x5A, 0x00, 0xFF
DATA_STRUCT = struct.Struct("<BBI3h3h4HHHB")      # ohne CRC (31 Byte)
HEADER_STRUCT = struct.Struct("<BBIHH9BHBB")       # 23 Byte (Version 1)
HEADER_V2_EXTRA = struct.Struct("<3HB")             # Byte 23..29: Feedforward, Flags (Version 2)


def crc8_d5(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


_CRC_TABLE = [crc8_d5(bytes([i])) for i in range(256)]


def crc8_fast(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc


@dataclass
class Flight:
    index: int
    header: dict
    rows: list = field(default_factory=list)

    def to_dataframe(self) -> pd.DataFrame:
        cols = ["t_us", "flags", "sp_roll", "sp_pitch", "sp_yaw", "gyro_roll", "gyro_pitch",
                "gyro_yaw", "m1", "m2", "m3", "m4", "throttle", "vbat", "lq"]
        df = pd.DataFrame(self.rows, columns=cols)
        for c in ("sp_roll", "sp_pitch", "sp_yaw", "gyro_roll", "gyro_pitch", "gyro_yaw"):
            df[c] = df[c] / 10.0
        df["throttle"] = df["throttle"] / 10000.0
        df["vbat"] = df["vbat"] / 1000.0
        return df[["t_us", "sp_roll", "sp_pitch", "sp_yaw", "gyro_roll", "gyro_pitch", "gyro_yaw",
                   "throttle", "m1", "m2", "m3", "m4", "flags", "vbat", "lq"]]


@dataclass
class DecodeResult:
    flights: list
    crc_errors: int
    unknown: int
    bytes_used: int


def decode(data: bytes) -> DecodeResult:
    flights: list[Flight] = []
    crc_err = unknown = 0
    current: Flight | None = None
    end = len(data) - len(data) % REC
    pos = 0
    for pos in range(0, end, REC):
        rec = data[pos:pos + REC]
        t = rec[0]
        if t == TYPE_ERASED and rec == b"\xff" * REC:
            break                                        # Ende der Daten
        if crc8_fast(rec[:31]) != rec[31]:
            crc_err += 1
            continue
        if t == TYPE_PAD:
            continue
        if t == TYPE_HEADER:
            v = HEADER_STRUCT.unpack_from(rec)
            pid = v[5:14]
            header = {
                "format_version": v[1],
                "git_hash": f"{v[2]:08x}",
                "loop_hz": v[3],
                "log_hz": v[4],
                "pid": {"roll": pid[0:3], "pitch": pid[3:6], "yaw": pid[6:9]},
                "max_rate_dps": v[14],
                "gyro_lpf_hz": v[15],
                "dterm_lpf_hz": v[16],
            }
            if v[1] >= 2:
                e = HEADER_V2_EXTRA.unpack_from(rec, 23)
                header["ff"] = {"roll": e[0], "pitch": e[1], "yaw": e[2]}
                header["iterm_relax"] = bool(e[3] & 1)
                header["tpa"] = bool(e[3] & 2)
                header["runaway_prevention"] = bool(e[3] & 4)
            current = Flight(index=len(flights) + 1, header=header)
            flights.append(current)
            continue
        if t == TYPE_DATA:
            if current is None:                          # Daten ohne Kopf (sollte nicht vorkommen)
                current = Flight(index=len(flights) + 1, header={})
                flights.append(current)
            v = DATA_STRUCT.unpack_from(rec)
            # Reihenfolge wie Flight.to_dataframe: t, flags, sp*3, gyro*3, m*4, thr, vbat, lq
            current.rows.append((v[2], v[1], *v[3:6], *v[6:9], *v[9:13], v[13], v[14], v[15]))
            continue
        unknown += 1
    return DecodeResult(flights=flights, crc_errors=crc_err, unknown=unknown, bytes_used=pos)


def write_csvs(result: DecodeResult, out_dir: str | Path, prefix: str) -> list[Path]:
    """Ein CSV pro Flug: <prefix>_B_<NN>.csv, dazu <prefix>_B_<NN>.json mit den Einstellungen."""
    import json
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    for f in result.flights:
        df = f.to_dataframe()
        if df.empty:
            continue
        name = f"{prefix}_B_{f.index:02d}"
        p = out / f"{name}.csv"
        df.to_csv(p, index=False)
        meta = dict(f.header)
        meta["samples"] = int(len(df))
        meta["duration_s"] = float((df.t_us.iloc[-1] - df.t_us.iloc[0]) % (1 << 32) / 1e6)
        (out / f"{name}.json").write_text(json.dumps(meta, indent=2))
        paths.append(p)
    return paths


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description="Binaerlog (Build B) in CSV umwandeln")
    ap.add_argument("bin", help="Datei aus 'fc_cli.py dump'")
    ap.add_argument("--out", default=".")
    ap.add_argument("--prefix", default="flug")
    a = ap.parse_args()
    res = decode(Path(a.bin).read_bytes())
    paths = write_csvs(res, a.out, a.prefix)
    print(f"{len(res.flights)} Flug/Fluege, CRC-Fehler: {res.crc_errors}")
    for p in paths:
        print(" ", p)


if __name__ == "__main__":
    main()
