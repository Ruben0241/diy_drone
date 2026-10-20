"""
Tests fuer den Decoder des Binaerlogs (Build B).

Prueft zwei Richtungen:
  * Datei, die der C-Code der Firmware erzeugt hat (build-tests/sample_log.bin, aus den Host-Tests)
  * selbst gebaute Datensaetze mit Fehlern (CRC, Ende, mehrere Fluege)

    python tools/analysis/test_buildb_log.py
"""
from pathlib import Path
import struct
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).parent))
import buildb_log as bl  # noqa: E402
import fc_analysis as fa  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
SAMPLE = ROOT / "build-tests" / "sample_log.bin"


def rec(body: bytes) -> bytes:
    body = body.ljust(31, b"\0")
    return body + bytes([bl.crc8_d5(body)])


def data_rec(t, sp_roll=0, gyro_roll=0, flags=1, thr=4000):
    return rec(bl.DATA_STRUCT.pack(0xA5, flags, t, sp_roll, 0, 0, gyro_roll, 0, 0, 48, 48, 48, 48, thr, 16000, 100))


def header_rec():
    return rec(bl.HEADER_STRUCT.pack(0x5A, 1, 0x1234ABCD, 1000, 1000, 45, 80, 30, 47, 84, 34, 45, 80, 0, 400, 150, 75))


def test_crc_matches_crsf_example():
    # Beispielpaket aus dem CRSF-Wiki (gleiches Polynom wie die Firmware)
    frame = bytes.fromhex("16e0031ff8c0073ef0810f7ce0031ff8c0073ef0810f7c")
    assert bl.crc8_d5(frame) == 0xAD
    assert bl.crc8_fast(frame) == 0xAD


def test_sample_from_firmware_code():
    if not SAMPLE.exists():
        print("  (uebersprungen: zuerst Host-Tests laufen lassen)")
        return
    res = bl.decode(SAMPLE.read_bytes())
    assert res.crc_errors == 0
    assert len(res.flights) == 1
    f = res.flights[0]
    assert f.header["git_hash"] == "deadbeef"
    assert f.header["pid"]["roll"] == (45, 80, 30)
    assert f.header["max_rate_dps"] == 400
    assert f.header["format_version"] == 2
    assert f.header["ff"] == {"roll": 120, "pitch": 125, "yaw": 0}
    assert f.header["iterm_relax"] and not f.header["tpa"] and f.header["runaway_prevention"]
    df = f.to_dataframe()
    assert len(df) == 20
    assert df.t_us.iloc[0] == 1_000_000 and df.t_us.iloc[19] == 1_019_000
    assert abs(df.sp_roll.iloc[19] - 190.0) < 1e-9
    assert abs(df.gyro_roll.iloc[19] - 180.5) < 1e-9
    assert abs(df.throttle.iloc[0] - 0.4321) < 1e-9
    assert abs(df.vbat.iloc[0] - 15.12) < 1e-9
    assert df.m3.iloc[0] == 2047 and df["flags"].iloc[0] == 5 and df.lq.iloc[0] == 99


def test_flights_crc_and_end():
    pad = rec(b"\x00")
    blob = header_rec() + b"".join(data_rec(i * 1000, sp_roll=i) for i in range(10)) + pad * 5
    bad = bytearray(data_rec(99999))
    bad[10] ^= 0xFF
    blob += header_rec() + bytes(bad) + data_rec(5) + pad * 6
    blob += b"\xff" * 64 + data_rec(1)          # nach dem Ende: wird ignoriert
    res = bl.decode(blob)
    assert len(res.flights) == 2
    assert len(res.flights[0].rows) == 10 and len(res.flights[1].rows) == 1
    assert res.crc_errors == 1


def test_csv_works_with_analysis():
    blob = header_rec() + b"".join(data_rec(i * 1000, sp_roll=i % 300, gyro_roll=i % 300)
                                   for i in range(3000))
    res = bl.decode(blob)
    with tempfile.TemporaryDirectory() as d:
        paths = bl.write_csvs(res, d, "test")
        assert len(paths) == 1 and paths[0].name == "test_B_01.csv"
        assert (Path(d) / "test_B_01.json").exists()
        df = fa.load_buildb_csv(paths[0])
        assert len(df) == 3000
        assert abs(df.t.iloc[-1] - 2.999) < 1e-9
        assert df.armed.all()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
