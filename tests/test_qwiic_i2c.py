"""Qwiic / STEMMA QT jack (J2, arena_12-18) — I2C bridge commands 0xB0/0xB1
and the LAB-211 sensors behind them.

Hardware-adaptive: the bus-level tests need only the jack (they pass with
nothing plugged in); each sensor test skips unless that part answers on the
root bus or behind a PCA9548 channel. On an arena_10-10 build (no jack) the
whole module skips.

Run:
    pixi run test-serial -- tests/test_qwiic_i2c.py -s
"""

import pytest

from .commands import I2C_TRANSFER_CMD
from .qwiic_sensors import (
    AS7343,
    MCP4725_ADDR,
    PCA9548,
    TSL2591,
    VEML7700,
    I2CError,
    NoQwiicJack,
    i2c_scan,
    i2c_xfer,
    is_mux,
    locate_devices,
    reach,
)


@pytest.fixture(scope="module")
def scan(transport):
    try:
        return i2c_scan(transport)
    except NoQwiicJack as e:
        pytest.skip(str(e))


@pytest.fixture(scope="module")
def devices(transport, scan):
    return locate_devices(transport)


def _find(devices, addr):
    hits = [d for d in devices if d.addr == addr]
    if not hits:
        pytest.skip(f"no device at 0x{addr:02X} on the Qwiic bus")
    return hits


# ── Bus level (no sensors required) ──────────────────────────────────────────

def test_scan_payload_shape(scan):
    assert scan == sorted(set(scan)), f"addresses must be ascending and unique: {scan}"
    assert all(0x08 <= a <= 0x77 for a in scan), f"reserved address in scan: {scan}"


def test_scan_does_not_see_the_ao_dac(scan):
    # The MCP4725 sits on Wire (D18/D19). Seeing it here would mean the
    # bridge is on the wrong bus.
    assert MCP4725_ADDR not in scan, "0x60 on the Qwiic bus: bridge is on Wire, not Wire1"


def test_transfer_rejects_bad_framing(transport, scan):
    st, _, payload, _ = transport.command(I2C_TRANSFER_CMD, bytes([0x10]))
    assert st == 1, f"too-short frame must be refused: {bytes(payload)!r}"
    st, _, payload, _ = transport.command(I2C_TRANSFER_CMD, bytes([0x10, 3, 0xAA, 0]))
    assert st == 1, f"wlen mismatch must be refused: {bytes(payload)!r}"
    st, _, payload, _ = transport.command(I2C_TRANSFER_CMD, bytes([0x10, 0, 65]))
    assert st == 1, f"rlen > 64 must be refused: {bytes(payload)!r}"
    st, _, payload, _ = transport.command(I2C_TRANSFER_CMD, bytes([0x80, 0, 0]))
    assert st == 1, f"8-bit address must be refused: {bytes(payload)!r}"


def test_transfer_nacks_an_empty_address(transport, scan):
    empty = next(a for a in range(0x08, 0x78) if a not in scan)
    with pytest.raises(I2CError) as e:
        i2c_xfer(transport, empty, b"\x00", 0)
    assert e.value.status == 2, "write to an empty address must report address NACK"
    with pytest.raises(I2CError) as e:
        i2c_xfer(transport, empty, b"", 1)
    assert e.value.status == 2, "read from an empty address must report address NACK"
    with pytest.raises(I2CError) as e:
        i2c_xfer(transport, empty)
    assert e.value.status == 2, "ACK probe of an empty address must report NACK"


def test_ack_probe_matches_scan(transport, scan):
    for addr in scan:
        assert i2c_xfer(transport, addr) == b"", f"0x{addr:02X} ACKed the scan but not a probe"


# ── PCA9548 mux ──────────────────────────────────────────────────────────────

def test_mux_control_register_roundtrip(transport, scan):
    muxes = [a for a in scan if is_mux(a)]
    if not muxes:
        pytest.skip("no PCA9548 on the bus")
    mux = PCA9548(transport, muxes[0])
    previous = mux.read_mask()
    try:
        for mask in (0x00, 0x01, 0x80, 0xA5):
            mux.select(mask)
            assert mux.read_mask() == mask, f"mux mask 0x{mask:02X} did not read back"
    finally:
        mux.select(previous)
    assert mux.read_mask() == previous


def test_mux_channel_map_is_reported(transport, scan, devices):
    if not any(is_mux(a) for a in scan):
        pytest.skip("no PCA9548 on the bus")
    downstream = [d for d in devices if d.mux is not None]
    print("\nmux channel map:", ", ".join(f"ch{d.channel}:0x{d.addr:02X}" for d in downstream) or "empty")


# ── Sensors ──────────────────────────────────────────────────────────────────

def test_tsl2591_identify_and_read(transport, devices):
    for dev in _find(devices, TSL2591.ADDR):
        with reach(transport, dev):
            s = TSL2591(transport)
            ident = s.identify()
            assert ident["id_ok"], f"TSL2591 @ {dev.where()}: ID 0x{ident['id']:02X} != 0x50"
            s.configure(gain="med", atime_ms=100)
            try:
                r = s.read()
            finally:
                s.power_off()
            print(f"\nTSL2591 @ {dev.where()}: {r}")
            assert r["valid"], "AVALID not set after one integration window"
            assert r["full"] >= r["ir"], "CH0 (full) must be >= CH1 (IR)"


def test_veml7700_config_roundtrip_and_read(transport, devices):
    for dev in _find(devices, VEML7700.ADDR):
        with reach(transport, dev):
            s = VEML7700(transport)
            ident = s.identify()
            print(f"\nVEML7700 @ {dev.where()}: id_raw=0x{ident['id_raw']:04X}"
                  f" (device_id 0x{ident['device_id']:02X}, expect 0x81 on current silicon)")
            s.write16(s.REG_CONF, s.CONF_GAIN1_IT200)
            assert s.read16(s.REG_CONF) == s.CONF_GAIN1_IT200, "ALS_CONF_0 did not read back"
            s.configure(s.CONF_GAIN1_IT100)
            assert s.read16(s.REG_CONF) == s.CONF_GAIN1_IT100
            r = s.read()
            print(f"VEML7700 @ {dev.where()}: {r}")
            assert 0 <= r["als"] <= 0xFFFF and 0 <= r["white"] <= 0xFFFF


def test_as7343_identify_and_enable_roundtrip(transport, devices):
    for dev in _find(devices, AS7343.ADDR):
        with reach(transport, dev):
            s = AS7343(transport)
            ident = s.identify()
            print(f"\nAS7343 @ {dev.where()}: id=0x{ident['id']:02X} rev=0x{ident['revid']:02X}"
                  f" aux=0x{ident['auxid']:02X}")
            assert ident["id_ok"], f"AS7343 ID 0x{ident['id']:02X} != 0x81 (check CFG0.REG_BANK handling)"
            assert s.enable_roundtrip(), "ENABLE.PON did not read back"


def test_as7343_spectral_read(transport, devices):
    for dev in _find(devices, AS7343.ADDR):
        with reach(transport, dev):
            s = AS7343(transport)
            s.configure(gain=64)
            try:
                r = s.read()
            finally:
                s.power_off()
            print(f"\nAS7343 @ {dev.where()} gain 64 tint {r['tint_ms']:.0f} ms sat={r['sat']}: "
                  + " ".join(f"{k}={v}" for k, v in r["spectral"].items())
                  + f" VIS={r['vis']:.0f} FD={r['fd']:.0f}")
            assert len(r["channels"]) == 18
            assert all(0 <= v <= r["full_scale"] for v in r["channels"].values()), r["channels"]
            # The clear channel is re-read in each of the 3 SMUX cycles; those
            # repeats must agree (a mis-ordered channel map would not).
            reps = [v for k, v in r["channels"].items() if k.startswith("VIS")]
            assert max(reps) - min(reps) <= max(20, 0.05 * max(reps)), (
                f"VIS repeats disagree across SMUX cycles: {reps}")
