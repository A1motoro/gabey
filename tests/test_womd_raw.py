"""Optional raw-reader CRC tests; tiny in-memory records, no downloaded data."""

import struct

import pytest

from pbs.data.womd import _masked_crc, iter_tfrecords

pytest.importorskip("google_crc32c", reason="Optional raw WOMD dependency")


def test_crc32c_known_vector_and_both_record_checks(tmp_path):
    crc = 0xE3069283  # published CRC32C check value for ASCII '123456789'
    expected = (((crc >> 15) | (crc << 17)) + 0xA282EAD8) & 0xFFFFFFFF
    assert _masked_crc(b"123456789") == expected
    payload = b"a real TFRecord envelope, not a protobuf"
    length = struct.pack("<Q", len(payload))
    data = length + struct.pack("<I", _masked_crc(length)) + payload
    data += struct.pack("<I", _masked_crc(payload))
    path = tmp_path / "test.bin"
    path.write_bytes(data)
    assert list(iter_tfrecords(path)) == [payload]
    corrupted = bytearray(data)
    corrupted[8] ^= 1
    path.write_bytes(corrupted)
    with pytest.raises(ValueError, match="Length CRC32C"):
        list(iter_tfrecords(path))
    corrupted = bytearray(data)
    corrupted[12] ^= 1
    path.write_bytes(corrupted)
    with pytest.raises(ValueError, match="Payload CRC32C"):
        list(iter_tfrecords(path))
