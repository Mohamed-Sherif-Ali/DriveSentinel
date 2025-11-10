import pc_v2.serial_protocol as sp

def test_checksum_roundtrip():
    line = "CMD:MODE:ON"
    with_sum = sp.append_checksum(line)
    data = sp.validate_and_strip_checksum(with_sum)
    assert data == line

def test_parse_dist():
    obj = sp.parse_line("DIST:F=123 R=88 B=999")
    assert obj["type"] == "DIST" and obj["F"] == "123" and obj["R"] == "88" and obj["B"] == "999"
