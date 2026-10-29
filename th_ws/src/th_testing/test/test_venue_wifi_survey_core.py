import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "venue_wifi"))

from wifi_survey_core import (aggregate_scans, band_for_frequency, channel_for_frequency,
                              classify, dbm_from_percent, filter_last_seen, format_summary,
                              parse_nmcli_rows, split_nmcli_t, summarize)


def row(ssid, bssid, chan, freq, pct=70, **extra):
    return {"ssid": ssid, "bssid": bssid, "channel": chan, "frequency_mhz": freq,
            "strength_pct": pct, "security": "WPA2", **extra}


def test_nmcli_escape_and_unicode_fields():
    parsed = split_nmcli_t(r"a\:b\\c:90\:B8\:32\:98\:DF\:68:1:2412:85:54 Mbit/s:WPA2:/ap")
    assert parsed == [r"a:b\c", "90:B8:32:98:DF:68", "1", "2412", "85", "54 Mbit/s", "WPA2", "/ap"]
    rows = parse_nmcli_rows(r"日本語:00\:11\:22\:33\:44\:55:6:2437 MHz:72:54 Mbit/s:WPA2:/ap" + "\n")
    assert rows[0]["ssid"] == "日本語"
    assert parse_nmcli_rows(r":00\:11\:22\:33\:44\:55:1:2412:50:54 Mbit/s:--:/ap")[0]["ssid"] == ""


def test_band_uses_frequency_and_boundaries():
    assert band_for_frequency(5975) == "6 GHz"
    assert channel_for_frequency(5975) == 5
    assert band_for_frequency(2499) == "2.4 GHz"
    assert band_for_frequency(2500) is None
    assert band_for_frequency(5925) == "6 GHz"
    assert channel_for_frequency(2484) == 14
    assert channel_for_frequency(2412) == 1
    assert channel_for_frequency(2437) == 6
    assert channel_for_frequency(2462) == 11
    # Same channel number as 2.4 GHz is still 6 GHz when FREQ says so.
    classified = classify(row("six-ghz", "six", 5, 5975))
    assert classified["band"] == "6 GHz" and classified["channel"] == 5


def test_last_seen_and_vehicle_ssids():
    rows = [classify(row("th-rpi-ap", "01", 1, 2412)), classify(row("th-rpi-5g", "02", 48, 5240)),
            classify(row("th-rpi-ap2", "03", 1, 2412))]
    assert [r["vehicle_ap"] for r in rows] == [True, True, False]
    assert filter_last_seen([{"id": 1, "last_seen": 100}, {"id": 2, "last_seen": 99}], 100) == [{"id": 1, "last_seen": 100}]
    assert summarize(rows)["two_total"] == 1
    assert len(summarize(rows)["vehicle"]) == 2


def test_bucket_edges_and_dbm_conversion():
    rows = aggregate_scans([[row("a", "a", 3, 2422), row("b", "b", 4, 2427),
                             row("c", "c", 8, 2447), row("d", "d", 9, 2452),
                             row("e", "e", 13, 2472), row("f", "f", 14, 2484),
                             row("g", "g", 48, 5240), row("h", "h", 52, 5260)]])
    s = summarize(rows)
    assert s["near"] == [1, 2, 3]
    assert s["five_w52"] == 1 and s["five_other"] == 1
    assert dbm_from_percent(85) == -49
    assert dbm_from_percent(72) == -57
    assert dbm_from_percent(100) == -40
    assert dbm_from_percent(0) == -100


def test_repeated_scans_keep_maximum_and_count_once_per_scan():
    rows = aggregate_scans([[row("x", "aa", 1, 2412, 60), row("x", "aa", 1, 2412, 65)],
                            [row("x", "aa", 1, 2412, 85)], []])
    assert rows[0]["strength_pct"] == 85
    assert rows[0]["seen_scans"] == 2
    assert rows[0]["scan_count"] == 3


def test_channel_load_overlap_rules():
    from wifi_survey_core import channel_load
    rows = aggregate_scans([[row("one", "z", 1, 2412), row("five", "a", 5, 2432, 80), row("six", "b", 6, 2437),
                             row("44", "c", 44, 5220), row("48", "d", 48, 5240),
                             row("36", "e", 36, 5180), row("40", "f", 40, 5200)]])
    loads = channel_load(rows)
    ch1 = loads["2.4 GHz"][0]
    ch6 = loads["2.4 GHz"][1]
    assert len(ch1["overlap"]) == 1 and ch1["overlap"][0]["channel"] == 5
    assert not any(r["channel"] == 1 for r in ch6["overlap"])
    assert len(loads["5 GHz"][0]["stations"]) == 2
    assert len(loads["5 GHz"][1]["stations"]) == 2


def test_output_contains_sheet_columns():
    rows = aggregate_scans([[row("a", "a", 1, 2412), row("b", "b", 48, 5240)]])
    text = format_summary("保管場所", "2026-11-05 10:12", "wlo1", 1, 1, 22, rows)
    for expected in ("ch1 付近", "ch6 付近", "ch11 付近", "合計", "ch36〜48", "その他", "最も強い電波"):
        assert expected in text


def test_csv_roundtrip_summary_preserves_counts(tmp_path):
    # Exercise the same CSV field representation used by the CLI.
    from wifi_survey_core import CSV_FIELDS
    samples = [aggregate_scans([[row("private", "aa", 1, 2412), row("private", "bb", 48, 5240)]]),
               aggregate_scans([[row("private", "cc", 6, 2437)]])]
    path = tmp_path / "sample.csv"
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for location, rows in (("保管場所", samples[0]), ("試験場", samples[1])):
            for r in rows:
                writer.writerow({"location": location, "started": "2026-11-05T10:12:00+09:00", "ifname": "wlo1",
                                 "band": r["band"], "channel": r["channel"], "frequency_mhz": r["frequency_mhz"],
                                 "strength_pct": r["strength_pct"], "dbm": r["dbm"], "seen_scans": r["seen_scans"],
                                 "scan_count": r["scan_count"], "ssid": r["ssid"], "bssid": r["bssid"],
                                 "scans_completed": 1, "elapsed_seconds": 3,
                                 "security": r.get("security", ""), "vehicle_ap": str(r["vehicle_ap"]).lower()})
    with path.open(encoding="utf-8", newline="") as stream:
        loaded = list(csv.DictReader(stream))
    assert len(loaded) == 3
    by_location = {name: [x for x in loaded if x["location"] == name] for name in ("保管場所", "試験場")}
    assert summarize([classify({**x, "channel": int(x["channel"]), "frequency_mhz": int(x["frequency_mhz"]),
                                "strength_pct": int(x["strength_pct"]), "vehicle_ap": x["vehicle_ap"] == "true"})
                      for x in by_location["保管場所"]])["two_total"] == 1
    assert len(by_location["試験場"]) == 1
