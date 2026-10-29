import csv
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "venue_wifi"))

from wifi_survey_core import (aggregate_scans, band_for_frequency, channel_for_frequency,
                              classify, collect_scans, dbm_from_percent, filter_last_seen,
                              format_summary, parse_nmcli_rows,
                              split_nmcli_t, summarize)


def row(ssid, bssid, chan, freq, pct=70, **extra):
    return {"ssid": ssid, "bssid": bssid, "channel": chan, "frequency_mhz": freq,
            "strength_pct": pct, "security": "WPA2", **extra}


def install_fake_cli(monkeypatch, advance_scans=True):
    import wifi_survey
    calls = []
    scan_value = [100]
    access_points = {
        "/org/freedesktop/NetworkManager/AccessPoint/1": 9,
        "/org/freedesktop/NetworkManager/AccessPoint/2": 10,
        "/org/freedesktop/NetworkManager/AccessPoint/3": 10,
    }
    listing = ("stale-only:AA\\:BB\\:CC\\:DD\\:EE\\:01:1:2412 MHz:61:54 Mbit/s:WPA2:/org/freedesktop/NetworkManager/AccessPoint/1\n"
               "fresh-only:AA\\:BB\\:CC\\:DD\\:EE\\:02:6:2437 MHz:75:54 Mbit/s:WPA2:/org/freedesktop/NetworkManager/AccessPoint/2\n"
               "th-rpi-ap:AA\\:BB\\:CC\\:DD\\:EE\\:03:1:2412 MHz:65:54 Mbit/s:WPA2:/org/freedesktop/NetworkManager/AccessPoint/3\n")

    def fake_run(command, env=None):
        calls.append((command, env))
        if command[0] == "nmcli":
            if command[1:3] == ["-t", "-f"] and command[3] == "DEVICE,TYPE":
                return SimpleNamespace(returncode=0, stdout="wlo1:wifi\n", stderr="")
            if command[1:3] == ["radio", "wifi"]:
                return SimpleNamespace(returncode=0, stdout="enabled\n", stderr="")
            if command[1:3] == ["-g", "GENERAL.DBUS-PATH"]:
                return SimpleNamespace(returncode=0, stdout="/org/freedesktop/NetworkManager/Devices/2\n", stderr="")
            if command[1:3] == ["dev", "wifi"] and command[3] == "rescan":
                if advance_scans:
                    scan_value[0] += 1
                return SimpleNamespace(returncode=0, stdout="", stderr="")
            if command[1:3] == ["-t", "-f"] and command[3].startswith("SSID,"):
                return SimpleNamespace(returncode=0, stdout=listing, stderr="")
            raise AssertionError(f"Unexpected nmcli command: {command}")
        if command[0] == "busctl":
            prop = command[-1]
            if prop == "LastScan":
                return SimpleNamespace(returncode=0, stdout=f"x {scan_value[0]}\n", stderr="")
            if prop == "LastSeen":
                return SimpleNamespace(returncode=0, stdout=f"i {access_points[command[4]]}\n", stderr="")
            raise AssertionError(f"Unexpected busctl property: {command}")
        raise AssertionError(f"Unexpected command: {command}")

    monkeypatch.setattr(wifi_survey, "run", fake_run)
    ticks = [0.0]
    monkeypatch.setattr(wifi_survey.time, "clock_gettime", lambda _: 10.75)
    monkeypatch.setattr(wifi_survey.time, "monotonic", lambda: ticks[0])
    monkeypatch.setattr(wifi_survey.time, "sleep", lambda seconds: ticks.__setitem__(0, ticks[0] + seconds))
    return wifi_survey, calls


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


def test_collect_scans_filters_each_round_and_tracks_scan_completion():
    now = [100.0]
    round_number = [0]
    last_scan = [0]
    request_ok = [True, True, False, True]
    should_advance = [True, True, False, False]
    samples = [
        [dict(row("old-ap", "aa", 1, 2412), last_seen=100)],
        [dict(row("old-ap", "aa", 1, 2412), last_seen=100), dict(row("new-ap", "bb", 6, 2437), last_seen=101)],
        [dict(row("new-ap", "bb", 6, 2437), last_seen=101)],
        [dict(row("new-ap", "bb", 6, 2437), last_seen=101)],
    ]

    def boottime():
        return 100 + round_number[0]

    def read_scan():
        if not read_scan.after_request:
            return last_scan[0]
        read_scan.after_request = False
        if should_advance[round_number[0] - 1]:
            last_scan[0] += 1
        return last_scan[0]
    read_scan.after_request = False

    def request():
        round_number[0] += 1
        read_scan.after_request = True
        return request_ok[round_number[0] - 1], "simulated refusal" if not request_ok[round_number[0] - 1] else ""

    def list_aps():
        return samples[round_number[0] - 1]

    def sleep(seconds):
        now[0] += seconds

    def monotonic():
        return now[0]

    result = collect_scans(4, request, read_scan, list_aps,
                           lambda ap: ap["last_seen"], boottime, monotonic,
                           sleep, timeout=1, interval=0.5)
    merged = aggregate_scans(result["scans"])
    by_bssid = {r["bssid"]: r for r in merged}
    assert result["completed"] == 2
    assert any("simulated refusal" in warning for warning in result["warnings"])
    assert any("4 回目" in warning for warning in result["warnings"])
    assert by_bssid["AA"]["seen_scans"] == 1
    assert by_bssid["AA"]["scan_count"] == 4
    assert by_bssid["BB"]["seen_scans"] == 1


def test_collect_scans_reports_zero_completions_but_still_reads_failed_request():
    listed = []
    result = collect_scans(1, lambda: (False, "denied"), lambda: 50,
                           lambda: listed.append(True) or [], lambda _: 50,
                           lambda: 50, lambda: 0, lambda _: None)
    assert result["completed"] == 0
    assert listed == [True]
    assert result["scans"] == [[]]
    assert "denied" in result["warnings"][0]


def test_collect_scans_warns_if_every_listed_ap_is_stale():
    result = collect_scans(1, lambda: (False, "denied"), lambda: 50,
                           lambda: [dict(row("old", "old", 1, 2412), last_seen=49)],
                           lambda ap: ap["last_seen"], lambda: 50,
                           lambda: 0, lambda _: None)
    assert result["scans"] == [[]]
    assert "一覧の AP がすべて測り始めより古い" in result["warnings"]


def test_main_wires_dbus_last_seen_into_collection_and_sets_nmcli_locale(tmp_path, monkeypatch, capsys):
    wifi_survey, calls = install_fake_cli(monkeypatch)
    monkeypatch.setattr(sys, "argv", ["wifi_survey.py", "保管場所", "--out", str(tmp_path), "--scans", "2"])
    assert wifi_survey.main() == 0
    output = capsys.readouterr().out
    csv_paths = list(tmp_path.glob("*.csv"))
    assert len(csv_paths) == 1
    assert "stale-only" not in output and "stale-only" not in csv_paths[0].read_text(encoding="utf-8-sig")
    assert "fresh-only" not in output
    assert "2.4 GHz の数   ch1 付近 0   ch6 付近 1" in output
    with csv_paths[0].open(encoding="utf-8-sig", newline="") as stream:
        saved = list(csv.DictReader(stream))
    ap_rows = {row["ssid"]: row for row in saved}
    assert set(ap_rows) == {"fresh-only", "th-rpi-ap"}
    assert ap_rows["th-rpi-ap"]["vehicle_ap"] == "true"
    assert all(env and env.get("LC_ALL") == "C" for command, env in calls if command[0] == "nmcli")


def test_main_does_not_save_when_lastscan_never_advances(tmp_path, monkeypatch, capsys):
    wifi_survey, _ = install_fake_cli(monkeypatch, advance_scans=False)
    monkeypatch.setattr(sys, "argv", ["wifi_survey.py", "保管場所", "--out", str(tmp_path), "--scans", "1"])
    assert wifi_survey.main() == 1
    assert list(tmp_path.glob("*.csv")) == []
    assert "スキャンが 1 回も完了しませんでした" in capsys.readouterr().err


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
    rows = aggregate_scans([[row("ch1", "a", 1, 2412, 70),
                             row("ch6a", "b", 6, 2437, 70), row("ch6b", "c", 6, 2437, 70),
                             row("ch11a", "d", 11, 2462, 70), row("ch11b", "e", 11, 2462, 70),
                             row("ch11c", "f", 11, 2462, 70), row("overlap", "g", 2, 2417, 80),
                             row("44", "h", 44, 5220, 70), row("48", "i", 48, 5240, 70),
                             row("36a", "j", 36, 5180, 75), row("36b", "k", 36, 5180, 72),
                             row("40", "l", 40, 5200, 71), row("other", "m", 149, 5745, 90),
                             row("six", "n", 5, 5975, 60), row("th-rpi-ap", "v", 1, 2412, 90)]])
    text = format_summary("保管場所", "2026-11-05 10:12", "wlo1", 1, 1, 22, rows)
    for expected in ("ch1 付近 2", "ch6 付近 2", "ch11 付近 3", "合計 7",
                     "ch36〜48 5", "その他 1", "約 −46 dBm（5 GHz ch 149）",
                     "ch 1（いま）  同じ ch 1・重なる ch 1",
                     "ch 6  同じ ch 2・重なる ch 1",
                     "ch 11  同じ ch 3・重なる ch 0",
                     "44+48（いま） 2 局", "36・40 の 3 局", "36+40 3 局"):
        assert expected in text, expected
    assert "6 GHz 1 件" in text


def test_csv_roundtrip_uses_production_writer_and_summary(tmp_path, capsys):
    import wifi_survey
    rows_a = aggregate_scans([[row("PRIVATE-SSID", "aa", 1, 2412), row("PRIVATE-SSID", "bb", 48, 5240),
                               row("th-rpi-ap", "ee", 1, 2412, 75)]])
    rows_b = aggregate_scans([[row("PRIVATE-SSID", "cc", 6, 2437)]])
    path_a = tmp_path / "2026-11-05_101200_保管場所.csv"
    path_b = tmp_path / "2026-11-05_101500_保管場所.csv"
    wifi_survey.write_csv(path_a, "保管場所", "2026-11-05T10:12:00+09:00", "wlo1", rows_a, 1, 3)
    wifi_survey.write_csv(path_b, "保管場所", "2026-11-05T10:15:00+09:00", "wlo1", rows_b, 1, 4)
    assert wifi_survey.summary("2026-11-05", tmp_path) == 0
    output = capsys.readouterr().out
    assert "保管場所 | 2026-11-05 10:12:00 | 1 | 0 | 0 | 1 | 1 | 0" in output
    assert "機体の AP: 見える" in output
    assert "保管場所 | 2026-11-05 10:15:00 | 0 | 1 | 0 | 1 | 0 | 0" in output
    assert output.count("場所 | 時刻 |") == 1
    assert "PRIVATE-SSID" not in output


def test_summary_skips_corrupt_csv_rows(tmp_path, capsys):
    import wifi_survey
    rows = aggregate_scans([[row("private", "aa", 1, 2412)]])
    path = tmp_path / "2026-11-05_101200_場所.csv"
    wifi_survey.write_csv(path, "場所", "2026-11-05T10:12:00+09:00", "wlo1", rows, 1, 3)
    content = path.read_text(encoding="utf-8-sig").replace(",2412,70,", ",not-a-number,70,")
    path.write_text(content, encoding="utf-8-sig")
    assert wifi_survey.summary("2026-11-05", tmp_path) == 1
    assert "警告" in capsys.readouterr().err
