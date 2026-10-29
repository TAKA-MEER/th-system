import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts" / "wifi_check"))

from wifi_check_core import (aggregate_scans, band_for_frequency, channel_for_frequency,
                              classify, collect_scans, dbm_from_percent, filter_last_seen,
                              format_summary, parse_nmcli_rows,
                              split_nmcli_t, summarize)


def row(ssid, bssid, chan, freq, pct=70, **extra):
    return {"ssid": ssid, "bssid": bssid, "channel": chan, "frequency_mhz": freq,
            "strength_pct": pct, "security": "WPA2", **extra}


def install_fake_cli(monkeypatch, advance_scans=True):
    import wifi_check
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
                # The old AP's timestamp advances only after the first sample.
                # Each round must use its own pre-rescan freshness boundary.
                value = access_points[command[4]] + (scan_value[0] - 101)
                return SimpleNamespace(returncode=0, stdout=f"i {value}\n", stderr="")
            raise AssertionError(f"Unexpected busctl property: {command}")
        raise AssertionError(f"Unexpected command: {command}")

    monkeypatch.setattr(wifi_check, "run", fake_run)
    ticks = [0.0]
    monkeypatch.setattr(wifi_check.time, "clock_gettime", lambda _: 10 + (scan_value[0] - 100))
    monkeypatch.setattr(wifi_check.time, "monotonic", lambda: ticks[0])
    monkeypatch.setattr(wifi_check.time, "sleep", lambda seconds: ticks.__setitem__(0, ticks[0] + seconds))
    return wifi_check, calls


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


def test_main_wires_dbus_last_seen_into_collection_and_sets_nmcli_locale(monkeypatch, capsys):
    wifi_check, calls = install_fake_cli(monkeypatch)
    monkeypatch.setattr(sys, "argv", ["wifi_check.py", "--scans", "2"])
    assert wifi_check.main() == 0
    output = capsys.readouterr().out
    assert "stale-only" not in output
    assert "fresh-only" in output
    assert "2.4 GHz  1 件" in output
    assert "th-rpi-ap ch1" in output
    assert any("th-rpi-ap" in line and line.endswith("機体") for line in output.splitlines())
    assert all(env and env.get("LC_ALL") == "C" for command, env in calls if command[0] == "nmcli")


def test_main_returns_one_when_lastscan_never_advances(monkeypatch, capsys):
    wifi_check, _ = install_fake_cli(monkeypatch, advance_scans=False)
    monkeypatch.setattr(sys, "argv", ["wifi_check.py", "--scans", "1"])
    assert wifi_check.main() == 1
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
    from wifi_check_core import channel_load
    rows = aggregate_scans([[row("one", "z", 1, 2412), row("five", "a", 5, 2432, 80), row("six", "b", 6, 2437),
                             row("44", "c", 44, 5220), row("48", "d", 48, 5240),
                             row("36", "e", 36, 5180), row("40", "f", 40, 5200)]])
    loads = channel_load(rows)
    ch1 = loads["2.4 GHz"][0]
    ch6 = next(item for item in loads["2.4 GHz"] if item["channel"] == 6)
    assert len(ch1["overlap"]) == 1 and ch1["overlap"][0]["channel"] == 5
    assert not any(r["channel"] == 1 for r in ch6["overlap"])
    assert len(loads["5 GHz"][0]["stations"]) == 2
    assert len(loads["5 GHz"][1]["stations"]) == 2


def test_output_contains_channel_tables_summary_and_counts():
    rows = aggregate_scans([[row("ch1", "a", 1, 2412, 70),
                             row("ch6a", "b", 6, 2437, 70), row("ch6b", "c", 6, 2437, 70),
                             row("ch11a", "d", 11, 2462, 70), row("ch11b", "e", 11, 2462, 70),
                             row("ch11c", "f", 11, 2462, 70), row("overlap", "g", 2, 2417, 80),
                             row("44", "h", 44, 5220, 70), row("48", "i", 48, 5240, 70),
                             row("36a", "j", 36, 5180, 75), row("36b", "k", 36, 5180, 72),
                             row("40", "l", 40, 5200, 71), row("other", "m", 149, 5745, 90),
                             row("six", "n", 5, 5975, 60), row("th-rpi-ap", "v", 1, 2412, 90)]])
    text = format_summary("2026-10-29 20:25", "wlo1", 1, 1, 22, rows)
    assert "2.4 GHz  7 件" in text
    assert "5 GHz    6 件" in text and "その他 1（ch52〜64 0・ch100〜144 0・それ以外 1）" in text
    assert "6 GHz    1 件" in text
    assert "  ch  13 " in text and "  ch  14 " not in text
    assert "ch 36" in text and "ch 149" in text
    assert "ch   1  █████            1   約 −58 dBm" in text
    assert "ch   6  █████████        2   約 −58 dBm" in text
    assert "ch  36  ██████████████   2   約 −55 dBm" in text
    assert "ch 149  ███████          1   約 −46 dBm" in text
    assert "ch1" in text


def test_five_ghz_other_subcategories():
    rows = aggregate_scans([[row("a", "a", 52, 5260), row("b", "b", 100, 5500), row("c", "c", 149, 5745)]])
    summary = summarize(rows)
    assert (summary["five_52_64"], summary["five_100_144"], summary["five_other_channels"]) == (1, 1, 1)


def test_channel_bars_have_one_block_for_small_nonzero_count():
    from wifi_check_core import format_channel_load
    crowded = [row(f"ap-{index}", f"{index:02X}", 1, 2412) for index in range(30)]
    crowded.append(row("one-ap", "FE", 2, 2417))
    lines = format_channel_load(aggregate_scans([crowded])).splitlines()
    ch2_line = next(line for line in lines if line.startswith("  ch   2"))
    assert ch2_line.count("█") == 1


def test_channel_14_appears_only_when_seen():
    from wifi_check_core import format_channel_load
    base = format_channel_load(aggregate_scans([[row("base", "a", 1, 2412)]]))
    seen = format_channel_load(aggregate_scans([[row("ch14", "b", 14, 2484)]]))
    assert "  ch  14 " not in base
    assert "  ch  14 " in seen


def test_candidate_heading_appears_once_for_each_relevant_band():
    rows = aggregate_scans([[row("two", "a", 1, 2412), row("five", "b", 36, 5180)]])
    for band in (None, "2.4", "5"):
        output = format_summary("now", "wlo1", 1, 1, 1, rows, band=band)
        assert output.count("使うチャネルの候補") == 1


def test_channel_load_labels_and_empty_rows():
    rows = aggregate_scans([[row("ch1", "a", 1, 2412, 70)]])
    output = format_summary("now", "wlo1", 1, 1, 1, rows)
    assert "最強" not in output
    assert output.count("チャネル別（2.4 GHz）") == 1
    assert output.count("チャネル別（5 GHz）") == 1
    assert output.count("いちばん強い電波") >= 3
    assert "  ch   2" in output and "0   —" in output
    assert "なし" not in "\n".join(line for line in output.splitlines() if line.startswith("  ch "))


def test_candidate_strongest_includes_overlapping_24ghz_channels():
    from wifi_check_core import channel_load, format_channel_load
    rows = aggregate_scans([[row("same", "a", 1, 2412, 50),
                             row("overlap", "b", 3, 2422, 83),
                             row("ch6", "c", 6, 2437, 10)]])
    item = next(item for item in channel_load(rows)["2.4 GHz"] if item["channel"] == 1)
    assert item["strongest"]["dbm"] == -70
    assert item["candidate_strongest"]["dbm"] == -50
    output = format_channel_load(rows, "2.4")
    channel_one = next(line for line in output.splitlines() if line.startswith("  ch   1"))
    candidate_one = next(line for line in output.splitlines() if line.startswith("  2.4 GHz  ch 1 "))
    assert "約 −70 dBm" in channel_one
    assert "約 −50 dBm" in candidate_one
    assert "約 −70 dBm" not in candidate_one


def test_five_ghz_candidates_use_only_their_40mhz_block():
    from wifi_check_core import format_channel_load
    rows = aggregate_scans([[row("ch36", "a", 36, 5180, 83),
                             row("ch40", "b", 40, 5200, 67),
                             row("ch44", "c", 44, 5220, 33)]])
    output = format_channel_load(rows, "5")
    candidate_3640 = next(line for line in output.splitlines() if line.startswith("  5 GHz    36+40"))
    candidate_4448 = next(line for line in output.splitlines() if line.startswith("  5 GHz    44+48"))
    assert "約 −50 dBm" in candidate_3640
    assert "約 −60 dBm" not in candidate_3640
    assert "約 −80 dBm" in candidate_4448
    assert "約 −50 dBm" not in candidate_4448


def test_display_width_counts_fullwidth_characters_as_two_columns():
    from wifi_check_core import display_width
    assert display_width("日本語") == 6
    assert display_width("ASCII") == 5
    assert display_width("件数") == 4
    assert display_width("ch 1") == 4


def test_vehicle_marks_follow_visible_channels_and_default_when_missing():
    from wifi_check_core import channel_load
    visible = aggregate_scans([[row("th-rpi-ap", "v2", 6, 2437), row("th-rpi-5g", "v5", 40, 5200)]])
    text = format_summary("now", "wlo1", 1, 1, 1, visible)
    two = {item["channel"]: item for item in channel_load(visible)["2.4 GHz"]}
    assert two[6]["is_vehicle"] and not two[1]["is_vehicle"]
    assert "36+40 ← 機体" in text
    defaults = format_summary("now", "wlo1", 1, 1, 1, [])
    assert "ch  1" in defaults and "← 機体（既定）" in defaults
    assert "44+48 ← 機体（既定）" in defaults


def test_find_top_all_hide_names_and_band_filter(monkeypatch):
    rows = aggregate_scans([[row("Alpha", "aa", 1, 2412, 90), row("beta", "bb", 6, 2437, 80),
                             row("gamma", "cc", 36, 5180, 70)]])
    filtered = format_summary("now", "wlo1", 1, 1, 1, rows, find="ALP", top=None)
    assert "Alpha" in filtered and "beta" not in filtered
    assert "2.4 GHz  2 件" in filtered
    assert "Alpha" in format_summary("now", "wlo1", 1, 1, 1, rows, top=1)
    assert "beta" not in format_summary("now", "wlo1", 1, 1, 1, rows, top=1)
    all_text = format_summary("now", "wlo1", 1, 1, 1, rows, top=None)
    assert all(name in all_text for name in ("Alpha", "beta", "gamma"))
    with_vehicle = rows + aggregate_scans([[row("th-rpi-ap", "dd:ee", 1, 2412)]])
    with_both_vehicle_aps = with_vehicle + aggregate_scans([[row("th-rpi-5g", "ff:00", 48, 5240)]])
    machine_search = format_summary("now", "wlo1", 1, 1, 1, with_both_vehicle_aps, find="th-rpi", top=None)
    machine_lines = [line for line in machine_search.splitlines() if "th-rpi" in line and line.endswith("機体")]
    assert len(machine_lines) == 2
    assert all(line.endswith("機体") for line in machine_lines)
    assert "2.4 GHz  2 件" in machine_search
    hidden = format_summary("now", "wlo1", 1, 1, 1, with_vehicle, hide_names=True, top=None)
    assert all(token not in hidden for token in ("Alpha", "beta", "gamma", "th-rpi-ap", "AA", "BB", "CC", "DD:EE"))
    assert any(line.endswith("機体") and "（非表示）" in line for line in hidden.splitlines())
    only_five = format_summary("now", "wlo1", 1, 1, 1, rows, band="5", top=None)
    assert "まとめ（" in only_five and "5 GHz" in only_five and "2.4 GHz  2 件" not in only_five


def test_main_defaults_to_three_scans_and_excludes_vehicle_from_count(monkeypatch, capsys):
    wifi_check, calls = install_fake_cli(monkeypatch)
    monkeypatch.setattr(sys, "argv", ["wifi_check.py"])
    assert wifi_check.main() == 0
    output = capsys.readouterr().out
    assert "スキャン 3 回（3 回完了）" in output
    assert "th-rpi-ap ch1" in output
    assert "2.4 GHz  1 件" in output
    assert all(env and env.get("LC_ALL") == "C" for command, env in calls if command[0] == "nmcli")


def test_watch_refreshes_each_round_and_keyboard_interrupt_returns_zero(monkeypatch, capsys):
    wifi_check, _ = install_fake_cli(monkeypatch)
    monkeypatch.setattr(sys, "argv", ["wifi_check.py", "--watch"])
    sleeps = [0]
    def stop_after_two(_seconds):
        sleeps[0] += 1
        if sleeps[0] == 4:
            raise KeyboardInterrupt
    monkeypatch.setattr(wifi_check.time, "sleep", stop_after_two)
    assert wifi_check.main() == 0
    out = capsys.readouterr().out
    assert out.count("\033[H\033[2J") == 2
    assert out.count("回目・Ctrl-C で終了") == 2
    assert "stale-only" not in out
