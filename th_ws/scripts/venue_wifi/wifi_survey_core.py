"""Pure helpers for the venue Wi-Fi survey."""
from __future__ import annotations

VEHICLE_SSIDS = {"th-rpi-ap", "th-rpi-5g"}
CSV_FIELDS = ["location", "started", "ifname", "band", "channel", "frequency_mhz", "strength_pct", "dbm", "seen_scans", "scan_count", "scans_completed", "elapsed_seconds", "ssid", "bssid", "security", "max_bitrate", "vehicle_ap"]
CSV_NUMERIC_FIELDS = ("channel", "frequency_mhz", "strength_pct", "dbm", "seen_scans", "scan_count", "scans_completed", "elapsed_seconds")


def split_nmcli_t(line: str) -> list[str]:
    """Split nmcli -t output, honoring backslash-escaped separators."""
    fields, current = [], []
    escaped = False
    for char in line.rstrip("\n"):
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(char)
    if escaped:
        current.append("\\")
    fields.append("".join(current))
    return fields


def parse_nmcli_rows(output: str) -> list[dict]:
    names = ("ssid", "bssid", "channel", "frequency_mhz", "strength_pct", "rate", "security", "dbus_path")
    rows = []
    for line in output.splitlines():
        values = split_nmcli_t(line)
        if len(values) != len(names):
            continue
        row = dict(zip(names, values))
        try:
            row["channel"] = int(row["channel"])
            # nmcli 1.36 renders the frequency field as, for example, "5180 MHz".
            row["frequency_mhz"] = int(row["frequency_mhz"].removesuffix(" MHz"))
            row["strength_pct"] = int(row["strength_pct"])
        except (TypeError, ValueError):
            continue
        rows.append(row)
    return rows


def band_for_frequency(freq: int) -> str | None:
    if 2400 <= freq < 2500:
        return "2.4 GHz"
    if 5000 <= freq < 5925:
        return "5 GHz"
    if 5925 <= freq < 7200:
        return "6 GHz"
    return None


def channel_for_frequency(freq: int) -> int | None:
    if freq == 2484:
        return 14
    band = band_for_frequency(freq)
    if band == "2.4 GHz":
        return round((freq - 2407) / 5)
    if band == "5 GHz":
        return round((freq - 5000) / 5)
    if band == "6 GHz":
        return round((freq - 5950) / 5)
    return None


def dbm_from_percent(pct: int) -> int:
    pct = max(0, min(100, int(pct)))
    return round(-40 - (100 - pct) * 0.6)


def dbm_label(pct: int) -> str:
    return "約 −40 dBm 以上" if pct >= 100 else f"約 −{abs(dbm_from_percent(pct))} dBm"


def classify(row: dict) -> dict | None:
    freq = int(row.get("frequency_mhz", 0))
    band = band_for_frequency(freq)
    if not band:
        return None
    result = dict(row)
    result.update(band=band, channel=channel_for_frequency(freq), dbm=dbm_from_percent(row.get("strength_pct", 0)))
    result["vehicle_ap"] = row.get("ssid", "") in VEHICLE_SSIDS
    return result


def aggregate_scans(scans: list[list[dict]]) -> list[dict]:
    merged = {}
    count = len(scans)
    for scan in scans:
        seen = set()
        for raw in scan:
            row = classify(raw)
            if row is None:
                continue
            key = row.get("bssid", "").upper()
            if not key:
                continue
            if key not in merged:
                merged[key] = {**row, "bssid": key, "seen_scans": 0, "scan_count": count}
            existing = merged[key]
            if row["strength_pct"] > existing["strength_pct"]:
                existing["strength_pct"], existing["dbm"] = row["strength_pct"], row["dbm"]
            for field in ("ssid", "channel", "frequency_mhz", "band", "security", "rate"):
                if row.get(field):
                    existing[field] = row[field]
            existing["vehicle_ap"] = row["vehicle_ap"]
            seen.add(key)
        for key in seen:
            merged[key]["seen_scans"] += 1
    return sorted(merged.values(), key=lambda x: (-x["strength_pct"], x["bssid"]))


def filter_last_seen(rows: list[dict], started_boottime: int) -> list[dict]:
    return [r for r in rows if int(r.get("last_seen", -1)) >= started_boottime]


def collect_scans(n, request_scan, read_last_scan, list_aps, read_last_seen,
                  boottime, monotonic, sleep, timeout=15, interval=0.25):
    """Collect per-scan fresh AP rows using injected I/O and clock functions.

    ``request_scan`` returns either a bool or ``(bool, error_text)``. A failed
    request still samples the current AP list but cannot count as completed.
    """
    scans, completed, warnings = [], 0, []
    for number in range(1, n + 1):
        before = read_last_scan()
        scan_started = int(boottime())
        request = request_scan()
        if isinstance(request, tuple):
            requested, error = request
        else:
            requested, error = bool(request), ""
        if not requested:
            warnings.append(f"再スキャン {number} 回目の要求に失敗しました{': ' + error if error else ''}")
        else:
            deadline = monotonic() + timeout
            finished = False
            while before >= 0 and monotonic() < deadline:
                sleep(interval)
                after = read_last_scan()
                if after > before:
                    completed += 1
                    finished = True
                    break
            if not finished:
                warnings.append(f"スキャン {number} 回目は {timeout} 秒以内に完了しませんでした")

        fresh = []
        for row in list_aps():
            enriched = {**row, "last_seen": read_last_seen(row)}
            if filter_last_seen([enriched], scan_started):
                fresh.append(enriched)
        scans.append(fresh)
    return {"scans": scans, "completed": completed, "warnings": warnings}


def csv_row(location, started, ifname, row, scans_completed, elapsed):
    """Convert one aggregated AP to the production CSV representation."""
    return {"location": location, "started": started, "ifname": ifname,
            "band": row["band"], "channel": row["channel"],
            "frequency_mhz": row["frequency_mhz"], "strength_pct": row["strength_pct"],
            "dbm": row["dbm"], "seen_scans": row["seen_scans"],
            "scan_count": row["scan_count"], "scans_completed": scans_completed,
            "elapsed_seconds": elapsed, "ssid": row.get("ssid", ""),
            "bssid": row["bssid"], "security": row.get("security", ""),
            "max_bitrate": row.get("rate", ""),
            "vehicle_ap": str(bool(row.get("vehicle_ap"))).lower()}


def parse_csv_row(row):
    """Parse and validate one CSV row; raise ValueError for corrupt values."""
    parsed = dict(row)
    try:
        for key in CSV_NUMERIC_FIELDS:
            parsed[key] = int(parsed[key])
        flag = parsed["vehicle_ap"].lower()
        if flag not in {"true", "false"}:
            raise ValueError("vehicle_ap must be true or false")
        parsed["vehicle_ap"] = flag == "true"
        if not parsed["location"] or not parsed["started"] or not parsed["ifname"]:
            raise ValueError("missing measurement metadata")
        if parsed["band"] not in {"2.4 GHz", "5 GHz", "6 GHz"}:
            raise ValueError("invalid band")
    except (KeyError, AttributeError, TypeError) as exc:
        raise ValueError("missing or invalid CSV field") from exc
    return parsed


def summarize(rows: list[dict]) -> dict:
    counted = [r for r in rows if not r.get("vehicle_ap")]
    two = [r for r in counted if r["band"] == "2.4 GHz"]
    five = [r for r in counted if r["band"] == "5 GHz"]
    six = [r for r in counted if r["band"] == "6 GHz"]
    near = [sum(1 for r in two if 1 <= r["channel"] <= 3), sum(1 for r in two if 4 <= r["channel"] <= 8), sum(1 for r in two if 9 <= r["channel"] <= 14)]
    strongest = max(counted, key=lambda r: r["strength_pct"], default=None)
    return {"near": near, "two_total": len(two), "five_w52": sum(36 <= r["channel"] <= 48 for r in five), "five_other": sum(not 36 <= r["channel"] <= 48 for r in five), "six_count": len(six), "strongest": strongest, "counted": counted, "vehicle": [r for r in rows if r.get("vehicle_ap")]}


def channel_load(rows: list[dict]) -> dict:
    counted = [r for r in rows if not r.get("vehicle_ap")]
    result = {"2.4 GHz": [], "5 GHz": []}
    for ch in (1, 6, 11):
        same = [r for r in counted if r["band"] == "2.4 GHz" and r["channel"] == ch]
        overlap = [r for r in counted if r["band"] == "2.4 GHz" and 0 < abs(r["channel"] - ch) <= 4]
        result["2.4 GHz"].append({"channel": ch, "same": same, "overlap": overlap, "strongest": max(same + overlap, key=lambda r:r["strength_pct"], default=None)})
    for label, block in (("44+48", {44,48}), ("36+40", {36,40})):
        stations = [r for r in counted if r["band"] == "5 GHz" and r["channel"] in block]
        result["5 GHz"].append({"block": label, "stations": stations, "strongest": max(stations,key=lambda r:r["strength_pct"],default=None)})
    return result


def format_channel_load(rows: list[dict]) -> str:
    loads = channel_load(rows)
    lines = ["機体のチャネルの混み具合（機体の AP は数に入れていない）"]
    for item in loads["2.4 GHz"]:
        strongest = dbm_label(item["strongest"]["strength_pct"]) if item["strongest"] else "なし"
        lines.append(f"  2.4 GHz  ch {item['channel']}{'（いま）' if item['channel']==1 else ''}  同じ ch {len(item['same'])}・重なる ch {len(item['overlap'])}   最強 {strongest}")
    for item in loads["5 GHz"]:
        strongest = dbm_label(item["strongest"]["strength_pct"]) if item["strongest"] else "なし"
        tail = f"   最強 {strongest}"
        if item["block"] == "44+48":
            overlap_80 = sum(r["band"] == "5 GHz" and r["channel"] in {36, 40} and not r.get("vehicle_ap") for r in rows)
            tail += f"   36・40 の {overlap_80} 局は 80 MHz 幅なら重なる"
        lines.append(f"  5 GHz    {item['block']}{'（いま）' if item['block']=='44+48' else ''} {len(item['stations'])} 局{tail}")
    return "\n".join(lines)


def format_summary(location: str, timestamp: str, ifname: str, scans_done: int, scans_total: int, elapsed: int, rows: list[dict]) -> str:
    s = summarize(rows)
    strongest = s["strongest"]
    strongest_text = f"{dbm_label(strongest['strength_pct'])}（{strongest['band']} ch {strongest['channel']}）" if strongest else "なし"
    vehicle = ", ".join(f"{r['ssid']} ch{r['channel']} {dbm_label(r['strength_pct'])}" for r in s["vehicle"]) or "見えない"
    lines = [f"E. 無線  場所: {location}   {timestamp}   {ifname}・スキャン {scans_total} 回（{scans_done} 回完了）・約 {elapsed} 秒",
             f"  2.4 GHz の数   ch1 付近 {s['near'][0]}   ch6 付近 {s['near'][1]}   ch11 付近 {s['near'][2]}   合計 {s['two_total']}",
             f"  5 GHz の数     ch36〜48 {s['five_w52']}   その他 {s['five_other']}",
             f"  最も強い電波   {strongest_text}", f"  メモ           6 GHz {s['six_count']} 件／機体の AP: {vehicle}",
             "  ※ 付近: ch1〜3 → 1、4〜8 → 6、9〜14 → 11。数は BSSID の数。dBm は強度 % からの換算", "",
             format_channel_load(rows)]
    return "\n".join(lines)


def format_summary_table(entries):
    """Render one spreadsheet-shaped row per measurement, then channel details."""
    header = "場所 | 時刻 | ch1 付近 | ch6 付近 | ch11 付近 | 合計 | ch36〜48 | その他 | 最も強い電波 | メモ"
    table = [header]
    for meta, rows in entries:
        counts = summarize(rows)
        strongest = counts["strongest"]
        strongest_text = f"{dbm_label(strongest['strength_pct'])}（{strongest['band']} ch {strongest['channel']}）" if strongest else "なし"
        vehicle_text = "見える" if counts["vehicle"] else "見えない"
        memo = f"6 GHz {counts['six_count']} 件／機体の AP: {vehicle_text}"
        stamp = meta["started"]
        table.append(f"{meta['location']} | {stamp} | {counts['near'][0]} | {counts['near'][1]} | {counts['near'][2]} | {counts['two_total']} | {counts['five_w52']} | {counts['five_other']} | {strongest_text} | {memo}")
    details = []
    for meta, rows in entries:
        stamp = meta["started"]
        details.append(f"\n{meta['location']} {stamp}")
        details.append(format_channel_load(rows))
    return "\n".join(table + details)


def safe_location(location: str) -> str:
    cleaned = "".join("_" if c in "/\\\0" or ord(c) < 32 else c for c in location).strip(" .")
    return cleaned or "場所"
