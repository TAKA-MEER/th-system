"""Pure helpers for the Wi-Fi checker (Python standard library only)."""
from __future__ import annotations

VEHICLE_SSIDS = {"th-rpi-ap", "th-rpi-5g"}
# Keep these defaults aligned with docs/network.md and
# th_ws/scripts/rpi_ap_set_channel.sh, which changes the vehicle AP channels.
DEFAULT_VEHICLE_CHANNELS = {"2.4 GHz": 1, "5 GHz": 48}


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
    saw_aps = False
    retained_aps = 0
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
        listed = list_aps()
        saw_aps = saw_aps or bool(listed)
        for row in listed:
            enriched = {**row, "last_seen": read_last_seen(row)}
            if filter_last_seen([enriched], scan_started):
                fresh.append(enriched)
                retained_aps += 1
        scans.append(fresh)
    if saw_aps and retained_aps == 0:
        warnings.append("一覧の AP がすべて測り始めより古い")
    return {"scans": scans, "completed": completed, "warnings": warnings}


def summarize(rows: list[dict]) -> dict:
    counted = [r for r in rows if not r.get("vehicle_ap")]
    two = [r for r in counted if r["band"] == "2.4 GHz"]
    five = [r for r in counted if r["band"] == "5 GHz"]
    six = [r for r in counted if r["band"] == "6 GHz"]
    near = [sum(1 for r in two if 1 <= r["channel"] <= 3), sum(1 for r in two if 4 <= r["channel"] <= 8), sum(1 for r in two if 9 <= r["channel"] <= 14)]
    strongest = max(counted, key=lambda r: r["strength_pct"], default=None)
    other = [r for r in five if not 36 <= r["channel"] <= 48]
    return {"near": near, "two_total": len(two), "five_w52": sum(36 <= r["channel"] <= 48 for r in five),
            "five_total": len(five), "five_other": len(other), "five_52_64": sum(52 <= r["channel"] <= 64 for r in other),
            "five_100_144": sum(100 <= r["channel"] <= 144 for r in other),
            "five_other_channels": sum(not (52 <= r["channel"] <= 64 or 100 <= r["channel"] <= 144) for r in other),
            "six_count": len(six), "strongest": strongest, "counted": counted,
            "vehicle": [r for r in rows if r.get("vehicle_ap")]}


def vehicle_channels(rows: list[dict]) -> dict[str, int]:
    """Use the strongest visible vehicle AP channel, or documented defaults."""
    result = dict(DEFAULT_VEHICLE_CHANNELS)
    for band in result:
        visible = [r for r in rows if r.get("vehicle_ap") and r["band"] == band]
        if visible:
            result[band] = max(visible, key=lambda r: r["strength_pct"])["channel"]
    return result


def five_ghz_block(channel: int) -> str | None:
    if channel in {36, 40}:
        return "36+40"
    if channel in {44, 48}:
        return "44+48"
    return None


def channel_load(rows: list[dict]) -> dict:
    counted = [r for r in rows if not r.get("vehicle_ap")]
    vehicle = vehicle_channels(rows)
    result = {"2.4 GHz": [], "5 GHz": [], "vehicle": vehicle,
              "vehicle_default": {band: not any(r.get("vehicle_ap") and r["band"] == band for r in rows) for band in vehicle}}
    two_channels = set(range(1, 14)) | {r["channel"] for r in rows if r["band"] == "2.4 GHz" and r["channel"] == 14} | {vehicle["2.4 GHz"]}
    for ch in sorted(two_channels):
        same = [r for r in counted if r["band"] == "2.4 GHz" and r["channel"] == ch]
        overlap = [r for r in counted if r["band"] == "2.4 GHz" and 0 < abs(r["channel"] - ch) <= 4]
        result["2.4 GHz"].append({"channel": ch, "same": same, "overlap": overlap,
                                  "is_vehicle": ch == vehicle["2.4 GHz"],
                                  "strongest": max(same, key=lambda r:r["strength_pct"], default=None),
                                  "candidate_strongest": max(same + overlap, key=lambda r:r["strength_pct"], default=None)})
    five_channels = {r["channel"] for r in rows if r["band"] == "5 GHz"} | {vehicle["5 GHz"]}
    for ch in sorted(five_channels):
        label = five_ghz_block(ch)
        block = ({36, 40} if label == "36+40" else {44, 48}) if label else {ch}
        stations = [r for r in counted if r["band"] == "5 GHz" and r["channel"] in block]
        channel_stations = [r for r in counted if r["band"] == "5 GHz" and r["channel"] == ch]
        result["5 GHz"].append({"channel": ch, "block": label, "is_vehicle": ch == vehicle["5 GHz"],
                                "stations": stations, "channel_stations": channel_stations,
                                "strongest": max(channel_stations,key=lambda r:r["strength_pct"],default=None)})
    return result


def format_channel_load(rows: list[dict], band: str | None = None) -> str:
    loads = channel_load(rows)
    counted = [r for r in rows if not r.get("vehicle_ap")]
    lines = []
    if band in (None, "2.4"):
        lines.append("チャネル別（2.4 GHz）")
    maximum = max((len(item["same"]) for item in loads["2.4 GHz"]), default=0)
    if band in (None, "2.4"):
        for item in loads["2.4 GHz"]:
            strongest = dbm_label(item["strongest"]["strength_pct"]) if item["strongest"] else "なし"
            count = len(item["same"])
            bar = "█" * (max(1, round(14 * count / maximum)) if count else 0)
            mark = " ← 機体" + ("（既定）" if loads["vehicle_default"]["2.4 GHz"] else "") if item["is_vehicle"] else ""
            lines.append(f"  ch {item['channel']:>3}  {bar:<14} {count:>3}   最強 {strongest}{mark}")
    if band in (None, "5"):
        lines.append("チャネル別（5 GHz）")
    maximum = max((len(item["channel_stations"]) for item in loads["5 GHz"]), default=0)
    if band in (None, "5"):
        for item in loads["5 GHz"]:
            strongest = dbm_label(item["strongest"]["strength_pct"]) if item["strongest"] else "なし"
            count = len(item["channel_stations"])
            bar = "█" * (max(1, round(14 * count / maximum)) if count else 0)
            mark = " ← 機体" + ("（既定）" if loads["vehicle_default"]["5 GHz"] else "") if item["is_vehicle"] else ""
            lines.append(f"  ch {item['channel']:>3}  {bar:<14} {count:>3}   最強 {strongest}{mark}")
    if band in (None, "2.4", "5"):
        lines.append("使うチャネルの候補")
    if band in (None, "2.4"):
        for item in loads["2.4 GHz"]:
            if item["channel"] not in {1, 6, 11}:
                continue
            strongest = dbm_label(item["candidate_strongest"]["strength_pct"]) if item["candidate_strongest"] else "なし"
            mark = " ← 機体" + ("（既定）" if loads["vehicle_default"]["2.4 GHz"] else "") if item["is_vehicle"] else ""
            lines.append(f"  2.4 GHz  ch {item['channel']}{mark}  同じ ch {len(item['same'])}・重なる ch {len(item['overlap'])}  最強 {strongest}")
    if band in (None, "5"):
        for label, block in (("36+40", {36, 40}), ("44+48", {44, 48})):
            stations = [r for r in counted if r["band"] == "5 GHz" and r["channel"] in block]
            strongest_row = max(stations, key=lambda r: r["strength_pct"], default=None)
            strongest = dbm_label(strongest_row["strength_pct"]) if strongest_row else "なし"
            is_vehicle = loads["vehicle"]["5 GHz"] in block
            mark = " ← 機体" + ("（既定）" if loads["vehicle_default"]["5 GHz"] else "") if is_vehicle else ""
            tail = f"{len(stations)} 局  最強 {strongest}"
            if label == "44+48":
                overlap_80 = sum(r["band"] == "5 GHz" and r["channel"] in {36, 40} and not r.get("vehicle_ap") for r in rows)
                tail += f"  36・40 の {overlap_80} 局は 80 MHz 幅なら重なる"
            lines.append(f"  5 GHz    {label}{mark}  {tail}")
    return "\n".join(lines)


def format_summary(timestamp: str, ifname: str, scans_done: int, scans_total: int, elapsed: int, rows: list[dict], band: str | None = None, find: str | None = None, top: int | None = 20, hide_names: bool = False, watch_number: int | None = None) -> str:
    s = summarize(rows)
    strongest = max((r for r in s["counted"] if band is None or r["band"].startswith(band)),
                    key=lambda r: r["strength_pct"], default=None)
    strongest_text = f"{dbm_label(strongest['strength_pct'])}（{strongest['band']} ch {strongest['channel']}）" if strongest else "なし"
    header = f"Wi-Fi 確認  {ifname}  {timestamp}  スキャン {scans_total} 回（{scans_done} 回完了）・約 {elapsed} 秒"
    if watch_number is not None:
        header = f"{header}  {watch_number} 回目・Ctrl-C で終了"
    lines = [header, "", "まとめ（機体の AP は数に入れていない。数は BSSID の数）"]
    if band in (None, "2.4"):
        lines.append(f"  2.4 GHz  {s['two_total']} 件   ch1 付近 {s['near'][0]}   ch6 付近 {s['near'][1]}   ch11 付近 {s['near'][2]}")
    if band in (None, "5"):
        lines.append(f"  5 GHz    {s['five_total']} 件   ch36〜48 {s['five_w52']}   その他 {s['five_other']}（ch52〜64 {s['five_52_64']}・ch100〜144 {s['five_100_144']}・それ以外 {s['five_other_channels']}）")
    if band in (None, "6"):
        lines.append(f"  6 GHz    {s['six_count']} 件")
    lines.append(f"  最も強い電波   {strongest_text if strongest else 'なし'}")
    if band is None or band in ("2.4", "5"):
        vehicle_rows = [r for r in s["vehicle"] if band is None or r["band"].startswith(band)]
        vehicle = ("／".join(f"AP ch{r['channel']} {dbm_label(r['strength_pct'])}" for r in vehicle_rows)
                   if hide_names else "／".join(f"{r['ssid']} ch{r['channel']} {dbm_label(r['strength_pct'])}" for r in vehicle_rows)) or "見えない"
        lines.append(f"  機体の AP      {vehicle}")
    lines.append("  ※ ch1〜3 → 1 付近、4〜8 → 6 付近、9〜14 → 11 付近。dBm は強度 % からの換算")
    if band in (None, "2.4", "5"):
        lines.extend(["", format_channel_load(rows, band)])
    listed = list(rows)
    if find:
        listed = [r for r in listed if find.casefold() in r.get("ssid", "").casefold()]
    if band:
        listed = [r for r in listed if r["band"].startswith(band)]
    if top is not None:
        listed = listed[:top]
    lines.extend(["", f"一覧（強い順・{'全部' if top is None else f'上位 {top} 件'}）"])
    for r in listed:
        ssid = "（非表示）" if hide_names else (r.get("ssid") or "（非公開）")
        bssid = "（非表示）" if hide_names else r["bssid"]
        vehicle_mark = "  機体" if r.get("vehicle_ap") else ""
        lines.append(f"  {r['band']}  ch {r['channel']}  {dbm_label(r['strength_pct'])}  {ssid}  {bssid}  {r['seen_scans']}/{r['scan_count']} 回{vehicle_mark}")
    return "\n".join(lines)
