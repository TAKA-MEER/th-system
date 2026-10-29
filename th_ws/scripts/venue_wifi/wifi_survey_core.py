"""Pure helpers for the venue Wi-Fi survey."""
from __future__ import annotations

from collections import defaultdict
from datetime import datetime

VEHICLE_SSIDS = {"th-rpi-ap", "th-rpi-5g"}
CSV_FIELDS = ["location", "started", "ifname", "band", "channel", "frequency_mhz", "strength_pct", "dbm", "seen_scans", "scan_count", "scans_completed", "elapsed_seconds", "ssid", "bssid", "security", "max_bitrate", "vehicle_ap"]


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
    other = {r["channel"] for r in counted if r["band"] == "5 GHz"} - {36,40,44,48}
    result["5 GHz_other_w52"] = [r for r in counted if r["band"] == "5 GHz" and r["channel"] in {36,40,44,48} and r["channel"] not in {ch for ch in (44,48)}]
    return result


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
             "機体のチャネルの混み具合（機体の AP は数に入れていない）"]
    for item in channel_load(rows)["2.4 GHz"]:
        strongest_text = dbm_label(item["strongest"]["strength_pct"]) if item["strongest"] else "なし"
        lines.append(f"  2.4 GHz  ch {item['channel']}{'（いま）' if item['channel']==1 else ''}  同じ ch {len(item['same'])}・重なる ch {len(item['overlap'])}   最強 {strongest_text}")
    loads = channel_load(rows)
    for item in loads["5 GHz"]:
        strongest_text = dbm_label(item["strongest"]["strength_pct"]) if item["strongest"] else "なし"
        tail = f"   最強 {strongest_text}"
        if item["block"] == "44+48":
            tail += f"   36・40 の {sum(r['band']=='5 GHz' and r['channel'] in {36,40} and not r.get('vehicle_ap') for r in rows)} 局は 80 MHz 幅なら重なる"
        lines.append(f"  5 GHz    {item['block']}{'（いま）' if item['block']=='44+48' else ''} {len(item['stations'])} 局{tail}")
    return "\n".join(lines)


def safe_location(location: str) -> str:
    cleaned = "".join("_" if c in "/\\\0" or ord(c) < 32 else c for c in location).strip(" .")
    return cleaned or "場所"
