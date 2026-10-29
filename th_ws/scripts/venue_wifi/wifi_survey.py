#!/usr/bin/env python3
"""Survey nearby Wi-Fi access points through NetworkManager."""
import argparse
import csv
import datetime as dt
import os
from pathlib import Path
import subprocess
import sys
import time

from wifi_survey_core import CSV_FIELDS, aggregate_scans, dbm_label, filter_last_seen, format_summary, parse_nmcli_rows, safe_location, summarize

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUT = Path.home() / "venue_wifi"


def run(args):
    return subprocess.run(args, text=True, capture_output=True)


def devices():
    result = run(["nmcli", "-t", "-f", "DEVICE,TYPE", "device"])
    if result.returncode:
        return []
    return [d for d, typ in (line.split(":", 1) for line in result.stdout.splitlines() if ":" in line) if typ == "wifi"]


def dbus_int(path, prop):
    result = run(["busctl", "--system", "get-property", "org.freedesktop.NetworkManager", path, "org.freedesktop.NetworkManager.AccessPoint", prop])
    if result.returncode:
        return -1
    try:
        return int(result.stdout.strip().split()[-1])
    except (ValueError, IndexError):
        return -1


def list_rows(ifname):
    result = run(["nmcli", "-t", "-f", "SSID,BSSID,CHAN,FREQ,SIGNAL,RATE,SECURITY,DBUS-PATH", "dev", "wifi", "list", "ifname", ifname])
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "nmcli の一覧取得に失敗しました")
    return parse_nmcli_rows(result.stdout)


def repository_path(path):
    try:
        path.resolve().relative_to(ROOT.resolve())
        return True
    except ValueError:
        return False


def write_csv(path, location, started, ifname, rows, scans_completed, elapsed):
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({"location": location, "started": started, "ifname": ifname, "band": row["band"], "channel": row["channel"], "frequency_mhz": row["frequency_mhz"], "strength_pct": row["strength_pct"], "dbm": row["dbm"], "seen_scans": row["seen_scans"], "scan_count": row["scan_count"], "scans_completed": scans_completed, "elapsed_seconds": elapsed, "ssid": row.get("ssid", ""), "bssid": row["bssid"], "security": row.get("security", ""), "max_bitrate": row.get("rate", ""), "vehicle_ap": str(bool(row.get("vehicle_ap"))).lower()})


def summary(day, outdir):
    paths = sorted(outdir.glob(f"{day}_*.csv"))
    grouped = {}
    for path in paths:
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            continue
        meta = rows[0]
        converted = []
        for row in rows:
            for key in ("channel", "frequency_mhz", "strength_pct", "dbm", "seen_scans", "scan_count"):
                row[key] = int(row[key])
            row["vehicle_ap"] = row["vehicle_ap"].lower() == "true"
            converted.append(row)
        grouped.setdefault(meta["location"], []).append((meta, converted))
    for location, entries in grouped.items():
        for meta, rows in entries:
            stamp = dt.datetime.fromisoformat(meta["started"]).strftime("%Y-%m-%d %H:%M")
            print(format_summary(location, stamp, meta["ifname"], int(meta["scans_completed"]), int(meta["scan_count"]), int(meta["elapsed_seconds"]), rows))
            print()
    if not paths:
        print(f"{day} の測定 CSV がありません: {outdir}", file=sys.stderr)
        return 1
    return 0


def main():
    parser = argparse.ArgumentParser(description="会場確認シート E 用の Wi-Fi 電波調査")
    parser.add_argument("location", nargs="?", help="測定場所")
    parser.add_argument("--summary", nargs="?", const="today", metavar="日付", help="保存済みの測定を表形式で表示")
    parser.add_argument("--ifname", help="測定する Wi-Fi デバイス")
    parser.add_argument("--scans", type=int, default=4, help="スキャン回数（既定 4）")
    parser.add_argument("--names", action="store_true", help="SSID・BSSID 一覧も表示")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="CSV 保存先")
    args = parser.parse_args()
    if args.scans < 1:
        parser.error("--scans は 1 以上を指定してください")
    if args.summary is not None:
        date = dt.date.today().isoformat() if args.summary == "today" else args.summary
        return summary(date, args.out)
    if not args.location:
        parser.error("場所を指定してください")
    devs = devices()
    if not devs:
        print("NetworkManager の Wi-Fi デバイスがありません。nmcli device status を確認してください。", file=sys.stderr)
        return 1
    if args.ifname:
        if args.ifname not in devs:
            print(f"Wi-Fi デバイスではありません: {args.ifname}\n利用可能: {', '.join(devs)}", file=sys.stderr)
            return 1
        ifname = args.ifname
    else:
        ifname = "wlo1" if "wlo1" in devs else devs[0]
    radio = run(["nmcli", "radio", "wifi"])
    if radio.returncode or radio.stdout.strip().lower() != "enabled":
        print("Wi-Fi が OFF です。nmcli radio wifi on を実行してください。", file=sys.stderr)
        return 1
    outdir = args.out.expanduser()
    if repository_path(outdir):
        print(f"警告: 保存先がリポジトリ内です。SSID・BSSID の取り扱いに注意してください: {outdir}", file=sys.stderr)
    outdir.mkdir(parents=True, exist_ok=True)
    start_dt = dt.datetime.now().astimezone()
    started = start_dt.isoformat(timespec="seconds")
    start_boot = int(time.clock_gettime(time.CLOCK_BOOTTIME))
    merged_scans, completed = [], 0
    t0 = time.monotonic()
    for index in range(args.scans):
        previous = run(["nmcli", "-g", "GENERAL.DBUS-PATH", "device", "show", ifname])
        if previous.returncode or not previous.stdout.strip():
            print(f"デバイスの D-Bus パスを取得できません: {ifname}", file=sys.stderr)
            continue
        devpath = previous.stdout.strip()
        state = run(["busctl", "--system", "get-property", "org.freedesktop.NetworkManager", devpath, "org.freedesktop.NetworkManager.Device.Wireless", "LastScan"])
        try:
            old_scan = int(state.stdout.split()[-1])
        except (ValueError, IndexError):
            old_scan = -1
        rescan = run(["nmcli", "dev", "wifi", "rescan", "ifname", ifname])
        if rescan.returncode:
            print(f"警告: 再スキャンを要求できませんでした: {rescan.stderr.strip()}", file=sys.stderr)
        else:
            deadline = time.monotonic() + 15
            while old_scan >= 0 and time.monotonic() < deadline:
                time.sleep(.25)
                state = run(["busctl", "--system", "get-property", "org.freedesktop.NetworkManager", devpath, "org.freedesktop.NetworkManager.Device.Wireless", "LastScan"])
                try:
                    new_scan = int(state.stdout.split()[-1])
                except (ValueError, IndexError):
                    new_scan = old_scan
                if new_scan > old_scan:
                    completed += 1
                    break
            else:
                print(f"警告: スキャン {index + 1} 回目は 15 秒以内に完了しませんでした", file=sys.stderr)
        rows = list_rows(ifname)
        fresh = filter_last_seen([{**r, "last_seen": dbus_int(r["dbus_path"], "LastSeen")} for r in rows if r.get("dbus_path")], start_boot)
        merged_scans.append(fresh)
    if completed == 0:
        print("スキャンが 1 回も完了しませんでした。測定を保存しません。", file=sys.stderr)
        return 1
    rows = aggregate_scans(merged_scans)
    elapsed = round(time.monotonic() - t0)
    print(format_summary(args.location, start_dt.strftime("%Y-%m-%d %H:%M"), ifname, completed, args.scans, elapsed, rows))
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    csv_path = outdir / f"{stamp}_{safe_location(args.location)}.csv"
    write_csv(csv_path, args.location, started, ifname, rows, completed, elapsed)
    if args.names:
        print("\n一覧（強い順）")
        for row in rows:
            ssid = row.get("ssid") or "（非公開）"
            print(f"  {row['band']} ch{row['channel']} {dbm_label(row['strength_pct'])}  {ssid}  {row['bssid']}  {row['seen_scans']}/{row['scan_count']} 回")
    print(f"\n保存: {csv_path}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
