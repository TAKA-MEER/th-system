#!/usr/bin/env python3
"""Show nearby Wi-Fi access points and channel occupancy via NetworkManager."""
import argparse
import datetime as dt
import os
import subprocess
import sys
import time

from wifi_check_core import aggregate_scans, collect_scans, format_summary, parse_nmcli_rows


def run(args, env=None):
    command_env = os.environ.copy()
    if env:
        command_env.update(env)
    return subprocess.run(args, text=True, capture_output=True, env=command_env)


def devices():
    result = run(["nmcli", "-t", "-f", "DEVICE,TYPE", "device"], env={"LC_ALL": "C"})
    if result.returncode:
        return []
    return [d for d, typ in (line.split(":", 1) for line in result.stdout.splitlines() if ":" in line) if typ == "wifi"]


def dbus_int(path, prop):
    result = run(["busctl", "--system", "get-property", "org.freedesktop.NetworkManager", path,
                  "org.freedesktop.NetworkManager.AccessPoint", prop])
    if result.returncode:
        return -1
    try:
        return int(result.stdout.strip().split()[-1])
    except (ValueError, IndexError):
        return -1


def list_rows(ifname):
    result = run(["nmcli", "-t", "-f", "SSID,BSSID,CHAN,FREQ,SIGNAL,RATE,SECURITY,DBUS-PATH",
                  "dev", "wifi", "list", "ifname", ifname], env={"LC_ALL": "C"})
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "nmcli の一覧取得に失敗しました")
    return parse_nmcli_rows(result.stdout)


def main():
    parser = argparse.ArgumentParser(description="周りの Wi-Fi とチャネルの混み具合を表示")
    parser.add_argument("--ifname", help="測定する Wi-Fi デバイス（既定: wlo1、なければ最初の Wi-Fi デバイス）")
    parser.add_argument("--scans", type=int, help="まとめるスキャン回数（既定 3、--watch 時 1）")
    parser.add_argument("--watch", action="store_true", help="測り直しながら画面を更新")
    parser.add_argument("--band", choices=("2.4", "5", "6"), help="表示する帯域")
    parser.add_argument("--find", metavar="文字列", help="一覧を SSID の部分一致で絞る")
    listing = parser.add_mutually_exclusive_group()
    listing.add_argument("--top", type=int, default=20, help="一覧に出す件数（既定 20）")
    listing.add_argument("--all", action="store_true", help="一覧をすべて表示")
    parser.add_argument("--hide-names", action="store_true", help="SSID と BSSID を隠す")
    args = parser.parse_args()
    if args.scans is not None and args.scans < 1:
        parser.error("--scans は 1 以上を指定してください")
    if args.top is not None and args.top < 1:
        parser.error("--top は 1 以上を指定してください")
    scan_count = args.scans if args.scans is not None else (1 if args.watch else 3)
    top = None if args.all else args.top

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
    radio = run(["nmcli", "radio", "wifi"], env={"LC_ALL": "C"})
    if radio.returncode or radio.stdout.strip().lower() != "enabled":
        print("Wi-Fi が OFF です。nmcli radio wifi on を実行してください。", file=sys.stderr)
        return 1
    path_result = run(["nmcli", "-g", "GENERAL.DBUS-PATH", "device", "show", ifname], env={"LC_ALL": "C"})
    if path_result.returncode or not path_result.stdout.strip():
        print(f"デバイスの D-Bus パスを取得できません: {ifname}", file=sys.stderr)
        return 1
    devpath = path_result.stdout.strip()

    def read_last_scan():
        result = run(["busctl", "--system", "get-property", "org.freedesktop.NetworkManager", devpath,
                      "org.freedesktop.NetworkManager.Device.Wireless", "LastScan"])
        try:
            return int(result.stdout.split()[-1]) if result.returncode == 0 else -1
        except (ValueError, IndexError):
            return -1

    def request_scan():
        result = run(["nmcli", "dev", "wifi", "rescan", "ifname", ifname], env={"LC_ALL": "C"})
        return result.returncode == 0, result.stderr.strip()

    iteration = 0
    try:
        while True:
            iteration += 1
            started = dt.datetime.now().astimezone()
            t0 = time.monotonic()
            collected = collect_scans(scan_count, request_scan, read_last_scan,
                                      lambda: list_rows(ifname),
                                      lambda row: dbus_int(row["dbus_path"], "LastSeen"),
                                      lambda: time.clock_gettime(time.CLOCK_BOOTTIME),
                                      time.monotonic, time.sleep)
            rows = aggregate_scans(collected["scans"])
            elapsed = round(time.monotonic() - t0)
            output = format_summary(started.strftime("%Y-%m-%d %H:%M"), ifname,
                                    collected["completed"], scan_count, elapsed, rows,
                                    band=args.band, find=args.find, top=top,
                                    hide_names=args.hide_names, watch_number=iteration if args.watch else None)
            if args.watch:
                print("\033[H\033[2J", end="")
            print(output)
            for warning in collected["warnings"]:
                print(f"警告: {warning}")
            if not args.watch:
                if collected["completed"] == 0:
                    print("スキャンが 1 回も完了しませんでした。", file=sys.stderr)
                    return 1
                return 0
            time.sleep(1)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
    except (OSError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
