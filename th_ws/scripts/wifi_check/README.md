# Wi-Fi 確認

NetworkManager が見つけた周囲のアクセスポイントと、チャネルごとの見え方を表示する道具です。会場の下見や、機体との通信が不安定なときの切り分けに使えます。測定結果は保存しません。

Ubuntu と NetworkManager が動く PC で使えます。`nmcli` と `busctl` を利用し、通常ユーザー権限で動作します。sudo は不要です。

```bash
python3 th_ws/scripts/wifi_check/wifi_check.py
python3 th_ws/scripts/wifi_check/wifi_check.py --watch
python3 th_ws/scripts/wifi_check/wifi_check.py --band 2.4
python3 th_ws/scripts/wifi_check/wifi_check.py --find th-rpi
```

## sudo を付けると正確な dBm

root で実行すると `wpa_cli scan_results` の受信信号レベルを読み、NetworkManager の AP 一覧に照合します。`iw` は不要です。root でない実行は従来どおり強度 % から換算し、画面にも測定方法を表示します。root で実測に失敗した場合は警告を出して換算値で続行します。照合できない AP もその行だけ換算値になります。

実測できないときに処理を失敗させるには `--exact` を付けます。sudo の実機確認は次で行えます。

```bash
sudo python3 th_ws/scripts/wifi_check/wifi_check.py --exact --scans 1 --hide-names
```

複数回の測定で実測値と換算値が混ざった場合、AP ごとに実測値を優先し、同じ種類の値同士では dBm が強い回を残します。

## オプション

| オプション | 既定 | 説明 |
| --- | --- | --- |
| `--ifname IFNAME` | `wlo1`、なければ最初の Wi-Fi デバイス | 測定する無線デバイス |
| `--scans N` | 3（`--watch` では 1） | 1 回の表示にまとめるスキャン回数 |
| `--watch` | 無効 | 測り直して画面を更新し続けます。Ctrl-C で終了します |
| `--exact` | 無効 | 実測 dBm を必須にします。sudo なし、または実測取得失敗時は終了コード 1 で終了します。`--watch` と併用できます |
| `--band 2.4\|5\|6` | 全帯域 | 表示する帯域を絞ります |
| `--find TEXT` | なし | 一覧を SSID の部分一致で絞ります（大文字小文字を区別しません）。機体の AP も一覧に出ますが、集計数には含みません。まとめやチャネル表には影響しません |
| `--top N` | 20 | 一覧に出す強い AP の件数 |
| `--all` | 無効 | 一覧をすべて表示します |
| `--hide-names` | 無効 | SSID と BSSID を隠します |

## 画面の読み方

「まとめ」の数は BSSID の数で、機体の AP（`th-rpi-ap` と `th-rpi-5g`）は数に含みません。2.4 GHz は ch1・6・11 付近の数、5 GHz は ch36〜48 とその他の数を表示します。チャネル別の表は 2.4 GHz ch1〜13 と、見つかった 5 GHz チャネルを表示します。5 GHz の候補は 36+40 と 44+48 の 40 MHz ブロックです。機体の AP が見えれば現在のチャネルに印を付け、見えない場合は既定チャネル（2.4 GHz ch1、5 GHz 44+48）に「機体（既定）」と表示します。

- 「付近」は ch1〜3 → 1、ch4〜8 → 6、ch9〜14 → 11 に分けます。
- 「重なる ch」はチャネル差が1〜4の AP 数です。
- チャネル別の「いちばん強い電波」はそのチャネル内、2.4 GHz の候補は同じ ch と重なる ch 内、5 GHz の候補はその 40 MHz 内の値です。いずれも機体の AP を除き、dBm は 0 に近いほど強い値です。
- 5 GHz の「その他」は ch52〜64、ch100〜144、それ以外に分けて表示します。

会場確認シート E に転記するときは、まとめの 2.4 GHz と 5 GHz の行、および「最も強い電波」を使います。

## 限界

- sudo なしでは dBm は NetworkManager の強度 % から換算した目安です。sudo で実測できます。
- NetworkManager からチャネル幅を取得できないため、実際に重なる幅は分かりません。
- DFS 対象の ch52 以上は、受け身のスキャンになり取りこぼすことがあります。
- BSSID の数は無線機器の台数とは一致しません。1 台が複数の BSSID を持つ場合があります。
