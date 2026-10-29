# 会場 Wi-Fi 調査

会場確認シート E「無線（3 か所で調べる）」に転記するため、NetworkManager が見つけた AP を数えます。Ubuntu の一般ユーザー権限で実行し、`nmcli` と `busctl` を使います。`iw`・sudo は使いません。

## 会場での手順

1. PC を持って保管場所で測ります。
2. 場所を移ってから同じ操作を経路の途中、試験場で行います。
3. 既定では 4 回スキャンし、表示された上 3 行をシート E の 1 行に写します。

```bash
python3 th_ws/scripts/venue_wifi/wifi_survey.py 保管場所
python3 th_ws/scripts/venue_wifi/wifi_survey.py 経路の途中
python3 th_ws/scripts/venue_wifi/wifi_survey.py 試験場 --names
python3 th_ws/scripts/venue_wifi/wifi_survey.py --summary
python3 th_ws/scripts/venue_wifi/wifi_survey.py --summary 2026-11-05
```

`--ifname` で無線デバイス、`--scans N` でスキャン回数、`--out DIR` で保存先を指定できます。既定の無線は `wlo1`（存在する場合）、既定の保存先は `~/venue_wifi/` です。SSID・BSSID の一覧は `--names` を指定した場合だけ表示します。

## 画面とシート E の対応

| 画面 | シート E |
| --- | --- |
| 2.4 GHz の数: ch1 付近・ch6 付近・ch11 付近・合計 | 2.4 GHz の 4 欄 |
| 5 GHz の数: ch36〜48・その他 | 5 GHz の 2 欄 |
| 最も強い電波 | dBm 欄 |
| 6 GHz 件数・機体の AP・チャネル別混み具合 | メモ欄 |

「付近」は ch 1〜3、4〜8、9〜14 の区分です。数える単位は BSSID です。複数の SSID を出す AP や、同じ建物の複数 AP があると、機器の台数より多く数えることがあります。機体の AP は SSID が `th-rpi-ap` または `th-rpi-5g` と完全一致するものだけ数から除きます。

チャネル別表示では、2.4 GHz は候補と同じチャネル、またはチャネル差 1〜4 の局を示します。5 GHz は現行設定の 44+48 と候補 36+40 を示し、36/40 は 44+48 に対して「80 MHz 幅なら重なる」局数として表示します。混雑度の○△✕は判定しません。

## 読み取り上の限界

- dBm は NetworkManager の強度 % からの換算値です。「約」として読み、100% は「約 −40 dBm 以上」と表示します。
- NetworkManager のスキャン一覧には古い AP が残るため、各スキャンの要求直前を基準に `LastSeen` が新しい AP だけをその回に数えます。
- DFS の W53/W56 は受け身のスキャンになり、複数回測っても取りこぼす場合があります。
- NetworkManager 1.36 では AP のチャネル幅を取得できないため、80 MHz の重なりは件数だけを参考表示します。
- 6 GHz はシートに専用欄がないため、件数をメモに出し合計には含めません。

## 保存データ

測定ごとに CSV を保存します。列には SSID と BSSID が含まれます。会場名・SSID・BSSID は会場の公開範囲と生成 AI への入力可否に従い、共有やリポジトリへの保存を判断してください。保存先をリポジトリ内にすると警告します。測定画面の既定表示には SSID・BSSID を含めません。

`--summary` は保存済み CSV を読み、シート E の欄順に 1 測定 1 行の表を出し、その下に各測定のチャネル別混み具合を表示します。同じ場所を複数回測った場合も時刻付きで各測定を表示します。SSID・BSSID は出しません。各スキャン回で、その回の再スキャン要求直前より古い `LastSeen` の AP は数えません。
