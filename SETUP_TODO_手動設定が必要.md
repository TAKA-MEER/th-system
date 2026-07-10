# 手動設定が必要な項目（権限の都合で自動設定できなかったもの）

2026-07-11 の実機検証セッションで必要と判明したが、管理者権限等の制約で
Claude Code から設定できなかった項目。各自の環境で実施すること。

## 1. Windows Firewall: ラズパイからの DDS 受信許可（管理者 PowerShell）

ラズパイ配信の `/scan` を受信するために必要（README「ラズパイ等ネットワーク経由の LiDAR」参照）。
ESP32 AP ネットワーク上 (192.168.4.x) では現状動作しているが、ラズパイの IP を変えた場合や
別ネットワークに繋いだ場合は、その IP に対するルールが必要になる。

```powershell
# <ラズパイのIP> を実際の IP に置き換える (例: 192.168.4.2 / 192.168.188.115)
New-NetFirewallRule -DisplayName "TH-System Pi LiDAR (inbound UDP)" -Direction Inbound -Action Allow -Protocol UDP -RemoteAddress <ラズパイのIP> -Profile Any
New-NetFirewallRule -DisplayName "TH-System Pi LiDAR (inbound TCP)" -Direction Inbound -Action Allow -Protocol TCP -RemoteAddress <ラズパイのIP> -Profile Any
```

## 2. ラズパイ: rplidar の systemd サービス化（ラズパイ上で sudo）

現在 LiDAR 配信は手動起動（nohup）のため、ラズパイを再起動すると止まる。
以下でサービス化すると再起動後も自動で `/scan` 配信が復帰する。
**`frame_id:=laser_link` は必須**（既定の `laser` だと TF が繋がらない）。

```bash
sudo tee /etc/systemd/system/rplidar.service << 'EOF'
[Unit]
Description=RPLIDAR S1 /scan publisher (TH system, frame_id=laser_link)
After=network.target

[Service]
Type=simple
User=mirs2602
Environment=ROS_DOMAIN_ID=10
ExecStart=/bin/bash -lc "source /opt/ros/humble/setup.bash && source /home/mirs2602/ros2_ws/install/setup.bash && exec ros2 launch rplidar_ros rplidar_s1_launch.py frame_id:=laser_link"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now rplidar
```

## 3. 【重要・未実施】PC の WiFi バックグラウンドスキャン停止（管理者 PowerShell）

実測で、Windows が th-esp32-ap 接続中に**約60秒周期で WiFi を微断**し（ESP32 側ログの
「接続クライアント数=0」で確認）、そのたびに WebSocket が切断→再接続まで40秒強かかる
事象を確認した。原因はインターネットの無い AP に対する Windows の定期スキャン/
ローミング動作と推定。運用時（走行時）は以下で該当アダプタのスキャンを止めること:

```powershell
# 走行前 (管理者)
netsh wlan set autoconfig enabled=no interface="Wi-Fi 3"

# 作業終了後に戻す
netsh wlan set autoconfig enabled=yes interface="Wi-Fi 3"
```

これを止めない場合、待機中に周期的な ESP32_DISCONNECTED フォルトが発生しうる
（走行中は ESP32 側ウォッチドッグ 300ms で安全に停止するため危険はないが、
追従が数十秒中断する）。

**注意（実機で確認）**: この設定は PC 再起動後も残り、**無効のままだと WiFi の
再接続自体ができなくなる**（`接続できません。アダプターで WLAN 自動構成が無効に
なっています`）。手順は必ず「①enabled=yes → ②th-esp32-ap に接続 → ③enabled=no」
の順で行うこと。PC 再起動後は①②を忘れずに。

## 3.5. （実施済みだが記録として）PC の WiFi プロファイル自動接続

`th-esp32-ap` のプロファイルが「手動接続」だと、瞬断後に Windows が再接続せず
WebSocket が数分間つながらない。以下は 2026-07-11 に設定済み。再インストール時は再設定すること。

```powershell
netsh wlan set profileparameter name=th-esp32-ap connectionmode=auto
```

## 3.6. 【要実施】古い portproxy エントリの削除（管理者 PowerShell）

esp32_bridge は 8766 で直接待ち受ける構成に変更した（params.yaml の ws_port: 8766）。
古い portproxy エントリが残っていると Windows が 8766 を横取りして通信不能になる:

```powershell
netsh interface portproxy delete v4tov4 listenport=8766 listenaddress=192.168.4.50
netsh interface portproxy show all   # 空になっていることを確認
```

（現在は portproxy のリスナー自体が機能停止しているため偶然通信できているが、
Windows 再起動などでリスナーが復活すると通信を横取りする。必ず削除すること。）

## 3.7. 【設定済み・記録】WSL のアイドル自動終了の無効化

WSL は対話セッションが無くなると約60秒で VM ごと自動終了し、docker・コンテナ・
ROS2 ノードが全て巻き添えで死ぬ（実機検証で「esp32_bridge が黙って消える」原因だった）。
`%USERPROFILE%\.wslconfig` に以下を設定済み（2026-07-11）。再セットアップ時は忘れずに:

```ini
[wsl2]
networkingMode=mirrored
vmIdleTimeout=2000000000   # ms。-1 は不正値で逆効果なので大きな正値を使う
```

ただしこの設定でも Windows 再起動では消えるため、ロボット運用中は WSL ターミナルを
1枚開いたままにしておくのが確実。

## 3.8. 【重要・未実施】PC とラズパイの時刻同期（管理者 PowerShell）

実測で **Windows の時計が実時間より約0.9秒遅れている**ことを確認した（ラズパイは NTP
同期済みで正確）。ROS2 はメッセージのタイムスタンプで TF 変換するため、このズレにより
SLAM Toolbox / AMCL / costmap がラズパイ配信の /scan を「未来のデータ」として全て破棄し、
**地図が一切生成されない**。

```powershell
# 即時同期（管理者 PowerShell）
w32tm /resync
# 確認: ずれの表示
w32tm /stripchart /computer:time.windows.com /samples:3 /dataonly
```

恒久対策（実運用時に必須）: 実機構成では ESP32 AP 配下でラズパイがインターネットに
出られず NTP 同期できないため、時間が経つと再びズレる。**PC (WSL) 側で chrony サーバー
を立て、ラズパイを PC に同期させる**構成を推奨:

```bash
# WSL (Ubuntu) 側: sudo apt install chrony して /etc/chrony/chrony.conf に追記
#   allow 192.168.4.0/24
#   local stratum 10
# ラズパイ側: /etc/chrony/chrony.conf の pool をコメントアウトし
#   server 192.168.4.50 iburst prefer
```

## 4. 残課題（設定ではなく調査事項）

- ESP32 の WS 定期リフレッシュ後、再接続に約45秒かかることがある
  （portproxy / WSL mirrored networking 経由の TCP 接続が間欠的に失敗する疑い）。
  → 2026-07-11 に FORCE_RECONNECT_MS を 15秒→5分に延長して緩和済み
  （通信断の頻度が 1/20 に減る。死活検知はハートビートが3秒周期で実施）。
- PC 上に旧実験の残骸らしき `esp32_network_microros_agent` コンテナが稼働中。
  不要なら: `docker rm -f esp32_network_microros_agent`
