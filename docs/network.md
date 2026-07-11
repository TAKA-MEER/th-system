# ネットワーク構成と復旧手順 (ESP32 AP)

[← README に戻る](../README.md)

## 構成の全体像

**ESP32 を WiFi AP にし、PC とラズパイをそのクライアントとして接続する**構成を採用している。

```txt
ESP32 (駆動用, WiFi AP)          192.168.4.1   SSID: th-esp32-ap
  ├── PC (Windows+WSL2, ROS2)   192.168.4.50  (固定IP必須 → setup.md 2-2)
  │     ESP32 → WS 接続先: 192.168.4.50:8766 (コンテナ内 esp32_bridge が直接待ち受け)
  └── ラズパイ (LiDAR 配信)      192.168.4.2   (DHCP。AP再起動で変わりうる)
        RPLIDAR S1 → rplidar_ros → /scan (ROS_DOMAIN_ID=10)
```

採用の経緯: ラズパイ自身を AP にする構成では「AP 自身が発信元の通信が PC に届かない」
非対称なネットワーク障害が実機で発生した。ESP32 を AP にして PC とラズパイを対等な
ステーションにすることでこの問題を回避している。

### 通信の性質 (実機検証で確立した前提)

| 事項 | 内容 |
| --- | --- |
| WS 定期リフレッシュ | ESP32 は接続 5 分ごとに WebSocket を意図的に再接続する(TCP の無言死への保険)。esp32_bridge のログに周期的な接続/切断が出るのは**正常**。死活検知本体は 3 秒周期の ping/pong |
| WiFi ジッタ | /scan・wheel_feedback とも 0.5〜1.2 秒程度の受信ギャップが平常時でも出る。safety_monitor のタイムアウトはこれを織り込んで 2000ms/2500ms に設定済み |
| portproxy | **廃止済み**。esp32_bridge が 8766 を直接待ち受ける。古いエントリが残っていると横取りされるので削除する(setup.md 2-4) |
| マルチキャスト | ESP32 SoftAP 経由のホスト間マルチキャストは**不安定**。DDS ディスカバリが成立しない場合はユニキャストピア設定を使う(後述) |

---

## 復旧手順 (症状別)

### PC が AP に繋がらない / 繋がっているのに ping 192.168.4.1 が通らない

AP(ESP32)がリセットされると、Windows が「死んだアソシエーション」を掴んだままになることがある。
**切断→再接続で直る**:

```powershell
netsh wlan disconnect interface="<アダプタ名>"
netsh wlan connect name=th-esp32-ap interface="<アダプタ名>"
ping 192.168.4.1
```

- `接続できません。アダプターで WLAN 自動構成が無効になっています` と出る場合:
  `netsh wlan set autoconfig enabled=yes interface="<アダプタ名>"`(管理者)してから接続する
  (走行前の autoconfig 無効化の副作用。setup.md 2-3 の注意参照)。

### ESP32 が WS に繋がらない (esp32_bridge に「接続:」ログが出ない)

切り分けのヒント: ESP32 のシリアルログに 5 秒ごとに `[WiFi-AP] 接続クライアント数=N` が出る。
これが 0 に落ちる場合は ESP32 側ではなく **PC/ラズパイ側の WiFi が切れている**。

1. PC の WiFi 状態と固定 IP を確認(上記 + `Get-NetIPAddress -InterfaceAlias "<アダプタ名>"` で 192.168.4.50)
2. portproxy の残骸を確認(setup.md 2-4)
3. コンテナ内でブリッジが 8766 を LISTEN しているか: `ss -tln | grep 8766`(WSL 側)
4. esp32_bridge を多重起動していないか(bind 失敗で fatal ログを出して落ちる設計)

> **シリアルモニタの注意**: シリアルポートを開くと DTR/RTS 自動リセット回路で
> **ESP32 が再起動 = AP が落ちる**。走行中・通信確認中はシリアルを開かないこと。
> 開いた後は上記の PC WiFi 再接続が必要になる。

### ラズパイに ssh できない

- AP 復帰直後は再接続に数十秒かかる。`ping 192.168.4.2` を待つ
- DHCP で IP が変わった可能性: `192.168.4.2〜.5` あたりを ping で探す
- それでもだめならラズパイの電源を入れ直す(rplidar は systemd で自動復帰する)

### /scan がコンテナに届かない (LIDAR_LOST が消えない)

確認の順番:

1. ラズパイ側でノードが生きているか:
   `ssh` して `systemctl status rplidar` / ローカルで `ros2 topic hz /scan`
2. **ラズパイのネットワークを切り替えた直後はノード再起動が必要**
   (FastDDS は起動時の IP に固着する): `sudo systemctl restart rplidar`
3. `ROS_DOMAIN_ID=10` の一致(環境変数はプロセス起動時にのみ読まれる)
4. Windows Firewall のラズパイ IP 許可(setup.md 3)
5. **時刻ズレ**: /scan は届いているのに SLAM/AMCL が捨てる場合はこれ
   (setup.md 4。`Message Filter dropping message` が症状)
6. マルチキャスト疎通テスト:
   ```bash
   # コンテナ側
   ros2 multicast receive
   # ラズパイ側
   ros2 multicast send
   ```
   届かない場合は次項のユニキャストピア設定を使う

### DDS マルチキャストが死んでいる場合 (ユニキャストピア)

SoftAP のマルチキャスト転送は不安定で、疎通しないことがある。その場合は
FastDDS の初期ピアをユニキャストで指定する:

- **コンテナ側**: `th_ws/src/th_bringup/config/fastdds_profile.xml` を
  `FASTRTPS_DEFAULT_PROFILES_FILE` で指定(docker-compose.yml にコメントアウト済みの行あり)
- **ラズパイ側**: 同等の XML(ピア=192.168.4.50)を作り、rplidar サービスの
  `Environment=FASTRTPS_DEFAULT_PROFILES_FILE=...` に指定

> 注意: コンテナ側でこのプロファイルを常用すると、コンテナ内の一部ノード間の
> ローカル発見が不安定になる事象を観測している。まずは既定(プロファイルなし)で試し、
> /scan が届かない場合のみラズパイ側だけに入れるのが安定。

### 全体が不調 (CLI が topic を見つけない・ノード間が部分的に不通)

長時間セッションや WiFi 再接続の繰り返しで、WSL の DDS まわりが劣化することがある。
**リセットの定石**(この順で強い):

```powershell
# 1. コンテナ再起動 (/dev/shm もクリアされる。SIGKILL された ROS プロセスの
#    共有メモリ残骸が DDS を壊すため、ROS ノードを pkill -9 したら必ずこれ)
docker restart th_robot

# 2. WSL ごと再起動 (mirrored networking の状態もリセット)
wsl --shutdown
# → コンテナ起動から毎回の起動手順 (README) をやり直す
```

### ESP32 自体が固まった (AP が見えない・ping 192.168.4.1 不可)

シリアルポートを一度開閉すると DTR/RTS でリセットがかかる(PC に USB 接続されている場合)。
リセット後は「PC が AP に繋がらない」の手順で各クライアントを再接続する。
