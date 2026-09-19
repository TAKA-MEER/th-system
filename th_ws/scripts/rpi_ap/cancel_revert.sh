#!/usr/bin/env bash
# 自動復旧の予約を取り消す（新ネットワークで疎通が取れたあとに実行する）
sudo systemctl stop th_ap_revert.timer 2>/dev/null
sudo systemctl reset-failed th_ap_revert.service 2>/dev/null
echo "自動復旧の予約を取り消しました"
