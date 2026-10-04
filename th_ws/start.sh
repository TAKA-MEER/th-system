#!/usr/bin/env bash
# ============================================================
# start.sh — TH システム 毎回の起動を 1 コマンドで
#
# 仕様: docs/plan/spec/Spec-ops.md §1・§2
# 手順: docs/使い方.md §1〜§2
#
# 電源 → PC がロボット AP に繋がっている → `cd th_ws && ./start.sh`
# → タブレットで表示された URL を開く、までを 1 本で行う。
#
# 終わるときは Ctrl-C。launch には INT を送って子ノードごと止める。
# 強制終了のシグナルや名前での一括停止は使わない（DDS discovery が
# 壊れる・自分のシェルを殺すため。CLAUDE.md「環境の癖」）。
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── 設定（冒頭に集約。環境変数で上書きできる） ──────────────
RPI_IP="${RPI_IP:-192.168.5.1}"                 # ラズパイ（AP 兼 /scan 配信元）
RPI_SSH_USER="${RPI_SSH_USER:-mirs2602}"        # ラズパイのユーザー名
ROBOT_UI_IP="${ROBOT_UI_IP:-192.168.5.50}"      # ロボット回線の PC 側 IP（タブレット用 URL）
WEBUI_PORT="${WEBUI_PORT:-5173}"                # WebUI の配信ポート
STARTSH_RESTART_MAX="${STARTSH_RESTART_MAX:-3}" # bringup の起動は最大何回まで（初回を含む）
STARTSH_RESTART_WAIT="${STARTSH_RESTART_WAIT:-5}" # 立て直しの前に待つ秒数

# bringup の既定の launch 引数（同じキーが渡されたら渡された方を使う）
DEFAULT_LAUNCH_ARGS=(lidar_source:=network use_stub:=false enable_route_slam:=true)

# ── 表示 ────────────────────────────────────────────────────
if [ -t 1 ]; then
    C_BOLD=$'\e[1m'; C_RED=$'\e[31m'; C_YELLOW=$'\e[33m'; C_RESET=$'\e[0m'
else
    C_BOLD=''; C_RED=''; C_YELLOW=''; C_RESET=''
fi
info() { echo "[start.sh] $*"; }
warn() { echo "${C_YELLOW}[start.sh] 警告: $*${C_RESET}"; }
err()  { echo "${C_RED}[start.sh] エラー: $*${C_RESET}" >&2; }

usage() {
    cat <<'USAGE'
使い方: ./start.sh [オプション] [launch 引数...]

  毎回の起動（bringup＋WebUI の配信）を 1 コマンドで行う。
  例: ./start.sh
      ./start.sh stage:=4
      ./start.sh enable_route_slam:=false
      ./start.sh --build stage:=4

オプション:
  --build      コンテナ内で colcon build、ホストで WebUI の本番ビルドを行う
               （既定ではしない。時間がかかるため）
  --dry-run    何も実行せず、これから何をするかだけ表示する
  --help, -h   この表示

launch 引数:
  `key:=value` 形式で bringup.launch.py にそのまま渡す。
  既定（lidar_source:=network use_stub:=false enable_route_slam:=true）と
  同じキーがあれば、渡された方が優先される。

環境変数:
  RPI_IP（既定 192.168.5.1）/ RPI_SSH_USER（既定 mirs2602）
  ROBOT_UI_IP（既定 192.168.5.50）/ WEBUI_PORT（既定 5173）
  STARTSH_RESTART_MAX（既定 3。起動は初回を含め最大この回数まで）
  STARTSH_RESTART_WAIT（既定 5 秒。立て直しの前の待ち時間）

止め方: Ctrl-C（launch に INT を送って子ノードごと止め、WebUI の配信も止める）
USAGE
}

# ── 引数の仕分け ────────────────────────────────────────────
DO_BUILD=0
DRY_RUN=0
USER_LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --help|-h) usage; exit 0 ;;
        --build) DO_BUILD=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --*) err "未知のオプション: $arg（--help を参照）"; exit 2 ;;
        *) USER_LAUNCH_ARGS+=("$arg") ;;
    esac
done

# 既定の引数に使用者の指定を上書きする（同じキーは使用者の方を使う）
LAUNCH_ARGS=()
for def in "${DEFAULT_LAUNCH_ARGS[@]}"; do
    key="${def%%:=*}"
    overridden=0
    for u in "${USER_LAUNCH_ARGS[@]}"; do
        if [ "${u%%:=*}" = "$key" ]; then overridden=1; break; fi
    done
    if [ "$overridden" -eq 0 ]; then LAUNCH_ARGS+=("$def"); fi
done
LAUNCH_ARGS+=("${USER_LAUNCH_ARGS[@]}")

INNER_LAUNCH="source /opt/ros/humble/setup.bash && source /root/th_ws/install/setup.bash && exec ros2 launch th_bringup bringup.launch.py ${LAUNCH_ARGS[*]:-}"

if [ "$DRY_RUN" -eq 1 ]; then
    cat <<DRY
[start.sh] --dry-run: 何も実行しない。実行時の手順は次のとおり:
  1. ping -c 3 ${RPI_IP}（届かなければ docs/network.md「復旧手順」を案内して終了）
     ssh ${RPI_SSH_USER}@${RPI_IP} で rpi-serial-relay を確認（失敗は警告のみ）
  2. コンテナ th_robot が無ければ \`docker compose up -d th_robot\`（th_ws/ で）、止まっていれば \`docker start th_robot\`
  3. コンテナ内で ros2 launch th_bringup bringup.launch.py が既に動いていれば、止めずに案内して終了
DRY
    if [ "$DO_BUILD" -eq 1 ]; then
        echo "  4. --build: コンテナ内で colcon build --symlink-install、ホストで npm run build（web_ui）"
    fi
    cat <<DRY
  5. bringup を起動: docker exec th_robot bash -lc '... exec ${INNER_LAUNCH#*exec }'
     終了コードにかかわらず立て直す（最大 ${STARTSH_RESTART_MAX} 回まで。その後は機体の電源再投入・AP・ケーブルの確認を案内）。
     立て直さないのは操作者の停止（Ctrl-C/SIGTERM）だけ
  6. WebUI を配信: web_ui/dist/ を npx vite preview --host --port ${WEBUI_PORT} --strictPort で配信
     タブレット: http://${ROBOT_UI_IP}:${WEBUI_PORT} ／ PC: http://localhost:${WEBUI_PORT}
  7. Ctrl-C で launch に INT を送って子ノードごと止め、WebUI の配信も止めて終わる
DRY
    exit 0
fi

# ── 1. ネットワークの確認 ───────────────────────────────────
info "ラズパイ (${RPI_IP}) に ping を送る..."
if ! ping -c 3 -W 2 "$RPI_IP" >/dev/null 2>&1; then
    err "ラズパイ (${RPI_IP}) に届かない。PC がロボット AP (th-rpi-ap-wlo1) に繋がっているか確かめること。"
    err "復旧手順は docs/network.md「復旧手順」を参照。"
    exit 1
fi
info "ラズパイに届いた。"

if ssh -o BatchMode=yes -o ConnectTimeout=3 "${RPI_SSH_USER}@${RPI_IP}" systemctl is-active rpi-serial-relay >/dev/null 2>&1; then
    info "rpi-serial-relay は動いている。"
else
    warn "rpi-serial-relay の状態を確認できなかった（ssh の鍵が無い場合を含む）。先へ進むが、ESP32 が繋がらないときは docs/network.md を参照。"
fi

# ── 2. コンテナの起動 ───────────────────────────────────────
if ! docker inspect th_robot >/dev/null 2>&1; then
    info "コンテナ th_robot が無いので作る..."
    (cd "$SCRIPT_DIR" && docker compose up -d th_robot)
elif [ "$(docker inspect -f '{{.State.Running}}' th_robot)" != "true" ]; then
    info "コンテナ th_robot を起動する..."
    docker start th_robot
else
    info "コンテナ th_robot は起動済み。"
fi

# ── 3. 二重起動の防止 ───────────────────────────────────────
# コンテナ内のプロセス一覧をホスト側で調べる。名前での一括停止は
# 自分のシェルを殺すことがあるため使わず、PID を特定するだけに留める。
find_launch_pid() {
    docker exec th_robot ps -eo pid,args 2>/dev/null \
        | awk '$0 ~ /bringup\.launch\.py/ && $0 ~ /ros2/ {print $1; exit}'
}
EXISTING_PID="$(find_launch_pid || true)"
if [ -n "${EXISTING_PID:-}" ]; then
    err "コンテナ内で bringup が既に動いている (PID ${EXISTING_PID})。何も止めずに終わる。"
    err "誰かが実機作業中の可能性がある。止めて起動し直すときは、その作業者に確認してから止めること。"
    exit 1
fi

# ── 4. （任意）ビルド ───────────────────────────────────────
if [ "$DO_BUILD" -eq 1 ]; then
    info "コンテナ内で colcon build --symlink-install を行う..."
    docker exec th_robot bash -lc 'source /opt/ros/humble/setup.bash && cd /root/th_ws && colcon build --symlink-install'
    info "ホストで WebUI の本番ビルドを行う..."
    (cd "$SCRIPT_DIR/web_ui" && npm run build)
fi

if [ ! -d "$SCRIPT_DIR/web_ui/dist" ]; then
    err "WebUI の本番ビルド (web_ui/dist/) が無い。./start.sh --build か ./setup.sh を実行してから起動すること。"
    exit 1
fi

# ── 6. WebUI の配信（bringup の見張りの前に裏で立てる） ──────
info "WebUI を配信する (port ${WEBUI_PORT})..."
( cd "$SCRIPT_DIR/web_ui" && exec npx vite preview --host --port "$WEBUI_PORT" --strictPort ) >/dev/null 2>&1 &
WEBUI_PID=$!
info "タブレット: http://${ROBOT_UI_IP}:${WEBUI_PORT}"
info "PC で見るだけ: http://localhost:${WEBUI_PORT}"

STOPPED=0
cleanup_webui() {
    if [ "${WEBUI_PID:-0}" -ne 0 ] && kill -0 "$WEBUI_PID" 2>/dev/null; then
        kill -TERM "$WEBUI_PID" 2>/dev/null || true
        wait "$WEBUI_PID" 2>/dev/null || true
    fi
}
request_stop() {
    STOPPED=1
    # launch に INT（Ctrl-C 相当）を PID 指定で送る。子ノードごと止まる止め方。
    lp="$(find_launch_pid || true)"
    if [ -n "${lp:-}" ]; then
        info "bringup (PID ${lp}) に INT を送って止める..."
        docker exec th_robot kill -INT "$lp" || true
    fi
}
trap request_stop INT TERM

# ── 5. bringup の起動と見張り ────────────────────────────────
# 終了コードにかかわらず立て直す。bringup は通常自分から正常終了せず、
# restart_control_stack は SIGTERM で落とす作りで ros2 launch は後始末して
# 0 を返しうるため、0 をもって直ったとはみなさない。立て直さないのは
# 操作者の停止（Ctrl-C/SIGTERM をこのスクリプトが受けた）だけ。
# 回数はこのスクリプトが数え、上限に達したら立て直さずに止める
# （Spec-ops.md §2.4「再起動しても直らない場合」）。
attempt=0
while [ "$attempt" -lt "$STARTSH_RESTART_MAX" ]; do
    attempt=$((attempt + 1))
    if [ "$attempt" -eq 1 ]; then
        info "${C_BOLD}bringup を起動します（1 回目／上限 ${STARTSH_RESTART_MAX} 回）${C_RESET}"
    else
        info "${C_BOLD}制御系を再起動しています（${attempt} 回目／上限 ${STARTSH_RESTART_MAX} 回）${C_RESET}"
    fi
    rc=0
    docker exec th_robot bash -lc "$INNER_LAUNCH" || rc=$?
    if [ "$STOPPED" -eq 1 ]; then
        info "操作者の停止により終わる。"
        break
    fi
    if [ "$attempt" -ge "$STARTSH_RESTART_MAX" ]; then
        err "bringup が ${STARTSH_RESTART_MAX} 回起動しても直らなかった。立て直しをやめる。"
        err "機体の電源再投入・AP の確認・ケーブルの確認をしてから、もう一度 ./start.sh を実行すること。"
        cleanup_webui
        trap - INT TERM
        exit 1
    fi
    warn "bringup が終了した（コード ${rc}）。${STARTSH_RESTART_WAIT} 秒待って立て直す..."
    sleep "$STARTSH_RESTART_WAIT"
done

# ── 7. 終了 ─────────────────────────────────────────────────
trap - INT TERM
cleanup_webui
info "終わった。"
