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
STARTSH_WEBUI_LOG="${STARTSH_WEBUI_LOG:-}"        # WebUI 配信のログ（空なら th_ws/log/start-webui.log）
STARTSH_WEBUI_WAIT="${STARTSH_WEBUI_WAIT:-2}"     # WebUI の生死確認まで待つ秒数
STARTSH_STOP_WAIT="${STARTSH_STOP_WAIT:-30}"      # Ctrl-C 後に launch が止まるのを待つ上限（秒）
STARTSH_PIDFILE="${STARTSH_PIDFILE:-$SCRIPT_DIR/log/start-sh.pid}" # 動作中の start.sh の PID（他での起動の検出用）
# 1b-6 SG-B6: 「運用の終了」（制御系を停止する）の印ファイル。state_manager の
# /shutdown/execute がコンテナ内の /root/th_data（＝下の既定値のホスト側）に置き、
# このスクリプトが bringup の終了後に見て立て直さずに終わる。
STARTSH_STOP_MARKER="${STARTSH_STOP_MARKER:-$SCRIPT_DIR/data/.control_stop}"

# bringup の既定の launch 引数（同じキーが渡されたら渡された方を使う）
DEFAULT_LAUNCH_ARGS=(lidar_source:=network use_stub:=false enable_route_slam:=true stage:=4)

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
      ./start.sh stage:=1          # 教示再生だけの軽い起動（既定は stage:=4）
      ./start.sh enable_route_slam:=false
      ./start.sh --build

オプション:
  --build      コンテナ内で colcon build、ホストで WebUI の本番ビルドを行う
               （既定ではしない。時間がかかるため）
  --dry-run    何も実行せず、これから何をするかだけ表示する
  --yes, -y    他で起動していたとき、確認せずにそれを止めてこちらで起動する
               （既定では確認する。端末が無く --yes も無ければ何も止めずに終わる）
  --help, -h   この表示

launch 引数:
  `key:=value` 形式で bringup.launch.py にそのまま渡す。
  既定（lidar_source:=network use_stub:=false enable_route_slam:=true stage:=4）と
  同じキーがあれば、渡された方が優先される。

環境変数:
  RPI_IP（既定 192.168.5.1）/ RPI_SSH_USER（既定 mirs2602）
  ROBOT_UI_IP（既定 192.168.5.50）/ WEBUI_PORT（既定 5173）
  STARTSH_RESTART_MAX（既定 3。起動は初回を含め最大この回数まで）
  STARTSH_RESTART_WAIT（既定 5 秒。立て直しの前の待ち時間）
  STARTSH_WEBUI_LOG（既定 th_ws/log/start-webui.log。WebUI 配信のログ）
  STARTSH_WEBUI_WAIT（既定 2 秒。WebUI の生死確認まで待つ時間）
  STARTSH_STOP_WAIT（既定 30 秒。Ctrl-C 後に launch が止まるのを待つ上限）
  STARTSH_STOP_MARKER（既定 th_ws/data/.control_stop。「運用の終了」の印ファイル）
  STARTSH_PIDFILE（既定 th_ws/log/start-sh.pid。動作中の start.sh の PID）

止め方: Ctrl-C（launch に INT を送って子ノードごと止め、WebUI の配信も止める）
USAGE
}

# ── 引数の仕分け ────────────────────────────────────────────
DO_BUILD=0
DRY_RUN=0
ASSUME_YES=0
USER_LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --help|-h) usage; exit 0 ;;
        --build) DO_BUILD=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --yes|-y) ASSUME_YES=1 ;;
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

# 1b-6 SG-B7: bringup へ起動回数を渡す（control_attempt:=n。1 回目=1）。
# connectivity_checker が S-00 の「制御系を再起動しています（n 回目）」表示に使う。
# 回数の正本はこのスクリプト（ノードは再起動で消えるため数えられない）。
# 使用者が control_attempt を直接渡したらそちらを優先する。
CONTROL_ATTEMPT_FROM_USER=0
for u in "${USER_LAUNCH_ARGS[@]}"; do
    if [ "${u%%:=*}" = "control_attempt" ]; then CONTROL_ATTEMPT_FROM_USER=1; break; fi
done
inner_launch() {
    local extra=""
    if [ "$CONTROL_ATTEMPT_FROM_USER" -eq 0 ]; then extra=" control_attempt:=${attempt}"; fi
    echo "source /opt/ros/humble/setup.bash && source /root/th_ws/install/setup.bash && exec ros2 launch th_bringup bringup.launch.py ${LAUNCH_ARGS[*]:-}${extra}"
}

if [ "$DRY_RUN" -eq 1 ]; then
    cat <<DRY
[start.sh] --dry-run: 何も実行しない。実行時の手順は次のとおり:
  1. ping -c 3 ${RPI_IP}（届かなければ docs/network.md「復旧手順」を案内して終了）
     ssh ${RPI_SSH_USER}@${RPI_IP} で rpi-serial-relay を確認（失敗は警告のみ）
  2. コンテナ th_robot が無ければ \`docker compose up -d th_robot\`（th_ws/ で）、止まっていれば \`docker start th_robot\`
  3. 他で起動していれば（別の start.sh、またはコンテナ内の bringup）、内容を表示して確認する
     「y」なら他方を止めてから、こちらで起動する（start.sh には TERM、bringup には INT を PID 指定で送り、
     止まるのを上限 ${STARTSH_STOP_WAIT} 秒まで待つ。強制終了はしない）。
     「y」以外・端末なし（--yes 無し）・止まらなかったときは、何も起動せずに終了
DRY
    if [ "$DO_BUILD" -eq 1 ]; then
        echo "  4. --build: コンテナ内で colcon build --symlink-install、ホストで npm run build（web_ui）"
    fi
    cat <<DRY
  5. bringup を起動: docker exec th_robot bash -lc '... exec ros2 launch th_bringup bringup.launch.py <launch 引数> control_attempt:=n'
      起動ごとに control_attempt:=n（n は 1 始まりの起動回数）を付けて渡す。
      終了コードにかかわらず立て直す（最大 ${STARTSH_RESTART_MAX} 回まで。その後は機体の電源再投入・AP・ケーブルの確認を案内）。
      立て直さないのは操作者の停止（Ctrl-C/SIGTERM）と「運用の終了」の停止要求
      （印ファイル ${STARTSH_STOP_MARKER}。/shutdown/execute が置く）だけ
  6. WebUI を配信: web_ui/dist/ を npx vite preview --host --port ${WEBUI_PORT} --strictPort で配信
     出力はログ（既定 th_ws/log/start-webui.log。git 管理外）に残し、起動直後に死んでいたら止まる
     タブレット: http://${ROBOT_UI_IP}:${WEBUI_PORT} ／ PC: http://localhost:${WEBUI_PORT}
  7. Ctrl-C で launch に INT を送って子ノードごと止め（止まるのを上限 30 秒まで待つ）、
     WebUI の配信も止めて終わる
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

# ── 3. 他で起動している場合の確認と引き継ぎ ──────────────────
# コンテナ内のプロセス一覧をホスト側で調べる。名前での一括停止は
# 自分のシェルを殺すことがあるため使わず、PID を特定するだけに留める。
find_launch_pid() {
    docker exec th_robot ps -eo pid,args 2>/dev/null \
        | awk '$0 ~ /bringup\.launch\.py/ && $0 ~ /ros2/ {print $1; exit}'
}

# 他で動いている start.sh の PID（pidfile から。自分は除く。PID の使い回しは cmdline で除く）
find_other_start_pid() {
    local pid
    pid="$(cat "$STARTSH_PIDFILE" 2>/dev/null || true)"
    case "$pid" in ''|*[!0-9]*) return 0 ;; esac
    [ "$pid" -ne "$$" ] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    if tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | grep -q 'start\.sh'; then
        echo "$pid"
    fi
}

# 他で起動しているものを、確認のうえ止める。止められなければ非0を返す。
takeover_existing() {
    local other_start="$1" other_launch="$2" ans waited running
    warn "他で起動している:"
    if [ -n "$other_start" ]; then
        warn "  - 別の start.sh (PID ${other_start})"
    fi
    if [ -n "$other_launch" ]; then
        warn "  - コンテナ内の bringup (PID ${other_launch})"
    fi
    warn "誰かが実機作業中の可能性がある。止めると、その作業は中断される。"
    if [ "$ASSUME_YES" -eq 1 ]; then
        info "--yes が指定されているので確認せずに進める。"
    elif [ -t 0 ]; then
        printf '%s' "止めて、こちらで起動しますか？ [y/N] "
        read -r ans || ans=""
        case "$ans" in
            y|Y|yes|YES) ;;
            *) err "止めないので、何もせずに終わる。"; return 1 ;;
        esac
    else
        err "端末から確認できない（--yes も無い）ので、何も止めずに終わる。"
        return 1
    fi

    # 別の start.sh には TERM を先に送り、続けて launch に INT を送る。
    # 相手は前景の docker exec が終わるまで trap を保留するので、launch を止めて
    # docker exec が終わった時点で「操作者の停止」として扱われ、立て直しに入らない。
    # start.sh 経由でない bringup（手作業の起動）も同じ INT で止まる。
    if [ -n "$other_start" ]; then
        info "start.sh (PID ${other_start}) に TERM を送る..."
        kill -TERM "$other_start" 2>/dev/null || true
    fi
    if [ -n "$other_launch" ]; then
        info "bringup (PID ${other_launch}) に INT を送って止める..."
        docker exec th_robot kill -INT "$other_launch" || true
    fi
    waited=0
    while [ "$waited" -lt $((STARTSH_STOP_WAIT + 10)) ]; do
        running=0
        if [ -n "$other_start" ] && kill -0 "$other_start" 2>/dev/null; then running=1; fi
        if [ -n "$(find_launch_pid || true)" ]; then running=1; fi
        [ "$running" -eq 1 ] || break
        sleep 1; waited=$((waited + 1))
    done
    if [ -n "$other_start" ] && kill -0 "$other_start" 2>/dev/null; then
        err "start.sh (PID ${other_start}) が止まらなかった。強制終了はしない。止まってから打ち直すこと。"
        return 1
    fi
    if [ -n "$(find_launch_pid || true)" ]; then
        err "bringup が止まらなかった。強制終了はしない。止まってから打ち直すこと。"
        return 1
    fi
    info "他で動いていたものが止まった。"
    return 0
}

OTHER_START_PID="$(find_other_start_pid || true)"
EXISTING_PID="$(find_launch_pid || true)"
if [ -n "${OTHER_START_PID:-}" ] || [ -n "${EXISTING_PID:-}" ]; then
    takeover_existing "${OTHER_START_PID:-}" "${EXISTING_PID:-}" || exit 1
fi

# 自分の PID を残す（次に起動した start.sh が、他で起動していると気づけるように）
mkdir -p "$(dirname "$STARTSH_PIDFILE")"
echo "$$" >"$STARTSH_PIDFILE"
remove_pidfile() {
    if [ "$(cat "$STARTSH_PIDFILE" 2>/dev/null || true)" = "$$" ]; then rm -f "$STARTSH_PIDFILE"; fi
}
trap remove_pidfile EXIT

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

# WebUI の生死待ち・立て直しの待ち時間中の停止に備え、ここで trap を入れる
#（sleep 中の INT に trap が無いと即死する）。
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
        # 子ノードの後始末には数秒かかる。止まったのを見届けてから終わる。
        # すぐ打ち直すと「二重起動」で拒否されるため。強制終了はしない。
        # 前提: 前景の docker exec は INT で終わるので、この trap が走る。
        # launch 側が INT を無視して残り続けると、docker exec が終わるまで
        # ここには来ない（bash は前景の完了まで trap を遅延させる）。
        waited=0
        while [ "$waited" -lt "$STARTSH_STOP_WAIT" ]; do
            lp="$(find_launch_pid || true)"
            if [ -z "${lp:-}" ]; then
                info "bringup が止まった。"
                return 0
            fi
            last_pid="$lp"
            sleep 1 || true  # 2 回目の INT でも落ちないよう受け止める
            waited=$((waited + 1))
        done
        warn "bringup (PID ${last_pid}) がまだ止まっていない。そのまま終わる（強制終了はしない）。"
        warn "止まってから打ち直すこと。止まらないときは作業者に確認する。"
    fi
}

trap request_stop INT TERM

# ── 6. WebUI の配信（bringup の見張りの前に裏で立てる） ──────
# 出力はログファイルに残す（th_ws/log/ は git 管理外）。起動直後に死んで
# いたら（ポートが埋まっている等。--strictPort は競合で即終了する）、
# ログの末尾を出して止める。bringup はまだ起動していないので止めるものは無い。
info "WebUI を配信する (port ${WEBUI_PORT})..."
if [ -z "$STARTSH_WEBUI_LOG" ]; then
    STARTSH_WEBUI_LOG="$SCRIPT_DIR/log/start-webui.log"
fi
mkdir -p "$(dirname "$STARTSH_WEBUI_LOG")"
( cd "$SCRIPT_DIR/web_ui" && exec npx vite preview --host --port "$WEBUI_PORT" --strictPort ) >>"$STARTSH_WEBUI_LOG" 2>&1 &
WEBUI_PID=$!
# INT で中断されると sleep は非0で終わる（set -e で落ちないよう受け止める）。
sleep "$STARTSH_WEBUI_WAIT" || true
if [ "$STOPPED" -eq 1 ]; then
    info "操作者の停止により終わる。"
    cleanup_webui
    trap - INT TERM
    exit 0
fi
webui_dead=0
if ! kill -0 "$WEBUI_PID" 2>/dev/null; then
    webui_dead=1
elif [ -r "/proc/$WEBUI_PID/stat" ] && [ "$(awk '{print $3}' "/proc/$WEBUI_PID/stat")" = "Z" ]; then
    webui_dead=1  # 終了済みで回収待ち（ゾンビ）。kill -0 では生きているように見える
fi
if [ "$webui_dead" -eq 1 ]; then
    wait "$WEBUI_PID" 2>/dev/null || true  # 終わっていた分を回収
    err "WebUI の配信が起動しなかった（port ${WEBUI_PORT} が埋まっている等）。ログ (${STARTSH_WEBUI_LOG}) の末尾:"
    tail -n 20 "$STARTSH_WEBUI_LOG" >&2 || true
    err "ポートを使っているものを止めるか、WEBUI_PORT を変えて起動すること。"
    exit 1
fi
info "タブレット: http://${ROBOT_UI_IP}:${WEBUI_PORT}"
info "PC で見るだけ: http://localhost:${WEBUI_PORT}"

# ── 5. bringup の起動と見張り ────────────────────────────────
# 終了コードにかかわらず立て直す。bringup は通常自分から正常終了せず、
# restart_control_stack は SIGTERM で落とす作りで ros2 launch は後始末して
# 0 を返しうるため、0 をもって直ったとはみなさない。立て直さないのは
# 操作者の停止（Ctrl-C/SIGTERM をこのスクリプトが受けた）と「運用の終了」の
# 停止要求（印ファイル。/shutdown/execute が置く）だけ。
# 回数はこのスクリプトが数え、上限に達したら立て直さずに止める
# （Spec-ops.md §2.4「再起動しても直らない場合」）。
# 1b-6 SG-B7: 回数は bringup へ control_attempt:=n として渡す（S-00 の表示用）。
attempt=0
# 前回の「運用の終了」の印が残っていたら消してから始める（前回は止めるつもりで
# 印を置いたままこのスクリプトが異常終了した場合の残骸。新規起動では無効）。
if [ -f "$STARTSH_STOP_MARKER" ]; then
    info "古い停止要求の印 (${STARTSH_STOP_MARKER}) を消してから始める。"
    rm -f "$STARTSH_STOP_MARKER"
fi
while [ "$attempt" -lt "$STARTSH_RESTART_MAX" ]; do
    if [ "$STOPPED" -eq 1 ]; then
        info "操作者の停止により終わる。"
        break
    fi
    attempt=$((attempt + 1))
    if [ "$attempt" -eq 1 ]; then
        info "${C_BOLD}bringup を起動します（1 回目／上限 ${STARTSH_RESTART_MAX} 回）${C_RESET}"
    else
        info "${C_BOLD}制御系を再起動しています（${attempt} 回目／上限 ${STARTSH_RESTART_MAX} 回）${C_RESET}"
    fi
    rc=0
    docker exec th_robot bash -lc "$(inner_launch)" || rc=$?
    if [ "$STOPPED" -eq 1 ]; then
        info "操作者の停止により終わる。"
        break
    fi
    # 1b-6 SG-B6: 「運用の終了」（S-01 の「制御系を停止する」→ /shutdown/execute）
    # の停止要求があれば、立て直さずに終わる（正常終了）。
    if [ -f "$STARTSH_STOP_MARKER" ]; then
        rm -f "$STARTSH_STOP_MARKER"
        info "運用の終了の停止要求により終わる。"
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
    # INT で中断されると sleep は非0で終わる（set -e で落ちないよう受け止める）。
    # 待ち時間中の停止は次の起動に進まず抜ける。
    sleep "$STARTSH_RESTART_WAIT" || true
    if [ "$STOPPED" -eq 1 ]; then
        info "操作者の停止により終わる。"
        break
    fi
done

# ── 7. 終了 ─────────────────────────────────────────────────
trap - INT TERM
cleanup_webui
info "終わった。"
