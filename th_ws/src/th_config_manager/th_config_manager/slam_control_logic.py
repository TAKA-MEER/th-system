"""
slam_control_logic.py
=====================
slam_control.py の pure 関数。ROS 依存なし。
"""

import re

# 経路名・セッション ID として不正な文字。送る側（経路記録側）は
# route_record_core._safe_id で `/` `\` を `_` に正規化して送ってくる。ここは
# 受け取った id が**未正規化でないか**を検証する（検証はしつつ変換はしない。
# サニタイズを 2 か所に持つと片方だけ変えて食い違うため、「送る側が正規化し、
# 受ける側は不正なら拒否」に役割を分ける）。
_UNSAFE_ID_RE = re.compile(r'[/\\]')
_TRAVERSAL_RE = re.compile(r'\.\.')


def deserialize_match_type(has_initial_pose: bool) -> int:
    """deserialize_map に渡す match_type を決める（純関数）。

    - has_initial_pose=True: 3 (LOCALIZE_AT_POSE)
      経路の始点 (initial_x, initial_y, initial_yaw) を初期姿勢として
      slam_toolbox に渡し、その位置でスキャンマッチングさせて自己位置を推定する。
      実機確認 (2026-09-03): match_type 3 + initial_pose で読み込み成功。
      読み込み後の map->base_link は始点との差 1.9 cm / 5.5° で整合し、
      SIGSEGV も発生しない。
    - has_initial_pose=False: 1 (START_AT_FIRST_NODE)
      初期姿勢が無い場合は地図の最初のノードで開始するフォールバック。
    """
    return 3 if has_initial_pose else 1


def slam_restart_complete(old_pids, current_pids) -> bool:
    """respawn 後、slam_toolbox の PID が総入れ替わりしたか（純関数）。

    WS-9S: 再生の地図読み直しは毎回 slam_toolbox を SIGTERM → launch の
    `respawn=True` で作り直してから `deserialize_map` する。旧プロセスが消えて
    新プロセスが立つまで待つ判定に使う。

    - current_pids が空 → まだ立ち上がっていない（False）
    - current_pids が old_pids と 1 つでも重なる → まだ旧プロセスが残っている（False）
    - current_pids が非空で old_pids と互いに素 → 総入れ替わり済み（True）
    """
    cur = set(current_pids)
    if not cur:
        return False
    return cur.isdisjoint(set(old_pids))


def open_session_error(slot: str, mode: str, session_id: str) -> "str | None":
    """/map_session/open の引数検証（純関数）。

    - slot は "ROUTE"（経路地図）と "VENUE"（試験場内地図）のみ受け付ける
    - mode は "save" / "reload" のみ
    - session_id が空は拒否
    - session_id に `/` `\\` `..` が含まれる（未正規化）は拒否

    問題なければ None、あればエラー文字列を返す。
    """
    if slot not in ('ROUTE', 'VENUE'):
        return f'slot は ROUTE / VENUE のみ対応 (given {slot!r})'
    if mode not in ('save', 'reload'):
        return f'mode は save/reload のみ対応 (given {mode!r})'
    if not session_id:
        return 'session_id が空'
    if _UNSAFE_ID_RE.search(session_id):
        return (f'session_id に `/` か `\\` が含まれる（未正規化）: '
                f'{session_id!r}')
    if _TRAVERSAL_RE.search(session_id):
        return f'session_id に `..` が含まれる（不正）: {session_id!r}'
    return None


def map_session_filename(session_id: str) -> str:
    """セッション ID から地図ファイル名を作る（純関数）。

    呼び出し元（/map_session/open ハンドラ）は open_session_error で検証済みの
    id だけを渡す（変換はしない）。ここはそのまま join に使える値を返す。
    """
    return session_id


def map_session_base_dir(slot: str, route_map_dir: str, venue_map_dir: str) -> str:
    """slot に応じた地図の保存先ディレクトリを返す（純関数）。

    'VENUE' なら venue_map_dir（試験場内地図）、それ以外（'ROUTE'）なら
    route_map_dir（経路地図）を返す。slot は open_session_error で検証済み前提。
    """
    if slot == 'VENUE':
        return venue_map_dir
    return route_map_dir


def map_session_name(slot: str, session_id: str) -> str:
    """slot に応じた地図ファイル名（拡張子なし）を返す（純関数）。

    'VENUE' は 'map' 固定（1 枚のみ保持。CL-M-9）。それ以外（'ROUTE'）は
    map_session_filename(session_id)。id は open_session_error で検証済み前提。
    """
    if slot == 'VENUE':
        return 'map'
    return map_session_filename(session_id)


def effective_reload_pose(has_initial_pose: bool, initial_x: float, initial_y: float,
                          initial_yaw: float, home_pose) -> "tuple":
    """VENUE の reload で使う実際の初期姿勢を決める（純関数）。

    実機事故 (2026-09-09・WS-9Y): 試験画面（VENUE）は has_initial_pose=false を
    固定送信していた。deserialize_match_type(False) は match_type=1
    (START_AT_FIRST_NODE) にフォールバックし、機体の実際の現在地とは無関係に
    「ポーズグラフの最初のノードにいる」と決め打ちで自己位置推定を始める。
    ずれた自己位置のまま /map の静的レイヤが重なり、costmap 上に実在しない
    障害物が出て ComputePathToPose が失敗し続け、ピンへ一歩も動かなくなった
    （実機確認: map->base_link と HOME ピンが 0.8m ずれ、/cmd_vel が完全に無音）。

    呼び出し側が既に has_initial_pose=True で明示的な姿勢を渡しているなら
    それを優先する（将来 VENUE 以外や別の呼び出し元が明示指定してきても壊さない）。
    呼び出し側が指定せず（False）、登録済み HOME ピンの姿勢（home_pose、
    (x, y, yaw) または None）があれば、それを初期姿勢として使う
    （match_type=3 LOCALIZE_AT_POSE に切り替わる。deserialize_match_type 経由）。
    HOME ピンも無ければ何もしない（False のまま。従来どおり START_AT_FIRST_NODE）。

    Returns: (has_initial_pose, x, y, yaw)
    """
    if has_initial_pose:
        return (True, initial_x, initial_y, initial_yaw)
    if home_pose is not None:
        hx, hy, hyaw = home_pose
        return (True, hx, hy, hyaw)
    return (False, initial_x, initial_y, initial_yaw)


def map_instance_ids_match(map_instance_id: str, pins_instance_id: str) -> bool:
    """WS-9AL(2026-09-11): 読み込む地図の instance_id と、ピンが最後に保存
    された時点の instance_id が同じ地図の生存世代を指しているかを判定する
    （純関数）。

    実機事故 (2026-09-11): bringup を再起動するたび SLAM は無関係な新しい
    座標系でまっさらに始まり直すが、pins.yaml はファイルとして永続化されて
    いて次回起動時にそのまま再利用される。地図を保存し直さないまま再起動を
    挟むと、ピンの数値は「もう存在しない古い座標系」のまま残り、次に地図を
    開いたとき HOME ピンの姿勢を無条件で信用してしまい、配電盤・待機場所・
    自機と地図がズレた（本人確認: map.data は古い時刻、pins.yaml はそれより
    後の時刻に更新されていた）。

    どちらか一方でも空（instance_id が付く前の古い保存・未登録）なら
    「確認できない」ので False（安全側 = HOME ピンの姿勢を信用しない）。
    """
    if not map_instance_id or not pins_instance_id:
        return False
    return map_instance_id == pins_instance_id
