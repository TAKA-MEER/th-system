"""
slam_control_logic.py
=====================
slam_control.py の pure 関数。ROS 依存なし。
"""

from th_planning.route_record_core import _safe_id


def converge_pause(wanted: bool, statuses) -> "str | None":
    """Pause トグルの収束判定（純関数）。

    `/slam_toolbox/pause_new_measurements` は**引数を受け取らないトグル**で、
    応答の `status` が**切り替えた後の状態**を返す（`status=True` = 一時停止中）。
    望みの状態になるまで**最大 2 回叩く**。

    statuses: これまでに得た応答 `status` のリスト（1 回目、2 回目の順。トグル
    のため、叩くたびに状態が反転していく）。

    戻り値: None = 望みの状態（wanted）に達した（追加の呼び出し不要）。
    それ以外は文字列で、呼び出し元はもう 1 回叩いて statuses に足して再判定する。
    2 回叩いても達しなければ失敗メッセージになる。
    """
    if not statuses:
        return 'retry'   # まだ 1 回も叩いていない（呼び出し元が status を足してくる）
    if any(s == wanted for s in statuses):
        return None
    if len(statuses) >= 2:
        return (f'pause_new_measurements を 2 回叩いても望みの状態にならない '
                f'(wanted paused={wanted}, got {statuses[-1]})')
    return 'retry'   # 1 回叩いて不一致。トグルなので状態が反転しているはず。もう 1 回


def open_session_error(slot: str, mode: str, session_id: str) -> "str | None":
    """/map_session/open の引数検証（純関数）。

    - slot は当面 "ROUTE" のみ受け付ける
    - mode は "save" / "reload" のみ
    - session_id が空は拒否

    問題なければ None、あればエラー文字列を返す。
    """
    if slot != 'ROUTE':
        return f'slot は ROUTE のみ対応 (given {slot!r})'
    if mode not in ('save', 'reload'):
        return f'mode は save/reload のみ対応 (given {mode!r})'
    if not session_id:
        return 'session_id が空'
    return None


def map_session_filename(session_id: str) -> str:
    """セッション ID から地図ファイル名を作る（純関数）。

    **route_record_core._safe_id と同じ規則を使う**。経路 JSON の保存名と地図
    （.posegraph / .data）のファイル名を一致させるため、正規表現を二重に持たない
    （片方だけ変えると、経路名の `/` が保存側だけ置換されて再生できなくなる —
    2026-09-03 に実際に踏んだバグと同じ種類）。

    `/` `\\` は `_` に置換されるため、`../` や `a/b` が混じっても `map_dir` の
    外を指せない（`.._evil` になる）。
    """
    return _safe_id(session_id)
