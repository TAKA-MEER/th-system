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
    - session_id に `/` `\\` `..` が含まれる（未正規化）は拒否

    問題なければ None、あればエラー文字列を返す。
    """
    if slot != 'ROUTE':
        return f'slot は ROUTE のみ対応 (given {slot!r})'
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
