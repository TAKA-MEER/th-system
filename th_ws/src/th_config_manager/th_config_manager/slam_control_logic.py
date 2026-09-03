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


def pause_toggle_needed(tracked_paused: bool, wanted_paused: bool) -> bool:
    """一時停止のトグルを叩く必要があるか（純関数）。

    `/slam_toolbox/pause_new_measurements` は引数を取らないトグルで、応答の status は
    常に True（＝呼べた）を返すだけで現在の状態を教えてくれない（実機で 3 回連続
    呼んで 3 回とも True。2026-09-03）。したがって状態は呼び出し側が持つしかない。

    旧 converge_pause は「status は切り替え後の状態を返す」前提で最大 2 回叩いて
    いたが、その前提が誤りで、必ず 2 回叩いて望みと逆の状態
    （wanted paused=False, got True）で終わっていた。
    """
    return tracked_paused != wanted_paused


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
