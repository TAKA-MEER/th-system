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
