"""
test_msg_definitions.py
==========================
WP-MSG-01 §7 単体試験 — th_system_msgs に新設した msg/srv の定義を検査する。

DetailedDesign-wp0.md `WP-MSG-01` §7:
  - test_all_new_msgs_importable  : names.md §5.1 の全 msg が Python から import できる
  - test_no_uint8_mode_constants  : M1。SystemState に uint8 定数が無い
  - test_legacy_msgs_untouched    : M3。旧 msg のファイルハッシュが変わっていない
                                     （FaultStatus.msg のみ末尾 1 行追加を許容）
  - test_fields_match_names_md    : names.md §5.1 の表を機械読みして突き合わせる

このテストは生成済みの th_system_msgs (colcon build 済み・source install/setup.bash
済み) が要る。ROS2 ノードは起動しない。
"""

import hashlib
import importlib
import os
import re

import pytest


_THIS_DIR = os.path.dirname(__file__)
_TH_SYSTEM_MSGS_ROOT = os.path.abspath(
    os.path.join(_THIS_DIR, '..', '..', 'th_system_msgs'))

# 設計書のパス。ホスト（リポジトリ直下から辿る）とコンテナ内（/docs に
# 読み取り専用マウント。docker-compose.yml 参照）の両方を試す。
_NAMES_MD_CANDIDATES = [
    os.path.abspath(os.path.join(
        _THIS_DIR, '..', '..', '..', '..',
        'docs', 'plan', 'detailed', 'DetailedDesign-names.md')),
    '/docs/plan/detailed/DetailedDesign-names.md',
]


def _names_md_path():
    for p in _NAMES_MD_CANDIDATES:
        if os.path.isfile(p):
            return p
    pytest.skip('DetailedDesign-names.md が見つからない（試した経路: %s）'
                % _NAMES_MD_CANDIDATES)


# names.md §5.1 の新 msg 一覧（reuse.md §2.1 の「(新設)」行と一致。16 件）。
# FaultStatus.msg（改修のみ）と WheelFeedback.msg（既存のまま）はここに含めない。
NEW_MSGS = [
    'SystemState', 'StateEvent', 'ActiveScreen', 'LimiterStatus',
    'WaitClearStatus', 'RouteList', 'LinkQuality', 'ParamsStatus',
    'PersonTargets', 'Pin', 'PinList', 'RouteInfo', 'RouteStatus',
    'MapSessionStatus', 'CheckStatus', 'CalibStatus',
]

# M3: 既存の 9 msg・5 srv のうち FaultStatus.msg 以外はバイト単位で不変。
# ハッシュはこのパケット着手前 (git HEAD) の内容から採取した
# (`git show HEAD:<path> | sha256sum`)。
_LEGACY_MSG_HASHES = {
    'FollowStatus.msg':  'd56a4ccf992548dcd5ee84ca1cd227e1a21288fd4196746c7f9243cccf896b36',
    'PanelArrival.msg':  '4862daa531968ddc7bce67596bba6a633101a42c5d00ca332b93b2b516851034',
    'PersonStatus.msg':  'ee9890b735caca61a3242ee8122c988e8413062ab219a71c761dcb4f51640036',
    'RobotMode.msg':     'c17690ddb264c0c524ef9e6eef10174cfb997cef809802bf9575db00183734bc',
    'SearchStatus.msg':  '6a86c3ffd152d40902d8ebc653c554ff271e3d4d436f0ce820811f9c7afcf9d3',
    'SummonStatus.msg':  'f1a806060672f0a4425f2d0fa20ea3d7b8c00ed6c0143cd468a9368796c13514',
    'WheelCommand.msg':  '11c7ea5a8503b420975c0521972bde47b6955647be7808d45c8c5494f1b6ca72',
    'WheelFeedback.msg': '73bcf457c804028681d664c35c43e5b98bfc6a0b79eb2bfe568c334249d18857',
}
_LEGACY_SRV_HASHES = {
    'CompleteInspection.srv': '8baf3f156636f12ba8e897875168b8ad1591353f37c7f2be54ecd081bea28afd',
    'GoToPanel.srv':          '47afe78c3ecabb6070989287deeeec748a015e022642dcd241f2fa021a62cd41',
    'SaveTunableParams.srv':  '7dba01ecdeb102b7d3aac913c94f46d862104d6a9ae99722e5881ef5566a6602',
    'SetMode.srv':            'ca58011dd82bcdebedd0de9bc1420432420e1d95a9ba461788434e20d7eaaa88',
    'SetTunableParams.srv':   'e099fba06599423590c2608c74dc12de206c9971e8f36da128ba8e8ece01b1bb',
}

# names.md §5.1 の表の 1 セル分（backtick で囲まれた「型 名前」）を拾う正規表現。
# 単一トークンの backtick 片（enum の値など。例: `IN` / `esp32`）は
# 「空白を含まない」ため素通りする。
_FIELD_RE = re.compile(
    r'`([A-Za-z_]\w*(?:/[A-Za-z_]\w*)?(?:\[\])?)\s+([a-zA-Z_]\w*)`')


def _sha256_of(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _parse_msg_fields(path):
    """.msg / .srv の request 側などの「型 名前」を出現順に取り出す。

    コメント (#...) と空行は無視する。request/response の区切り (---) や
    定数行 (= を含む) は本パケットの新設ファイルには出てこないため考慮しない。
    """
    fields = []
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            line = line.split('#', 1)[0].strip()
            if not line or line.startswith('---'):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            fields.append((parts[0], parts[1]))
    return fields


def _normalize_type(t):
    """パッケージ接頭辞の有無を吸収する（例: std_msgs/Header と Header は同一視）。"""
    return t.split('/')[-1]


def _extract_expected_fields(md_text, msg_name):
    needle = '`%s.msg`' % msg_name
    for line in md_text.splitlines():
        if needle in line and line.strip().startswith('|'):
            return _FIELD_RE.findall(line)
    return []


# ── test_all_new_msgs_importable ────────────────────────────────────────

def test_all_new_msgs_importable():
    msg_mod = importlib.import_module('th_system_msgs.msg')
    missing = [name for name in NEW_MSGS if not hasattr(msg_mod, name)]
    assert not missing, 'import できない新 msg: %s' % missing


# ── test_no_uint8_mode_constants (M1) ───────────────────────────────────

def test_no_uint8_mode_constants():
    from th_system_msgs.msg import SystemState

    field_types = SystemState.get_fields_and_field_types()
    assert field_types['mode'] == 'string'
    assert field_types['state'] == 'string'

    # RobotMode.msg で使われていた uint8 定数名が SystemState には
    # 再現されていないことを確認する（M1: 「RobotMode.msg の失敗を繰り返さない」）。
    legacy_uint8_constant_names = [
        'INIT', 'IDLE', 'FOLLOWING', 'MOVING_TO_PANEL', 'AT_PANEL',
        'MANUAL', 'ESTOP', 'FOLLOWING_MAPLESS', 'SUMMONING',
    ]
    leaked = [n for n in legacy_uint8_constant_names if hasattr(SystemState, n)]
    assert not leaked, (
        'SystemState に uint8 定数が残っている (M1 違反): %s' % leaked)


# ── test_legacy_msgs_untouched (M3) ─────────────────────────────────────

def test_legacy_msgs_untouched():
    mismatched = []
    for fname, expected_hash in _LEGACY_MSG_HASHES.items():
        path = os.path.join(_TH_SYSTEM_MSGS_ROOT, 'msg', fname)
        if _sha256_of(path) != expected_hash:
            mismatched.append(fname)
    for fname, expected_hash in _LEGACY_SRV_HASHES.items():
        path = os.path.join(_TH_SYSTEM_MSGS_ROOT, 'srv', fname)
        if _sha256_of(path) != expected_hash:
            mismatched.append(fname)
    assert not mismatched, (
        '旧 msg/srv のファイルが変更されている (M3 違反): %s' % mismatched)

    # FaultStatus.msg だけは唯一の例外。既存 3 フィールドの順序・型はそのまま、
    # 末尾に string severity が 1 行だけ増えていること。
    fault_path = os.path.join(_TH_SYSTEM_MSGS_ROOT, 'msg', 'FaultStatus.msg')
    fields = _parse_msg_fields(fault_path)
    assert fields[:3] == [
        ('bool', 'active'), ('string', 'fault_type'), ('string', 'description'),
    ], 'FaultStatus.msg の既存フィールドの順序または型が変わっている (M3 違反)'
    assert len(fields) == 4, (
        'FaultStatus.msg は severity 1 行だけが末尾に追加されているはず: %s' % (fields,))
    assert fields[3] == ('string', 'severity'), (
        'FaultStatus.msg の末尾フィールドは string severity のはず: %s' % (fields[3],))


# ── test_fields_match_names_md ──────────────────────────────────────────

def test_fields_match_names_md():
    md_path = _names_md_path()
    with open(md_path, encoding='utf-8') as fh:
        md_text = fh.read()

    mismatches = {}
    for name in NEW_MSGS:
        expected = _extract_expected_fields(md_text, name)
        assert expected, 'names.md に %s.msg の表が見つからない' % name

        msg_path = os.path.join(_TH_SYSTEM_MSGS_ROOT, 'msg', '%s.msg' % name)
        actual = _parse_msg_fields(msg_path)

        expected_norm = [(_normalize_type(t), n) for t, n in expected]
        actual_norm = [(_normalize_type(t), n) for t, n in actual]

        if actual_norm != expected_norm:
            mismatches[name] = {'names.md': expected_norm, 'msg 実装': actual_norm}

    assert not mismatches, (
        'names.md §5.1 と実装のフィールドが食い違う: %s' % mismatches)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
