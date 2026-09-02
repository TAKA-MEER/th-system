"""
test_route_record_files.py
===========================
経路ファイルの保存系（WS-9H）のテスト。route_record_core の純関数
（save_route_atomic / finalize_route_file / autosave_path / previous_path /
list_finalized_route_files）を、実ファイルで検証する。

ROS2 なし・純粋 Python。pytest の tmp_path フィクスチャを使って実ディレクトリを
作る。

背景: 教示は 10 分前後かかる。いまは finalize（画面「保存」）の 1 回だけしか
ファイルに書かず、その間の点列はメモリにしか無い（落ちると 10 分の成果が消える）。
さらに同名で保存し直すと旧版 <id>.json が丸ごと消える（EXCEPTION-LEDGER W-04）。
ここでは「途中経過の .wip 自動保存」「旧版の .prev 退避」「原子的書き込み」を固定する。
"""

import json
import os
import sys

import pytest

# ── パスを通す（colcon build 前にも直接 pytest できるように）
sys.path.insert(0, os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'th_planning', 'th_planning'))

from route_record_core import (
    autosave_path, previous_path, finalized_path,
    save_route_atomic, finalize_route_file, list_finalized_route_files,
)


def _mkroutes(tmp_path):
    d = tmp_path / 'routes'
    d.mkdir()
    return str(d)


def test_save_route_atomic_writes_readable_json(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    dest = finalized_path(routes_dir, 'r')
    data = {'id': 'r', 'points': [[0.0, 0.0, 0.0], [1.0, 1.0, 0.1]]}
    save_route_atomic(dest, data)
    assert json.load(open(dest, encoding='utf-8')) == data


def test_save_route_atomic_uses_temp_then_replace(tmp_path, monkeypatch):
    """一時ファイルに書いてから os.replace で差し替える（半分書きの JSON を読ませない）。

    変更: save_route_atomic を「本体へ直接 open して書く」に変えると、os.replace が
    呼ばれず、このテストが落ちる（変異チェック 2）。
    """
    import route_record_core as m
    dest = finalized_path(str(tmp_path), 'r')
    calls = []
    real_replace = m.os.replace

    def fake_replace(src, dst):
        calls.append((src, dst))
        real_replace(src, dst)

    monkeypatch.setattr(m.os, 'replace', fake_replace)
    save_route_atomic(dest, {'id': 'r'})
    assert len(calls) == 1, 'os.replace が 1 回だけ呼ばれること（原子的差し替え）'
    _src, dst = calls[0]
    assert dst == dest                     # 差し替え先は本体
    assert _src != dest                    # 元は一時ファイル（名前が本体と異なる）
    assert os.path.basename(_src).startswith('.')   # 一時ファイルは隠し名


def test_save_route_atomic_leaves_no_partial_or_temp(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    dest = finalized_path(routes_dir, 'r')
    save_route_atomic(dest, {'id': 'r'})
    # 書き込み直後は完成品の .json だけが残り、一時ファイルが残らない
    assert sorted(os.listdir(routes_dir)) == ['r.json']
    assert json.load(open(dest, encoding='utf-8')) == {'id': 'r'}


def test_finalize_route_file_backs_up_existing_to_prev(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    d1 = {'id': 'r', 'points': [[0.0, 0.0, 0.0]]}
    d2 = {'id': 'r', 'points': [[9.0, 9.0, 0.0]]}
    finalize_route_file(routes_dir, 'r', d1)
    finalize_route_file(routes_dir, 'r', d2)
    assert json.load(open(finalized_path(routes_dir, 'r'), encoding='utf-8')) == d2
    assert json.load(open(previous_path(routes_dir, 'r'), encoding='utf-8')) == d1


def test_finalize_route_file_keeps_only_latest_and_one_previous(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    finalize_route_file(routes_dir, 'r', {'g': 1})
    finalize_route_file(routes_dir, 'r', {'g': 2})
    finalize_route_file(routes_dir, 'r', {'g': 3})
    # 最新と 1 つ前の 2 つだけが残る（世代は積み上がらない）
    assert sorted(os.listdir(routes_dir)) == ['r.json', 'r.prev']
    assert json.load(open(finalized_path(routes_dir, 'r'), encoding='utf-8')) == {'g': 3}
    assert json.load(open(previous_path(routes_dir, 'r'), encoding='utf-8')) == {'g': 2}


def test_finalize_route_file_removes_leftover_autosave(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    wip = routes_dir + '/r.wip'
    with open(wip, 'w', encoding='utf-8') as f:
        json.dump({'id': 'r'}, f)
    finalize_route_file(routes_dir, 'r', {'id': 'r'})
    assert not os.path.exists(wip)                      # 途中経過は消える
    assert os.path.exists(finalized_path(routes_dir, 'r'))


def test_catalog_list_excludes_wip_and_prev(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    # 完成品だけを一覧に載せ、途中経過(.wip)・1 世代前(.prev)は出さない。
    for fname in ('a.json', 'b.wip', 'c.prev', 'd.wip'):
        with open(os.path.join(routes_dir, fname), 'w', encoding='utf-8') as f:
            f.write('{}')
    assert list_finalized_route_files(routes_dir) == ['a.json']


def test_dotdot_and_slash_do_not_escape_routes_dir(tmp_path):
    base = str(tmp_path / 'base')
    os.makedirs(base)
    # `../evil` や `a/b` は routes_dir の外へ書けない
    finalize_route_file(base, '../evil', {'id': 'evil'})
    finalize_route_file(base, 'a/b', {'id': 'b'})
    made = os.listdir(base)
    # 作られたファイル名はすべて平ら（パス区切りを含まない＝サブディレクトリへも
    # 外へも逃げていない）。.. は名前の一部として字面に残ることはあるが、区切りが
    # 無ければ base の外には書けない。
    assert all('/' not in n and '\\' not in n for n in made)
    # tmp_path 直下（base の外）に evil 系のファイルは無い
    assert not os.path.exists(os.path.join(str(tmp_path), 'evil.json'))
    assert not os.path.exists(os.path.join(str(tmp_path), 'evil'))
    assert all(os.path.dirname(os.path.join(base, n)) == base for n in made)
    # 無害化した名前に .json で保存されている
    assert os.path.exists(finalized_path(base, '../evil'))
    assert os.path.exists(finalized_path(base, 'a/b'))
    assert 'a_b.json' in made


def test_autosave_path_sanitizes_dotdot(tmp_path):
    routes_dir = _mkroutes(tmp_path)
    p = autosave_path(routes_dir, '../../evil')
    assert p.startswith(routes_dir)
    assert '/../' not in '/' + p
