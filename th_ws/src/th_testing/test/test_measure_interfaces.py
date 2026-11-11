"""
test_measure_interfaces.py
==========================
測定機能（fk-system）とのやり取りの型 5 つが、詳細設計
docs/plan/detailed/DetailedDesign-measure.md §3 と一致していることを縛る。

fk-system は th_system_msgs を本システムのリポジトリから取り込んでビルドする
（同 §1.2）。ROS 2 Humble には型の照合が無いので、型が設計書から黙ってずれると
fk 側は気づけない。型を直すときは設計書を先に直し、interface_version を上げるか
fk に知らせる。

ROS 2 不要（ホストの python3 -m pytest で走る）。生成物の import は見ない。
"""

import os
import re

import pytest


_THIS_DIR = os.path.dirname(__file__)
_MSGS_ROOT = os.path.abspath(os.path.join(_THIS_DIR, '..', '..', 'th_system_msgs'))
_DOC_CANDIDATES = [
    os.path.abspath(os.path.join(
        _THIS_DIR, '..', '..', '..', '..',
        'docs', 'plan', 'detailed', 'DetailedDesign-measure.md')),
    '/docs/plan/detailed/DetailedDesign-measure.md',
]

INTERFACES = [
    ('MeasureCommand', 'srv'),
    ('MeasureResult', 'msg'),
    ('MeasureStatus', 'msg'),
    ('MeasureHeartbeat', 'msg'),
    ('CheckArmClearance', 'srv'),
]


def _doc_text():
    for p in _DOC_CANDIDATES:
        if os.path.isfile(p):
            with open(p, encoding='utf-8') as f:
                return f.read()
    pytest.skip('DetailedDesign-measure.md が見つからない（試した経路: %s）'
                % _DOC_CANDIDATES)


def _fields(text):
    """コメントと空行を除き、(型, 名前) と区切り '---' の列にする。"""
    out = []
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        if line == '---':
            out.append(('---',))
            continue
        out.append(tuple(line.split()[:2]))
    return out


def _doc_block(doc, name, kind):
    m = re.search(r'### 3\.\d `%s\.%s`.*?\n```\n(.*?)```' % (name, kind), doc, re.S)
    assert m, '設計書 §3 に %s.%s の定義ブロックが無い' % (name, kind)
    return m.group(1)


@pytest.mark.parametrize('name,kind', INTERFACES)
def test_fields_match_design_doc(name, kind):
    doc = _doc_text()
    path = os.path.join(_MSGS_ROOT, kind, '%s.%s' % (name, kind))
    with open(path, encoding='utf-8') as f:
        actual = _fields(f.read())
    expected = _fields(_doc_block(doc, name, kind))
    assert actual == expected, '%s.%s が設計書 §3 と食い違う' % (name, kind)


def test_design_doc_has_no_other_interfaces():
    """設計書 §3 に型を足したのに、ここ（と th_system_msgs）に足し忘れるのを防ぐ。"""
    doc = _doc_text()
    found = re.findall(r'### 3\.\d `(\w+)\.(msg|srv)`', doc)
    assert sorted(found) == sorted(INTERFACES)


@pytest.mark.parametrize('name,kind', INTERFACES)
def test_registered_in_cmake(name, kind):
    with open(os.path.join(_MSGS_ROOT, 'CMakeLists.txt'), encoding='utf-8') as f:
        cmake = f.read()
    assert '"%s/%s.%s"' % (kind, name, kind) in cmake


def test_interface_version_is_one():
    """初版は "1"（設計書 §1.2）。上げるときは設計書とここを同時に直す。"""
    doc = _doc_text()
    assert '**初版は `"1"`**' in doc
