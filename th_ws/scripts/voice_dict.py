#!/usr/bin/env python3
"""voice_dict.py — 音声合成用のユーザー辞書。

VOICEVOX の既定辞書では正しく読めない語をここに集約する。合成前に
register_all() を呼んで Engine に登録すること。Engine を再起動すると辞書は消えるため、
生成のたびに登録し直す必要がある。

単体でも実行できる:
  python3 voice_dict.py            # 登録して読みを確認
  python3 voice_dict.py --list     # 現在の登録内容を表示
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.parse
import urllib.request

# VOICEVOX Nemo Engine の既定ポート (VOICEVOX 本体の 50021 とは別)
DEFAULT_HOST = 'http://127.0.0.1:50121'

# priority は 0-10。既定の 5 では組み込み辞書に負けて反映されないため
# 10 を使う（実機確認済み: 5 だと「追従」が「ツイショオ」のままだった）。
PRIORITY = 10

# (表記, 読み, アクセント型, 理由)
#
# アクセント型は 0 = 平板、1 以降はその位置で下がる。耳で聞いて不自然なら調整する。
TERMS = [
    # 「追従」は既定辞書だと「ツイショオ」と読まれる。これは「へつらう」意味の
    # 別語で、制御分野の「ついじゅう」とは異なる。N5/N6/N14/N21/N27 が該当。
    ('追従', 'ツイジュウ', 0,
     '既定は「ツイショオ」(へつらう意) になる。制御用語としては「ついじゅう」'),

    # ラテン文字は綴り読みされる。「リディーエーアール」になっていた。
    ('LiDAR', 'ライダー', 1,
     '既定は綴り読み「リディイエエアアル」になる'),

    # 「中」が「ナカ」と読まれる。他の「〜中」(待機中・追従中など) は
    # 正しく「チュウ」と読まれるので、この語だけの問題。
    ('向き合わせ中', 'ムキアワセチュウ', 0,
     '既定は「ムキアワセナカ」になる'),
]


def _post(host: str, path: str, params: dict, timeout: float = 30.0):
    url = f'{host}{path}?' + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, method='POST')
    with urllib.request.urlopen(req, timeout=timeout) as res:
        return res.read()


def register_all(host: str = DEFAULT_HOST) -> int:
    """TERMS をすべて登録する。登録済みでも重複登録されるだけで害はない。"""
    for surface, yomi, accent, _ in TERMS:
        _post(host, '/user_dict_word', {
            'surface': surface,
            'pronunciation': yomi,
            'accent_type': accent,
            'priority': PRIORITY,
        })
    return len(TERMS)


def reading_of(host: str, text: str, speaker: int = 10005) -> str:
    """text を Engine に読ませたときのカナを返す（確認用）"""
    url = (f'{host}/audio_query?'
           + urllib.parse.urlencode({'text': text, 'speaker': speaker}))
    req = urllib.request.Request(url, method='POST')
    with urllib.request.urlopen(req, timeout=30) as res:
        q = json.load(res)
    return ''.join(m['text'] for p in q['accent_phrases'] for m in p['moras'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=DEFAULT_HOST)
    ap.add_argument('--list', action='store_true', help='Engine の登録内容を表示')
    args = ap.parse_args()

    if args.list:
        with urllib.request.urlopen(f'{args.host}/user_dict', timeout=30) as res:
            for uuid, w in json.load(res).items():
                print(f"  {w['surface']:<14} {w['pronunciation']:<16} "
                      f"priority={w['priority']} accent={w['accent_type']}")
        return 0

    n = register_all(args.host)
    print(f'{n} 語を登録しました\n')
    ng = 0
    for surface, yomi, _, reason in TERMS:
        actual = reading_of(args.host, surface)
        # Engine のモーラ表記は長音「ー」を母音に展開する (ライダー → ライダア)
        # ため、そのまま文字列比較すると誤って NG になる
        ok = re.fullmatch(yomi.replace('ー', '.'), actual) is not None
        print(f"  {'OK ' if ok else 'NG '} {surface:<14} → {actual}"
              f"{'' if ok else f'  (期待: {yomi})'}")
        if not ok:
            ng += 1
            print(f'        {reason}')
    return 1 if ng else 0


if __name__ == '__main__':
    sys.exit(main())
