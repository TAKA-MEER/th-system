#!/usr/bin/env python3
"""generate_voice.py — 本番の音声クリップを一括生成する。

announcements.js のマニフェストを唯一の正として読み、採用話者で全件を合成して
web_ui/public/voice/ に書き出す。文案を直したらこれを流し直すだけでよい。

前提: VOICEVOX Nemo Engine が起動していること (既定ポート 50121)。
  Docker の voicevox_engine には Nemo は入っていない。詳細は docs/voice-credits.md

使い方:
  python3 generate_voice.py                 # 採用話者で全件生成
  python3 generate_voice.py --speaker 10005 # 話者を変えて試す
  python3 generate_voice.py --wav           # MP3 にせず WAV のまま出す
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import tempfile
import sys
import urllib.error

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import voice_dict  # noqa: E402
from voice_audition import SYNTH_TUNING, api, synth, to_mp3  # noqa: E402

# 採用話者: 女声3 (ゆう)。docs/voice-credits.md 参照
DEFAULT_SPEAKER = 10004

# 長いフレーズだけ話速を上げ、句点の間を詰める。
#
# 既定 (話速 1.15 / 抑揚 0.8) で全件を生成して実測したところ、設計時の見積もり
# (7モーラ/秒) に対し平均 1.41 倍長く、最長の N15 は 3.17 秒あった。短いものは
# そのままで問題ないが、長いものは待たされる感が出るため個別に詰める。
#
# 対象は「既定設定での実測が 2.0 秒以上だったもの」。閾値ではなく ID を直接
# 持つのは、詰めた結果 2.0 秒を下回ると再実行のたびに対象が変わってしまうため。
LONG_IDS = {'N15', 'N20', 'N4', 'A2', 'N36', 'N27', 'N19', 'A1'}
LONG_TUNING = {
    'speedScale':       1.4,
    'pauseLengthScale': 0.75,   # 句点「。」の間の長さ
}

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MANIFEST = os.path.join(REPO_ROOT, 'th_ws', 'web_ui', 'src', 'voice', 'announcements.js')
DEFAULT_OUT = os.path.join(REPO_ROOT, 'th_ws', 'web_ui', 'public', 'voice')


def load_manifest() -> list[dict]:
    """announcements.js を node 経由で読む。

    正規表現でパースすると書式変更に弱いので、実際に import して
    ANNOUNCEMENTS をそのまま受け取る。JS 側を唯一の正に保つための措置。

    マニフェストの中身をそのまま一時 .mjs にコピーして実行する。web_ui/package.json
    に "type": "module" が無いため、node 18 は .js を CJS と決め打ちして "export" で
    構文エラーになる。import 元を .mjs にしても、読み込まれる .js 側は CJS 扱いの
    ままなので、ファイルごと .mjs にする必要がある。
    announcements.js は他を import しない純データなので、コピーで完結する。
    """
    with open(MANIFEST, encoding='utf-8') as f:
        src = f.read()
    code = (src + "\nconsole.log(JSON.stringify(ANNOUNCEMENTS.map("
                  "a => ({ id: a.id, text: a.text, layer: a.layer }))));\n")
    with tempfile.NamedTemporaryFile('w', suffix='.mjs', delete=False,
                                     encoding='utf-8') as f:
        f.write(code)
        tmp = f.name
    try:
        out = subprocess.run(['node', tmp], capture_output=True, text=True,
                             encoding='utf-8')
    finally:
        os.unlink(tmp)
    if out.returncode != 0:
        raise RuntimeError(f'マニフェストを読めません:\n{out.stderr}')
    line = next(l for l in out.stdout.splitlines() if l.strip().startswith('['))
    return json.loads(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=voice_dict.DEFAULT_HOST)
    ap.add_argument('--speaker', type=int, default=DEFAULT_SPEAKER)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--wav', action='store_true', help='MP3 に変換しない')
    args = ap.parse_args()

    if not args.wav and not shutil.which('ffmpeg'):
        print('MP3 変換には ffmpeg が必要です (--wav で回避できます)', file=sys.stderr)
        return 1

    try:
        speakers = json.loads(api(args.host, '/speakers'))
    except (urllib.error.URLError, OSError) as e:
        print(f'VOICEVOX Engine に接続できません ({args.host}): {e}', file=sys.stderr)
        print('  VOICEVOX Nemo Engine を起動してください (docs/voice-credits.md 参照)',
              file=sys.stderr)
        return 1

    found = next(((sp['name'], st['name'])
                  for sp in speakers for st in sp['styles']
                  if st['id'] == args.speaker), None)
    if found is None:
        print(f'話者 id={args.speaker} が Engine にありません。'
              f'Nemo Engine ではなく VOICEVOX 本体に繋いでいませんか',
              file=sys.stderr)
        return 1
    print(f'話者: {found[0]} ({found[1]}, id={args.speaker})')

    n = voice_dict.register_all(args.host)
    print(f'ユーザー辞書 {n} 語を登録')

    entries = load_manifest()
    os.makedirs(args.out, exist_ok=True)
    ext = 'wav' if args.wav else 'mp3'

    n_long = 0
    for e in entries:
        tuning = LONG_TUNING if e['id'] in LONG_IDS else None
        if tuning:
            n_long += 1
        wav_path = os.path.join(args.out, f"{e['id']}.wav")
        with open(wav_path, 'wb') as f:
            f.write(synth(args.host, e['text'], args.speaker, tuning))
        if not args.wav:
            to_mp3(wav_path)

    total = sum(os.path.getsize(os.path.join(args.out, f))
                for f in os.listdir(args.out) if f.endswith(ext))
    print(f'{len(entries)} 件を生成 → {args.out} ({total / 1024:.0f} KB)')
    print(f'  既定    : 話速 {SYNTH_TUNING["speedScale"]}倍 / 抑揚 {SYNTH_TUNING["intonationScale"]}倍')
    print(f'  長い{n_long}件: 話速 {LONG_TUNING["speedScale"]}倍 / '
          f'間 {LONG_TUNING["pauseLengthScale"]}倍  ({", ".join(sorted(LONG_IDS))})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
