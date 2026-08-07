#!/usr/bin/env python3
"""generate_voice.py — 本番の音声クリップを一括生成する。

announcements.js のマニフェストを唯一の正として読み、採用話者で全件を合成して
web_ui/public/voice/ に書き出す。文案を直したらこれを流し直すだけでよい。

ANNOUNCEMENTS (完成した1発話) に加え、数値分割合成用の語彙 CLIP_WORDS
(数字・単位。VISION.md §7.5) も同じ要領で <id>.mp3 として生成する。

前提: VOICE_PROFILES に登録した Engine が起動していること。
  nemo    : VOICEVOX Nemo Engine (既定ポート 50121)。Docker の voicevox_engine には
            Nemo は入っていない。詳細は docs/voice-credits.md
  zundamon: 通常版 VOICEVOX Engine (既定ポート 50021)。展示専用の例外採用
            （docs/voice-credits.md「展示専用の例外: ずんだもん」参照）

使い方:
  python3 generate_voice.py                    # --profile nemo (既定) で全件生成
  python3 generate_voice.py --profile zundamon # ずんだもんで全件生成
  python3 generate_voice.py --speaker 10005    # 話者だけ変えて試す (出力先はプロファイル既定のまま)
  python3 generate_voice.py --wav              # MP3 にせず WAV のまま出す
  python3 generate_voice.py --only hokaku,0,1,2,meter  # id を絞って生成
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

# 話者プロファイル。host/speaker/出力サブディレクトリをまとめて切り替える。
# --host/--speaker/--out を個別指定した場合はそちらを優先する (プロファイルは既定値の束に過ぎない)。
#
# zundamon は展示専用の例外採用 (docs/voice-credits.md「展示専用の例外: ずんだもん」参照)。
# 通常運用の既定は nemo のまま変えない。style id 3 は「ノーマル」(通常版 VOICEVOX Engine
# の /speakers で確認済み)。話速チューニング (LONG_IDS/LONG_TUNING) は当面 nemo で実測した
# 値をそのまま流用する — ずんだもんでの実測調整は未着手 (VISION.md §7.5 参照)。
VOICE_PROFILES = {
    'nemo':     {'host': 'http://127.0.0.1:50121', 'speaker': 10004, 'out_subdir': 'nemo'},
    'zundamon': {'host': 'http://127.0.0.1:50021', 'speaker': 3,     'out_subdir': 'zundamon'},
}

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
PUBLIC_VOICE_DIR = os.path.join(REPO_ROOT, 'th_ws', 'web_ui', 'public', 'voice')


def load_manifest() -> list[dict]:
    """announcements.js を node 経由で読む。

    正規表現でパースすると書式変更に弱いので、実際に import して
    ANNOUNCEMENTS / CLIP_WORDS をそのまま受け取る。JS 側を唯一の正に保つための措置。

    マニフェストの中身をそのまま一時 .mjs にコピーして実行する。web_ui/package.json
    に "type": "module" が無いため、node 18 は .js を CJS と決め打ちして "export" で
    構文エラーになる。import 元を .mjs にしても、読み込まれる .js 側は CJS 扱いの
    ままなので、ファイルごと .mjs にする必要がある。
    announcements.js は他を import しない純データなので、コピーで完結する。

    CLIP_WORDS は数値分割合成用の語彙断片で、layer を持たない (キューの再生対象
    ではないため)。ANNOUNCEMENTS と同じ {id, text, layer} 形に揃えて合流させる
    (layer は null のまま。生成処理では使わない)。
    """
    with open(MANIFEST, encoding='utf-8') as f:
        src = f.read()
    code = (src + "\nconsole.log(JSON.stringify("
                  "ANNOUNCEMENTS.map(a => ({ id: a.id, text: a.text, layer: a.layer }))"
                  ".concat(CLIP_WORDS.map(w => ({ id: w.id, text: w.text, layer: null })))"
                  "));\n")
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
    ap.add_argument('--profile', choices=sorted(VOICE_PROFILES), default='nemo',
                     help='host/speaker/出力先をまとめて切り替える (既定 nemo)')
    ap.add_argument('--host', default=None, help='未指定なら --profile の値を使う')
    ap.add_argument('--speaker', type=int, default=None, help='未指定なら --profile の値を使う')
    ap.add_argument('--out', default=None, help='未指定なら --profile の値を使う')
    ap.add_argument('--wav', action='store_true', help='MP3 に変換しない')
    ap.add_argument('--only', help='カンマ区切りの id だけ生成する (例: --only hokaku,0,1,meter)')
    args = ap.parse_args()

    profile = VOICE_PROFILES[args.profile]
    if args.host is None:
        args.host = profile['host']
    if args.speaker is None:
        args.speaker = profile['speaker']
    if args.out is None:
        args.out = os.path.join(PUBLIC_VOICE_DIR, profile['out_subdir'])

    if not args.wav and not shutil.which('ffmpeg'):
        print('MP3 変換には ffmpeg が必要です (--wav で回避できます)', file=sys.stderr)
        return 1

    try:
        speakers = json.loads(api(args.host, '/speakers'))
    except (urllib.error.URLError, OSError) as e:
        print(f'VOICEVOX Engine に接続できません ({args.host}): {e}', file=sys.stderr)
        print(f'  --profile {args.profile} が使う Engine を起動してください '
              '(docs/voice-credits.md 参照)', file=sys.stderr)
        return 1

    found = next(((sp['name'], st['name'])
                  for sp in speakers for st in sp['styles']
                  if st['id'] == args.speaker), None)
    if found is None:
        print(f'話者 id={args.speaker} が Engine ({args.host}) にありません。'
              f'--profile {args.profile} が想定する Engine (nemo=Nemo Engine port 50121 / '
              f'zundamon=通常版 VOICEVOX Engine port 50021) に繋いでいませんか',
              file=sys.stderr)
        return 1
    print(f'話者: {found[0]} ({found[1]}, id={args.speaker})')

    n = voice_dict.register_all(args.host)
    print(f'ユーザー辞書 {n} 語を登録')

    entries = load_manifest()
    if args.only:
        wanted = set(args.only.split(','))
        entries = [e for e in entries if e['id'] in wanted]
        missing = wanted - {e['id'] for e in entries}
        if missing:
            print(f'--only で指定された id がマニフェストに無い: {sorted(missing)}',
                  file=sys.stderr)
            return 1
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
