#!/usr/bin/env python3
"""voice_audition.py — VOICEVOX Nemo の話者聞き比べ用の音声を一括生成する。

VISION.md §7.5 の「話者 ID を差し替えて一括再生成できる構造」の先行実装。
本番の全 49 件生成 (generate_voice.py) の前に、代表フレーズだけを全話者分
生成して聞き比べ、話者を決めるために使う。

前提: VOICEVOX Nemo Engine が起動していること (既定ポート 50121)。
  Docker の voicevox_engine には Nemo は入っていないので使えない。
  VOICEVOX アプリを起動するか、run.exe を直接叩く。詳細は docs/voice-credits.md

使い方:
  python3 voice_audition.py                    # 女性話者すべて
  python3 voice_audition.py --all-speakers     # 男性も含む全話者
  python3 voice_audition.py --out ./audition   # 出力先を指定

出力: <out>/<話者名>/<ID>.wav と、比較用の index.html
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import voice_dict  # noqa: E402  (同じディレクトリの兄弟モジュール)

# VOICEVOX Nemo Engine の既定ポート。50021 は VOICEVOX 本体 (キャラクター版)
# なので、そちらに繋ぐと Nemo の話者が見つからない。
DEFAULT_HOST = 'http://127.0.0.1:50121'

# Engine が返す話者名は「女声1」等の番号のみ。公式サイト掲載の音声提供者名を
# 併記して選びやすくする (https://voicevox.hiroshiba.jp/nemo/)。
NEMO_PROVIDERS = {
    '女声1': '亜咲比 凛',
    '女声2': '透川ナナ',
    '女声3': 'ゆう',
    '女声4': 'ぬっぴぃ',
    '女声5': 'たけだまり',
    '女声6': '藤田昌代',
    '男声1': 'レナード・ジン',
    '男声2': 'かちょゴリラ',
    '男声3': '待ち人',
}

# ── 聞き比べ用の代表フレーズ ──────────────────────────────
# VISION.md §7.7 から、性格の異なるものを選んである。
#   ・英字を含む (読みが崩れないか)
#   ・最短 (0.7s。潰れないか)
#   ・実運用で最も出やすい安全通知
#   ・復帰系 (安心感が出るか)
#   ・デモの山場
#   ・数値を含む (分割合成の対象。単体でも自然か)
#
# 主流でないモードでしか鳴らないものは選ばない。例えば C4「逃げ場なし。離れて」は
# 地図あり追従 (FOLLOWING) 専用で、そのモードは VISION.md §3 で優先度低とされている。
# (ID, テキスト, 選定理由)
# 誤読は voice_dict.py のユーザー辞書で直す。フレーズ側で読みを持たない
# ことで、同じ語が別のフレーズに出てきても自動的に正しく読まれる。
AUDITION_PHRASES = [
    ('A1',  '停止。LiDAR 異常',
     '英字を含む。辞書で「ライダー」と読ませている'),
    ('A3',  '緊急停止',
     '最短 0.7s。短くても潰れないか'),
    ('B1',  '対象ロスト。待機へ',
     '実運用で最も出やすい安全通知。追従が止まったと伝わるか'),
    ('D1',  '復旧。再開操作を',
     '復帰系。安心感が出るか'),
    ('N27', '再発見。追従再開',
     'デモの山場。辞書で「ツイジュウ」と読ませている'),
    ('N4',  '捕捉。前方 3 メートル',
     '数値を含む。分割合成の対象'),
]

# 機器アナウンスらしく寄せる (VISION.md §7.5)。
# 話速はやや速め、抑揚は抑え気味。
SYNTH_TUNING = {
    'speedScale':     1.15,
    'intonationScale': 0.8,
    'volumeScale':    1.0,
    'prePhonemeLength':  0.05,
    'postPhonemeLength': 0.1,
}


def api(host: str, path: str, params: dict | None = None,
        data: bytes | None = None, method: str = 'GET', timeout: float = 60.0):
    """VOICEVOX Engine の HTTP API を叩く。

    注意: /audio_query はボディを取らないが POST である。method を data の
    有無から推測すると 405 になるため、呼び出し側で明示する。
    """
    url = f'{host}{path}'
    if params:
        url += '?' + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header('Content-Type', 'application/json')
    with urllib.request.urlopen(req, timeout=timeout) as res:
        return res.read()


def fetch_speakers(host: str):
    """Engine から話者一覧を取得し、(表示名, スタイル名, style_id) の列にする"""
    raw = json.loads(api(host, '/speakers'))
    out = []
    for sp in raw:
        for st in sp.get('styles', []):
            out.append((sp['name'], st['name'], st['id']))
    return out


def is_female_nemo(name: str) -> bool:
    """Nemo の女性話者かどうか。Engine の表示名は「女声1」形式（女"声"）"""
    return name.startswith('女声')


def safe_dirname(name: str) -> str:
    """話者名から ASCII のディレクトリ名を作る。

    「女声1」等の日本語をそのままパスに使うと、GitHub Pages や Markdown の
    リンクで URL エンコードを挟むことになり環境差が出やすい。共有物なので
    ASCII に寄せる（表示名は HTML/Markdown 側で別に持つ）。
    """
    m = re.match(r'(女声|男声)(\d+)', name)
    if m:
        return ('f' if m.group(1) == '女声' else 'm') + m.group(2)
    return re.sub(r'[^\w\-]+', '_', name) or 'speaker'


def synth(host: str, text: str, style_id: int) -> bytes:
    query = json.loads(api(host, '/audio_query', method='POST',
                           params={'text': text, 'speaker': style_id}))
    query.update(SYNTH_TUNING)
    return api(host, '/synthesis', method='POST',
               params={'speaker': style_id},
               data=json.dumps(query).encode('utf-8'))


def to_mp3(wav_path: str) -> str:
    """WAV を MP3 に変換して WAV を消す。リポジトリに載せる際のサイズ対策。"""
    mp3_path = wav_path[:-4] + '.mp3'
    subprocess.run(
        ['ffmpeg', '-y', '-loglevel', 'error', '-i', wav_path,
         '-codec:a', 'libmp3lame', '-b:a', '96k', mp3_path],
        check=True)
    os.remove(wav_path)
    return mp3_path


def build_index_html(out_dir: str, speakers: list[tuple[str, str, int]], ext: str):
    """ブラウザで並べて聞き比べるためのページを書き出す"""
    rows = []
    for pid, text, note in AUDITION_PHRASES:
        cells = []
        for name, style, sid in speakers:
            d = safe_dirname(f'{name}_{style}')
            cells.append(
                f'<td><audio controls preload="none" src="{d}/{pid}.{ext}"></audio></td>')
        rows.append(
            f'<tr><th><div class="id">{pid}</div>'
            f'<div class="txt">{text}</div>'
            f'<div class="note">{note}</div></th>{"".join(cells)}</tr>')

    heads = ''.join(
        f'<th><div class="spk">{name}</div>'
        f'<div class="prov">{NEMO_PROVIDERS.get(name, "")}</div>'
        f'<div class="note">id {sid}</div></th>'
        for name, style, sid in speakers)

    html = f"""<!doctype html>
<meta charset="utf-8">
<title>VOICEVOX Nemo 話者オーディション — TH システム</title>
<style>
  body {{ background:#0f1117; color:#e0e0e0; font-family:system-ui,sans-serif;
         margin:0; padding:1.5rem; }}
  h1 {{ font-size:1.1rem; margin:0 0 .3rem; }}
  p.lead {{ color:#888; font-size:.8rem; margin:0 0 1.2rem; }}
  table {{ border-collapse:collapse; width:100%; }}
  th, td {{ border:1px solid #2a2d35; padding:.5rem .6rem; text-align:left;
            vertical-align:middle; }}
  thead th {{ background:#1a1d24; position:sticky; top:0; }}
  tbody th {{ background:#1a1d24; min-width:15rem; font-weight:normal; }}
  .id  {{ color:#80deea; font-size:.8rem; }}
  .txt {{ font-size:.95rem; margin:.15rem 0; }}
  .spk {{ color:#80deea; font-size:.9rem; }}
  .prov {{ color:#aaa; font-size:.75rem; }}
  .note {{ color:#666; font-size:.7rem; }}
  audio {{ width:13rem; height:2rem; }}
</style>
<h1>VOICEVOX Nemo 話者オーディション</h1>
<p class="lead">
  横が話者、縦が代表フレーズ。話速 {SYNTH_TUNING['speedScale']}倍・
  抑揚 {SYNTH_TUNING['intonationScale']}倍（機器アナウンス寄りの設定）で生成。
  クレジット表記「VOICEVOX Nemo」が必要。
</p>
<table>
  <thead><tr><th>フレーズ</th>{heads}</tr></thead>
  <tbody>{''.join(rows)}</tbody>
</table>
"""
    with open(os.path.join(out_dir, 'index.html'), 'w', encoding='utf-8') as f:
        f.write(html)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=DEFAULT_HOST)
    ap.add_argument('--out', default='audition')
    ap.add_argument('--all-speakers', action='store_true',
                    help='男性話者も含めて生成する')
    ap.add_argument('--mp3', action='store_true',
                    help='MP3 に変換する (リポジトリに載せて共有する場合。要 ffmpeg)')
    args = ap.parse_args()

    if args.mp3 and not shutil.which('ffmpeg'):
        print('--mp3 には ffmpeg が必要です', file=sys.stderr)
        return 1

    try:
        speakers = fetch_speakers(args.host)
    except (urllib.error.URLError, OSError) as e:
        print(f'VOICEVOX Engine に接続できません ({args.host}): {e}', file=sys.stderr)
        print('  VOICEVOX Nemo Engine を起動してください (docs/voice-credits.md 参照)',
              file=sys.stderr)
        return 1

    if not args.all_speakers:
        speakers = [s for s in speakers if is_female_nemo(s[0])]
    if not speakers:
        print('対象の話者が見つかりません。--all-speakers で全件を確認してください。',
              file=sys.stderr)
        return 1

    # 誤読対策のユーザー辞書を先に登録する。Docker の Engine は毎回新しい
    # コンテナなので、生成のたびに入れ直す必要がある
    n = voice_dict.register_all(args.host)
    print(f'ユーザー辞書 {n} 語を登録')

    os.makedirs(args.out, exist_ok=True)
    print(f'{len(speakers)} 話者 × {len(AUDITION_PHRASES)} フレーズを生成します')

    for name, style, sid in speakers:
        d = os.path.join(args.out, safe_dirname(f'{name}_{style}'))
        os.makedirs(d, exist_ok=True)
        for pid, text, _ in AUDITION_PHRASES:
            wav = synth(args.host, text, sid)
            wav_path = os.path.join(d, f'{pid}.wav')
            with open(wav_path, 'wb') as f:
                f.write(wav)
            if args.mp3:
                to_mp3(wav_path)
        print(f'  ✓ {name} ({style}, id={sid})')

    build_index_html(args.out, speakers, 'mp3' if args.mp3 else 'wav')
    print(f'\n完了。{os.path.join(args.out, "index.html")} をブラウザで開いてください')
    return 0


if __name__ == '__main__':
    sys.exit(main())
