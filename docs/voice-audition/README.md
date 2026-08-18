# 音声アナウンス — 話者オーディション

音声フィードバック（[docs/voice-and-audience.md §2](../voice-and-audience.md#2-音声フィードバック)）で使う話者を
チームで聞き比べて決めるためのページ。

## 聞き方

### 方法1: GitHub Pages（推奨・並べて聞ける）

<https://taka-meer.github.io/th-system/voice-audition/>

横が話者 6 名、縦が代表フレーズ 6 種の表になっていて、その場で再生・比較できる。

> **リポジトリ管理者へ**: 上の URL を有効にするには一度だけ設定が要る。
> Settings → Pages → Source: `Deploy from a branch` →
> Branch: `feat/voice-feedback` / Folder: `/docs` → Save。

### 方法2: ファイルを直接開く

**GitHub の Markdown は `<audio>` タグを除去するため、この README 上で再生することはできない。**
音声の埋め込みは MP4 の動画としてドラッグ&ドロップした場合のみ対応している。
そのため個別に開くには下表のリンクから各ファイルを開く（ブラウザでそのまま再生される）。

| フレーズ | 女声1<br>亜咲比 凛 | 女声2<br>透川ナナ | 女声3<br>ゆう | 女声4<br>ぬっぴぃ | 女声5<br>たけだまり | 女声6<br>藤田昌代 |
| --- | --- | --- | --- | --- | --- | --- |
| **A1** 停止。LiDAR 異常 | [▶](f1/A1.mp3) | [▶](f2/A1.mp3) | [▶](f3/A1.mp3) | [▶](f4/A1.mp3) | [▶](f5/A1.mp3) | [▶](f6/A1.mp3) |
| **A3** 緊急停止 | [▶](f1/A3.mp3) | [▶](f2/A3.mp3) | [▶](f3/A3.mp3) | [▶](f4/A3.mp3) | [▶](f5/A3.mp3) | [▶](f6/A3.mp3) |
| **B1** 対象ロスト。待機へ | [▶](f1/B1.mp3) | [▶](f2/B1.mp3) | [▶](f3/B1.mp3) | [▶](f4/B1.mp3) | [▶](f5/B1.mp3) | [▶](f6/B1.mp3) |
| **D1** 復旧。再開操作を | [▶](f1/D1.mp3) | [▶](f2/D1.mp3) | [▶](f3/D1.mp3) | [▶](f4/D1.mp3) | [▶](f5/D1.mp3) | [▶](f6/D1.mp3) |
| **N27** 再発見。追従再開 | [▶](f1/N27.mp3) | [▶](f2/N27.mp3) | [▶](f3/N27.mp3) | [▶](f4/N27.mp3) | [▶](f5/N27.mp3) | [▶](f6/N27.mp3) |
| **N4** 捕捉。前方 3 メートル | [▶](f1/N4.mp3) | [▶](f2/N4.mp3) | [▶](f3/N4.mp3) | [▶](f4/N4.mp3) | [▶](f5/N4.mp3) | [▶](f6/N4.mp3) |

### 方法3: 手元で再生成する

```powershell
# VOICEVOX Nemo Engine を起動する (VOICEVOX アプリを立ち上げても起動する)。
# Docker の voicevox_engine には Nemo は入っていないので注意 — あちらは
# キャラクター版 43 名で、Nemo の 9 話者は含まれない。
cd "$env:APPDATA\voicevox\vvpp-engines\VOICEVOX_Nemo_Engine+<uuid>"
.\run.exe --port 50121
```

```bash
# 生成 (--mp3 を外すと WAV。--all-speakers で男声も出る)
python3 th_ws/scripts/voice_audition.py --out docs/voice-audition --mp3
```

## 選ぶときの観点

フレーズは性格の異なるものを選んである。**主流でないモードでしか鳴らないものは含めない**
（例: C4「逃げ場なし。離れて」は地図あり追従 FOLLOWING 専用で、そのモードは
旧 VISION.md §3 で優先度低とされていた。現在の走行方式の位置づけは
[Spec-transit.md](../plan/spec/Spec-transit.md)）。

| ID | 文言 | 何を見るか |
| --- | --- | --- |
| A1 | 停止。LiDAR 異常 | 英字を含む。辞書で「ライダー」と読ませている |
| A3 | 緊急停止 | 最短 0.7 秒。短くても潰れず聞き取れるか |
| B1 | 対象ロスト。待機へ | 実運用で最も出やすい安全通知。追従が止まったと伝わるか |
| D1 | 復旧。再開操作を | 復帰系。安心感が出るか |
| N27 | 再発見。追従再開 | デモの山場。辞書で「ついじゅう」と読ませている |
| N4 | 捕捉。前方 3 メートル | 数値を含む。数値は分割合成する対象 |

### 誤読の扱い

VOICEVOX の既定辞書では正しく読めない語があるため、`th_ws/scripts/voice_dict.py` に
ユーザー辞書を持たせて合成前に登録している。現在の登録は 3 語。

| 表記 | 正しい読み | 既定の読み（誤り） |
| --- | --- | --- |
| 追従 | ツイジュウ | ツイショオ（「へつらう」意の別語） |
| LiDAR | ライダー | リディイエエアアル（綴り読み） |
| 向き合わせ中 | ムキアワセチュウ | ムキアワセナカ |

**聞いていて他にも読みがおかしい語があれば教えてほしい。** 辞書に足せば全フレーズに反映される。

```bash
python3 th_ws/scripts/voice_dict.py          # 登録して読みを確認
python3 th_ws/scripts/voice_dict.py --list   # 現在の登録内容
```

合成条件は**話速 1.15 倍・抑揚 0.8 倍**（機器アナウンス寄りに寄せた設定、[docs/voice-and-audience.md](../voice-and-audience.md) §2.5）。
話者を決めた後、この値自体も調整の余地がある。

判断の前提として、この音声は**オリジナルキャラクターの声**として使う想定。
VOICEVOX Nemo はキャラクターとしての人格を持たない音声ライブラリなので、
既存キャラクターとの同一性の問題が起きない（それが Nemo を選んだ理由）。

## クレジット

この音声は **VOICEVOX Nemo** で生成している。利用にはクレジット表記が必要。

- 公式: <https://voicevox.hiroshiba.jp/nemo/>
- 利用規約: <https://voicevox.hiroshiba.jp/nemo/term/>

詳細は [../voice-credits.md](../voice-credits.md) を参照。
