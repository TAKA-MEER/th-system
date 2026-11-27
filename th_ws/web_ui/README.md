# WebUI（th_ws/web_ui）

rosbridge 経由で機体と話す画面。仕様は `docs/plan/spec/Spec-webui.md`（正本はモックアップとの整合＋ spec 優先）。

## 起動
```bash
cd th_ws/web_ui
npm run build            # dist へ
npx serve -s dist -l 5173  # 電子組立画面で立ち上げる
```
開発中は `npm run dev`。機体へは rosbridge（WebSocket）で接続する。

## 試験
```bash
npm run test:unit                         # 単体（node --test）
CI=1 PW_PORT=4401 npx playwright test     # e2e。必ず CI=1 と PW_PORT を付ける
```
※ `CI=1 PW_PORT` なしだと既定 4173 番の他者のサーバを使い回してしまう。

## 7 機（状態を読む）
- 機体が接続できないとき（`connected == false`、rosbridge 未起動）: ヘッダは「切断」表示、
  画面は動かない。`/th_ws` の起動手順で rosbridge を起動する。
- `connected == true` に戻れば通常表示になる。WebSocket の再接続は roslib が行う。

## ルート表と位置
| ルート | 意味 |
|---|---|
| `src/main.jsx` | 入口。`#app`・`#stage`・`data-theme`・`data-orientation`。 |
| `src/shell/AppShell.jsx` | ヘッダ・本体・窓・非常停止と解除バー。自動ブレーキ OFF 警告・自律枠もここ。 |
| `src/shell/Header.jsx` | ヘッダ。状態札（帯・一行）・速度制限（場内だけ）・自動ブレーキなし（点滅）。 |
| `src/shell/Windows.jsx` | 例外の窓（W-1/W-2 確認だけ。W-4 は使わない）・W-6・異常履歴。 |
| `src/shell/OperationCard.jsx` | 操作カード（停止・保存・手動・走行・終了・中断）。 |
| `src/shell/OperationBar.jsx` | 左上の操作バー（終了ボタン。`exit` の形）。 |
| `src/shell/Layouts.jsx` | 3つの型の骨組み（走らせる・手順を進める・見る・変える）。`data-pos` で位置を固定。 |
| `src/parts/` | 共通部品（停止│走行・ヘルプ・自動ブレーキのロック・人検出など）。 |
| `src/screens/S*.jsx` | 各画面。型は `Layouts.jsx` に寄せる。 |

## 色の決まり（3.3.1）
- 緑=走行（次に押す「走行」も緑）、黄=注意、赤=危険（非常停止だけ底が黄）、白と黒=押せる、
  灰=押せない、明るい青=リンク・選んだものだけ。
- ボタンが色を変える場面は「停止│走行」「切│入」「ON│OFF」「自動ブレーキ（場内の切替）」だけ。

## テストの書き方
- 単体: `test/unit/*.test.js`（`node --test`）。新しい純関数にはここに足す。
- e2e: `e2e/*.spec.js`。ヘルパーは `e2e/helpers.js`。`__thSetTestState` で状態を入れて回す。
- 変更した挙動は変異チェック（変えて試験が落ちることを確認）をする。

## 書き換え前の姿
書き換え前に残した仕様書は `docs/plan/spec/_legacy/` にある。現行は `docs/plan/spec/` を読む。
