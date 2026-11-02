# 図解マニュアル（docs/manual）

タブレット画面（WebUI）のスクリーンショットに番号付きの注釈を重ねた、`docs/使い方.md` の図解版。
**画面はいまモックアップ（`docs/plan/spec/mockup/index.html`）を「横長」1280×720 で撮っている。**
実機の画面での撮り直しは今後（下の「実機の画面に切り替えるとき」）。

- 見る: `index.html` をブラウザで開く（外部を読まないので、機体の AP 配下でも開ける）
- 配る: **PDF にして配る**（`tools/make_pdf.py`）。PDF は生成物なので git に入れない（`.gitignore`）

## 構成

| ファイル | 役割 |
| --- | --- |
| `index.html` | 本文。A4 縦の `.sheet` 1 つが PDF の 1 ページ。凡例・手順・注意の文言はここに書く |
| `manual.css` | ページ・図・注釈の体裁。ページの大きさは mm で決め打ち（画面でも PDF でも同じページ割り） |
| `annotate.js` | `img/shots.js` を読んで図に枠と番号丸を重ねる。凡例の番号との食い違いも検査する |
| `tools/shots.py` | ショットの定義（状態を作る JS・切り抜く要素・注釈の番号と要素）。**文言は持たない** |
| `tools/capture.py` | モックアップを開いて撮影し、`img/<id>.png` と `img/shots.js`（注釈の矩形）を書く |
| `tools/make_pdf.py` | `index.html` を `使い方-図解.pdf` にし、ページ割りと注釈を検査する |
| `img/` | 撮影結果（生成物だが、HTML をそのまま開けるよう git に入れる） |

## 作り直す

Windows ホストの Python で動く（2026-11-02 に Python 3.12・Playwright 1.61・pypdf 6.14 で確認）。

```bash
pip install playwright pypdf && python -m playwright install chromium   # 初回だけ

python docs/manual/tools/capture.py              # 全部撮り直す（約 1 分）
python docs/manual/tools/capture.py s11 s14-run  # 指定したショットだけ
python docs/manual/tools/make_pdf.py             # PDF を作って検査する
python docs/manual/tools/make_pdf.py --preview <DIR>   # 各ページの PNG も出す（目で確かめる用）
```

`capture.py` が**失敗にする**もの: 準備の JS の例外・注釈の要素が見つからない／見えない・撮影範囲から半分以上はみ出す。
**警告**: 本文がスクロールしないと見えない分がある（意図して一部だけ載せるショットは `allow_scroll=True`）。

`make_pdf.py` が**失敗にする**もの（終了コード 1）:

- ページのあふれ（`.sheet` の中身が A4 の枠を越えた）
- PDF のページ数と `.sheet` の数の食い違い（白紙ページ・2 ページへの割れ）
- 読めない画像・凡例と図の番号の食い違い・目次のリンク切れ・コンソールのエラー・外部への読み込み

## 書き方の決まり

- **文言の正は `docs/使い方.md`**。この冊子は「画面のどこを見て、どこを押すか」を図で示すだけにして、
  手順・挙動は使い方.md に合わせる。各ページの末尾に対応する節を書く。
- 画面に見える語はモックアップに合わせる。**モックアップと実機が違うところは、紫の「実機との違い」の枠で書く**
  （実機に無い画面は付録に回し「実機では未実装」の札を付ける）。
- 1 ページ = 1 つの `.sheet`。あふれたらページを分ける（`make_pdf.py` が教えてくれる）。
- 注釈は `tools/shots.py` に番号と要素だけを書き、名前と意味は `index.html` の `<ol class="legend" data-shot="ID">` に書く。
  番号丸の位置は `tl/t/tr/l/c/r/bl/b/br`（対象の外側に出る）。文字に重なったら位置を変える。
- 同じショットを別のページで一部の番号だけ使うときは、`<figure data-marks="1,3">` で絞る（凡例はページごとに突き合わせる）。

## 保守のルール

- **モックアップ（`docs/plan/spec/mockup/index.html`）を変えたら**、`capture.py` で撮り直して `make_pdf.py` の検査を通す。
  要素の名前（id・class・文言）を変えたときは `shots.py` のセレクタも直す（見つからなければ `capture.py` が落ちる）。
- **`docs/使い方.md` の手順・挙動・画面の文言を変えたら**、この冊子の該当ページの文も同じコミットで直す。

## 実機の画面に切り替えるとき（今後）

`capture.py` の開く先を、モックアップから実機の WebUI の本番ビルド（`th_ws/web_ui` の `vite preview`）に変える。
状態は `th_ws/web_ui/e2e/helpers.js` の `gotoScreen` などと同じやり方（偽の rosbridge に状態を流す）で作れる。
`shots.py` の `setup` とセレクタを実機向けに差し替えれば、`index.html` の文言と凡例はそのまま使える
（番号と要素の対応が変わったら凡例も直す）。実機の画面にそろったら「実機との違い」の枠を消す。
