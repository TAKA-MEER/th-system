"""図解マニュアル（index.html）を PDF にし、ページ割りが崩れていないかを検査する。

    python docs/manual/tools/make_pdf.py                  # docs/manual/使い方-図解.pdf を作る
    python docs/manual/tools/make_pdf.py --preview DIR    # ついでに各ページを PNG で DIR に出す（目視用）

検査（どれか 1 つでも引っかかったら終了コード 1。PDF は作るが、配る前に直すこと）:
  - あふれ: .sheet（A4 1 ページ）の中身が枠からはみ出していない
  - ページ数: PDF のページ数 ＝ .sheet の数（白紙ページや 2 ページへの割れが無い）
  - 画像: すべての図の PNG が読めている
  - 注釈: annotate.js が見つけた食い違い（凡例と番号・ショットの欠落・目次のリンク先）
  - ページで JS の例外やコンソールのエラーが出ていない
  - 外部への読み込みが無い（機体の AP 配下でも開ける）
"""
import argparse
import pathlib
import sys

from playwright.sync_api import sync_playwright
from pypdf import PdfReader

MANUAL = pathlib.Path(__file__).resolve().parent.parent
INDEX = MANUAL / "index.html"
PDF = MANUAL / "使い方-図解.pdf"

CHECK_JS = r"""
(() => {
  const out = {overflow: [], broken: [], sheets: 0};
  const sheets = [...document.querySelectorAll('.sheet')];
  out.sheets = sheets.length;
  sheets.forEach((s, i) => {
    const title = (s.querySelector('h2,h1') || {}).textContent || '';
    const dy = s.scrollHeight - s.clientHeight, dx = s.scrollWidth - s.clientWidth;
    // 子要素の下端がページの内側（下余白の上）を越えていないか
    const pad = parseFloat(getComputedStyle(s).paddingBottom);
    const limit = s.getBoundingClientRect().bottom - pad + 0.5;
    let low = 0;
    [...s.children].forEach(c => { low = Math.max(low, c.getBoundingClientRect().bottom - limit); });
    if (dy > 1 || dx > 1 || low > 0)
      out.overflow.push(`p.${i + 1}「${title.trim().slice(0, 30)}」: 縦 ${Math.max(dy, Math.round(low))}px / 横 ${dx}px はみ出し`);
  });
  document.querySelectorAll('img').forEach(im => {
    if (!im.complete || !im.naturalWidth) out.broken.push(im.getAttribute('src'));
  });
  out.manual = window.__manualErrors || ['annotate.js が動いていない'];
  return out;
})()
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--preview", help="各ページを PNG で書き出すディレクトリ")
    ap.add_argument("--out", default=str(PDF))
    args = ap.parse_args()

    problems = []
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 900, "height": 1200}, device_scale_factor=1)
        console = []
        external = []
        page.on("console", lambda m: m.type == "error" and console.append(m.text))
        page.on("pageerror", lambda e: console.append(str(e)))
        page.on("request", lambda r: (not r.url.startswith("file:") and not r.url.startswith("data:")) and external.append(r.url))
        page.emulate_media(media="print")
        page.goto(INDEX.as_uri(), wait_until="load")
        page.evaluate("document.fonts.ready")
        page.wait_for_timeout(300)

        r = page.evaluate(CHECK_JS)
        problems += ["あふれ: " + x for x in r["overflow"]]
        problems += ["画像が読めない: " + x for x in r["broken"]]
        problems += ["注釈: " + x for x in r["manual"]]
        problems += ["コンソール: " + x for x in console]
        problems += ["外部への読み込み: " + x for x in external]

        page.pdf(path=args.out, prefer_css_page_size=True, print_background=True)

        if args.preview:
            outdir = pathlib.Path(args.preview)
            outdir.mkdir(parents=True, exist_ok=True)
            for old in outdir.glob("p*.png"):
                old.unlink()
            for i, h in enumerate(page.query_selector_all(".sheet"), 1):
                h.screenshot(path=str(outdir / f"p{i:02d}.png"))
        browser.close()

    n = len(PdfReader(args.out).pages)
    if n != r["sheets"]:
        problems.append(f"ページ数: PDF {n} ページ / シート {r['sheets']} 枚（白紙や割れたページがある）")

    print(f"{args.out}: {n} ページ")
    for x in problems:
        print("NG", x)
    if not problems:
        print("検査はすべて通過")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
