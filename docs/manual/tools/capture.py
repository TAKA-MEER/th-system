"""図解マニュアルのスクリーンショットを撮る。

モックアップ docs/plan/spec/mockup/index.html を「横長」（論理 1280×720）で開き、
tools/shots.py の定義どおりに状態を作って撮る。注釈を付ける要素の位置もその場で測り、
img/shots.js（window.SHOTS）に書き出す。HTML 側の annotate.js がそれを読んで番号を重ねる。

    python docs/manual/tools/capture.py              # 全部撮る
    python docs/manual/tools/capture.py s11-main     # 指定したものだけ撮る（shots.js は差分更新）

失敗にするもの: 準備の JS が例外を出す・注釈の要素が見つからない・大きさ 0・撮影範囲から半分以上はみ出す。
警告にするもの: 本文（#body）がスクロールしないと見えない分を持っている（載せ漏れの見落とし防止）。
"""
import json
import pathlib
import re
import sys

from playwright.sync_api import sync_playwright

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from shots import SHOTS  # noqa: E402

MANUAL = HERE.parent
REPO = MANUAL.parent.parent
MOCKUP = REPO / "docs" / "plan" / "spec" / "mockup" / "index.html"
IMG = MANUAL / "img"
SHOTS_JS = IMG / "shots.js"

# fitStage() の倍率が 1 になる大きさ: (W-36)/1280 = (H-36)/720 = 1
VIEWPORT = {"width": 1316, "height": 756}
SCALE = 2  # PNG の解像度（論理 px の 2 倍。A4 幅 186mm で約 350dpi）

# モックアップ専用の操作パネル（#demo）は隠すだけにする。消すと engageEstop() が落ちる
INJECT_CSS = """
#demo{display:none!important}
*,*::before,*::after{transition:none!important;animation:none!important;caret-color:transparent!important}
"""

# 準備の JS から使う小道具
HELPERS_JS = r"""
window.__tab = id => {
  const b = document.querySelector(`[onclick="tab(this,'${id}')"]`);
  if (!b) throw new Error('tab not found: ' + id);
  b.click();
};
window.__ocrow = i => oc(document.querySelectorAll('#s30 .ocrow')[i], 'oc' + (i + 1));
window.__vis = e => {
  if (!e || !e.getClientRects().length) return false;
  const cs = getComputedStyle(e);
  if (cs.visibility === 'hidden' || cs.display === 'none') return false;
  const r = e.getBoundingClientRect();
  return r.width > 0 && r.height > 0;
};
// "css" か "css::含む文字列"。" || " で区切ると矩形を合併する
window.__rect = spec => {
  let L = Infinity, T = Infinity, R = -Infinity, B = -Infinity;
  for (const part of spec.split(' || ')) {
    const i = part.indexOf('::');
    const css = (i < 0 ? part : part.slice(0, i)).trim();
    const text = i < 0 ? null : part.slice(i + 2).trim();
    let c = [...document.querySelectorAll(css)].filter(__vis);
    if (text !== null) {
      c = c.filter(e => e.textContent.replace(/\s+/g, ' ').includes(text));
      c.sort((a, b) => a.textContent.length - b.textContent.length);
    }
    if (!c.length) return {error: 'not found or not visible: ' + part};
    const r = c[0].getBoundingClientRect();
    L = Math.min(L, r.left); T = Math.min(T, r.top); R = Math.max(R, r.right); B = Math.max(B, r.bottom);
  }
  return {x: L, y: T, w: R - L, h: B - T};
};
// 最後の式が関数だと Playwright がそれを呼んでしまうので、値で終える
true;
"""


def load_existing():
    if not SHOTS_JS.exists():
        return {}
    m = re.search(r"window\.SHOTS\s*=\s*(\{.*\});?\s*$", SHOTS_JS.read_text(encoding="utf-8"), re.S)
    return json.loads(m.group(1)) if m else {}


def write_shots_js(data):
    body = json.dumps(data, ensure_ascii=False, indent=1, sort_keys=True)
    SHOTS_JS.write_text(
        "// tools/capture.py が生成する。手で直さない。\n"
        "// 各ショットの大きさ（論理 px）と、注釈を付ける要素の矩形（撮影範囲の左上が原点）。\n"
        f"window.SHOTS = {body};\n",
        encoding="utf-8",
    )


def capture_one(page, shot):
    sid = shot["id"]
    page.goto(MOCKUP.as_uri())
    page.add_style_tag(content=INJECT_CSS)
    page.evaluate(HELPERS_JS)
    page.evaluate("document.body.className='pv-landscape'; fitStage();")
    page.evaluate("document.fonts.ready")
    try:
        page.evaluate("() => {" + shot["setup"] + "\n}")
    except Exception as e:  # noqa: BLE001
        raise RuntimeError(f"{sid}: 準備の JS が失敗した: {e}") from e
    page.wait_for_timeout(shot.get("wait", 150))

    app = page.evaluate("(()=>{const r=document.getElementById('app').getBoundingClientRect();return {x:r.left,y:r.top,w:r.width,h:r.height}})()")
    if abs(app["w"] - 1280) > 0.5 or abs(app["h"] - 720) > 0.5:
        raise RuntimeError(f"{sid}: #app が 1280×720 になっていない: {app}")

    if shot.get("clip"):
        r = page.evaluate("s => __rect(s)", shot["clip"])
        if "error" in r:
            raise RuntimeError(f"{sid}: clip の{r['error']}")
        pad = shot.get("pad", 12)
        x0 = max(app["x"], r["x"] - pad)
        y0 = max(app["y"], r["y"] - pad)
        x1 = min(app["x"] + app["w"], r["x"] + r["w"] + pad)
        y1 = min(app["y"] + app["h"], r["y"] + r["h"] + pad)
        clip = {"x": round(x0), "y": round(y0), "width": round(x1 - x0), "height": round(y1 - y0)}
    else:
        clip = {"x": app["x"], "y": app["y"], "width": app["w"], "height": app["h"]}

    warnings = []
    if not shot.get("clip") and not shot.get("allow_scroll"):
        sh, ch = page.evaluate("(()=>{const b=document.getElementById('body');return [b.scrollHeight,b.clientHeight]})()")
        if sh > ch + 2:
            warnings.append(f"本文が {sh - ch}px スクロールしないと見えない")

    marks = []
    for m in shot.get("marks", []):
        n, spec = m[0], m[1]
        pos = m[2] if len(m) > 2 else "tl"
        r = page.evaluate("s => __rect(s)", spec)
        if "error" in r:
            raise RuntimeError(f"{sid}: 注釈 {n} の{r['error']}")
        x, y = r["x"] - clip["x"], r["y"] - clip["y"]
        # 撮影範囲と重なる面積が半分未満なら、写っていないものに番号を振ることになる
        ix = max(0, min(x + r["w"], clip["width"]) - max(x, 0))
        iy = max(0, min(y + r["h"], clip["height"]) - max(y, 0))
        if ix * iy < 0.5 * r["w"] * r["h"]:
            raise RuntimeError(f"{sid}: 注釈 {n}（{spec}）が撮影範囲の外にある")
        # 撮影範囲に切り詰めて記録する
        x0, y0 = max(x, 0), max(y, 0)
        x1, y1 = min(x + r["w"], clip["width"]), min(y + r["h"], clip["height"])
        marks.append({"n": n, "x": round(x0, 1), "y": round(y0, 1), "w": round(x1 - x0, 1), "h": round(y1 - y0, 1), "pos": pos})

    page.screenshot(path=str(IMG / f"{sid}.png"), clip=clip)
    return {"w": clip["width"], "h": clip["height"], "marks": marks}, warnings


def main(argv):
    only = set(argv)
    ids = [s["id"] for s in SHOTS]
    dup = {i for i in ids if ids.count(i) > 1}
    if dup:
        sys.exit(f"ショット id が重複している: {sorted(dup)}")
    unknown = only - set(ids)
    if unknown:
        sys.exit(f"shots.py に無い id: {sorted(unknown)}")
    targets = [s for s in SHOTS if not only or s["id"] in only]

    IMG.mkdir(parents=True, exist_ok=True)
    data = load_existing() if only else {}
    errors, warns = [], []
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport=VIEWPORT, device_scale_factor=SCALE)
        page_errors = []
        page.on("pageerror", lambda e: page_errors.append(str(e)))
        for shot in targets:
            page_errors.clear()
            try:
                data[shot["id"]], w = capture_one(page, shot)
                warns += [f"{shot['id']}: {x}" for x in w]
                if page_errors:
                    errors.append(f"{shot['id']}: ページで例外: {page_errors}")
                print(f"ok   {shot['id']}")
            except Exception as e:  # noqa: BLE001
                errors.append(str(e))
                print(f"FAIL {shot['id']}")
        browser.close()

    if not only:
        # 定義から消したショットの PNG を残さない
        for png in IMG.glob("*.png"):
            if png.stem not in data:
                png.unlink()
    write_shots_js(data)
    for w in warns:
        print("警告:", w)
    for e in errors:
        print("エラー:", e)
    print(f"{len(targets) - len(errors)}/{len(targets)} 枚")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main(sys.argv[1:]))
