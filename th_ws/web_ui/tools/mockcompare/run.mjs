#!/usr/bin/env node
// tools/mockcompare/run.mjs — 実画面をモックアップ（docs/plan/spec/mockup/index.html）と数値で比べる。
//
//   node tools/mockcompare/run.mjs --port 4520                     全状態・両方の向き
//   node tools/mockcompare/run.mjs --port 4520 --screen S11 --ori L
//   node tools/mockcompare/run.mjs --port 4520 --state S30-list,S50-run --frame
//   node tools/mockcompare/run.mjs --list                           状態の一覧だけ
//
// 引数:
//   --port N      実画面の vite preview のポート（必須。並行で使うので人ごとに変える。PW_PORT でも可）
//   --screen A,B  画面で絞る（状態名の先頭。S11 / S21 / W1 など）
//   --state A,B   状態名で絞る（完全一致。S11-idle など）
//   --ori L|P     向き（L=横長 1280×720・P=縦長 960×1280。既定は両方）
//   --frame       骨組みの領域だけ比べる（regions.mjs の FRAME）
//   --tol N       位置・大きさの許容（px。既定 2）
//   --no-build    dist を作り直さない（既定は毎回 vite build する）
//   --reuse       そのポートで既に動いているサーバを使う（既定は使わず、埋まっていれば止まる）
//   --no-png      画像を書かない（速い）
//   --all         許容内の行も表に出す
//
// 出力（tools/mockcompare/out/。gitignore 済み）:
//   summary.md               状態ごとの超過の数と大きいもの
//   <状態>-<向き>.md          その状態の差の表（超過に ✗）
//   <状態>-<向き>.side.png    左＝モックアップ・右＝実画面
//   <状態>-<向き>.diff.png    違う画素を赤、超過した領域の枠（桃＝モックアップ・水色＝実画面）
//   results.json             全部の数値
// 終了コード: 許容を超えたものが 1 つでもあれば 1。載せられなかった状態（states.mjs の NA）は数えない。
import { chromium } from '@playwright/test'
import { spawn, spawnSync } from 'node:child_process'
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { STATES, GLOBAL_IGNORE } from './states.mjs'
import { REGIONS, FRAME } from './regions.mjs'

const HERE = path.dirname(fileURLToPath(import.meta.url))
const WEBUI = path.resolve(HERE, '../..')
const REPO = path.resolve(WEBUI, '../..')
const MOCKUP = path.join(REPO, 'docs/plan/spec/mockup/index.html')
const OUT = path.join(HERE, 'out')
const SIZE = { L: { width: 1280, height: 720 }, P: { width: 960, height: 1280 } }

// ---------------- 引数 ----------------
function parseArgs(argv) {
  const a = { port: process.env.PW_PORT || null, tol: 2, build: true, reuse: false, png: true, all: false, frame: false }
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i]
    const v = () => argv[++i]
    if (k === '--port') a.port = v()
    else if (k === '--screen') a.screen = v().split(',')
    else if (k === '--state') a.state = v().split(',')
    else if (k === '--ori') a.ori = v().toUpperCase().split(',')
    else if (k === '--tol') a.tol = Number(v())
    else if (k === '--frame') a.frame = true
    else if (k === '--no-build') a.build = false
    else if (k === '--reuse') a.reuse = true
    else if (k === '--no-png') a.png = false
    else if (k === '--all') a.all = true
    else if (k === '--list') a.list = true
    else if (k === '-h' || k === '--help') a.help = true
    else throw new Error(`知らない引数: ${k}`)
  }
  return a
}

// ---------------- 実画面のサーバ（e2e の playwright.config.js と同じく build → preview） ----------------
async function up(url) {
  try { const r = await fetch(url); return r.ok } catch { return false }
}
async function startServer(args) {
  const url = `http://localhost:${args.port}/`
  if (await up(url)) {
    if (!args.reuse) throw new Error(`ポート ${args.port} は使用中。別のポートにするか、自分のサーバなら --reuse を付ける`)
    console.log(`既に動いているサーバを使う: ${url}（dist が古いと古い画面を測る）`)
    return { url, stop: () => {} }
  }
  if (args.build) {
    console.log('vite build …')
    const r = spawnSync('npx', ['vite', 'build', '--logLevel', 'error'], { cwd: WEBUI, stdio: 'inherit' })
    if (r.status !== 0) throw new Error('vite build が失敗した')
  }
  const child = spawn('npx', ['vite', 'preview', '--port', String(args.port), '--strictPort'],
    { cwd: WEBUI, stdio: 'ignore', detached: true })
  for (let i = 0; i < 100; i++) {
    if (await up(url)) return { url, stop: () => { try { process.kill(-child.pid, 'SIGTERM') } catch {} } }
    await new Promise((r) => setTimeout(r, 150))
  }
  try { process.kill(-child.pid, 'SIGTERM') } catch {}
  throw new Error(`vite preview が ${url} で立ち上がらなかった`)
}

// ---------------- ページの準備 ----------------
const NO_MOTION = '*,*::before,*::after{transition:none!important;animation:none!important;caret-color:transparent!important}'
// docs/manual/tools/capture.py と同じ。外側の切り替え欄（#bar）と説明（#notes）を隠し、#dev を倍率 1 で左上に置く
const MOCK_CSS = `
#bar,#notes{display:none!important}
#main,#stageWrap{display:block!important;position:static!important;width:auto!important;height:auto!important;padding:0!important;margin:0!important}
#devBox{position:fixed!important;left:0;top:0;width:auto!important;height:auto!important}
#dev{transform:none!important;position:absolute!important;left:0!important;top:0!important;box-shadow:none!important}
body{overflow:hidden!important}
${NO_MOTION}`
const MOCK_HELPERS = `
window.__c = sel => { const e = document.querySelector(sel); if (!e) throw new Error('見つからない: ' + sel); e.dispatchEvent(new MouseEvent('click', {bubbles: true, cancelable: true})); };
window.st = (scr, sub, o = {}) => { __c('[data-scr="' + scr + '"]'); if (o.teach !== undefined) { S[scr].teach = o.teach; render(); } if (sub) __c('[data-sub="' + sub + '"]'); };
window.ab = k => __c('[data-ab="' + k + '"]');
window.act1 = (a, id) => __c('#dev [data-act="' + a + '"]' + (id ? '[data-id="' + id + '"]' : ''));
true;`

async function openMock(browser, st, ori) {
  const page = await browser.newPage({ viewport: SIZE[ori] })
  const errors = []
  page.on('pageerror', (e) => errors.push(String(e)))
  await page.goto(pathToFileURL(MOCKUP).href)
  await page.addStyleTag({ content: MOCK_CSS })
  await page.evaluate(MOCK_HELPERS)
  await page.evaluate((o) => { S.ori = o; render() }, ori)
  await page.evaluate(`(() => {${st.mock}\n})()`)
  await page.evaluate('document.fonts.ready')
  await page.waitForTimeout(150)
  const dev = await page.evaluate(() => { const r = document.getElementById('dev').getBoundingClientRect(); return { w: r.width, h: r.height } })
  if (Math.abs(dev.w - SIZE[ori].width) > 0.5 || Math.abs(dev.h - SIZE[ori].height) > 0.5) {
    throw new Error(`モックアップの #dev が ${SIZE[ori].width}×${SIZE[ori].height} でない: ${dev.w}×${dev.h}`)
  }
  return { page, errors, root: '#dev' }
}

async function openReal(browser, url, st, ori) {
  const page = await browser.newPage({ viewport: SIZE[ori] })
  const errors = []
  page.on('pageerror', (e) => errors.push(String(e)))
  const r = st.real
  await page.addInitScript(({ s, scr, seeds }) => {
    window.__thTestState = s
    window.__thTestScreen = scr
    for (const k of Object.keys(seeds)) window[k] = seeds[k]
  }, { s: r.state ?? { mode: 'IDLE' }, scr: r.screen ?? 'S01', seeds: r.seeds ?? {} })
  await page.goto(url)
  await page.locator('#body').waitFor()
  await page.addStyleTag({ content: NO_MOTION })
  if (r.after) await r.after(page)
  await page.evaluate('document.fonts.ready')
  await page.waitForTimeout(200)
  const app = await page.evaluate(() => {
    const a = document.getElementById('app').getBoundingClientRect()
    return { w: a.width, h: a.height, orientation: document.getElementById('stage')?.dataset.orientation }
  })
  if (Math.abs(app.w - SIZE[ori].width) > 0.5 || Math.abs(app.h - SIZE[ori].height) > 0.5) {
    throw new Error(`実画面の #app が ${SIZE[ori].width}×${SIZE[ori].height}（倍率 1）でない: ${app.w}×${app.h}`)
  }
  return { page, errors, root: '#app' }
}

// ---------------- 測る（ページの中で走る） ----------------
function measureInPage({ root, regions }) {
  const R = document.querySelector(root).getBoundingClientRect()
  const vis = (e) => {
    if (!e || !e.getClientRects().length) return false
    // 大きさ 0 の要素も「在る」と数える（場所を空けていない＝0 px の枠は、それ自体が差）
    const cs = getComputedStyle(e)
    return cs.visibility !== 'hidden' && cs.display !== 'none'
  }
  const find = (spec, all) => {
    let out = null
    for (const alt of spec) {          // 先に書いたものから（実画面は data-mc が先）
      let union = null
      let els = []
      for (const part of alt.split(' || ')) {
        const i = part.indexOf('::')
        const css = (i < 0 ? part : part.slice(0, i)).trim()
        const text = i < 0 ? null : part.slice(i + 2).trim()
        let c = [...document.querySelectorAll(`${root} ${css.split(',').join(`, ${root} `)}`)].filter(vis)
        if (text !== null) {
          c = c.filter((e) => e.textContent.replace(/\s+/g, ' ').includes(text))
          c.sort((a, b) => a.textContent.length - b.textContent.length)
        }
        if (!c.length) { union = null; els = []; break }
        if (all) { els = c; break }
        els.push(c[0])
        const r = c[0].getBoundingClientRect()
        union = union
          ? { l: Math.min(union.l, r.left), t: Math.min(union.t, r.top), r: Math.max(union.r, r.right), b: Math.max(union.b, r.bottom) }
          : { l: r.left, t: r.top, r: r.right, b: r.bottom }
      }
      if (all && els.length) { out = els; break }
      if (!all && union) { out = { el: els[0], u: union }; break }
    }
    return out
  }
  const style = (e) => {
    const cs = getComputedStyle(e)
    const bw = Math.max(...['Top', 'Right', 'Bottom', 'Left'].map((s) => parseFloat(cs[`border${s}Width`]) || 0))
    return {
      fontSize: parseFloat(cs.fontSize), fontWeight: cs.fontWeight, color: cs.color,
      background: cs.backgroundColor, bgImage: cs.backgroundImage, fontFamily: cs.fontFamily, radius: parseFloat(cs.borderTopLeftRadius) || 0, borderWidth: bw,
      text: e.textContent.replace(/\s+/g, ' ').trim().slice(0, 24),
    }
  }
  const rel = (u) => ({ x: u.l - R.left, y: u.t - R.top, w: u.r - u.l, h: u.b - u.t })
  const res = {}
  for (const g of regions) {
    if (g.group) {
      const els = find(g.sel, true) || []
      els.forEach((e, i) => {
        const r = e.getBoundingClientRect()
        res[`${g.name}-${i + 1}`] = { rect: rel({ l: r.left, t: r.top, r: r.right, b: r.bottom }), style: style(e) }
      })
    } else {
      const f = find(g.sel, false)
      if (f) res[g.name] = { rect: rel(f.u), style: style(f.el) }
    }
  }
  return res
}

function regionSpecs(side, frameOnly) {
  return REGIONS.filter((g) => !frameOnly || FRAME.has(g.name)).map((g) => ({
    name: g.name, group: !!g.group,
    sel: side === 'real' ? [`[data-mc="${g.name}"]`, g.real] : [g.mock],
  }))
}

// ---------------- 比べる ----------------
const TRANSPARENT = new Set(['rgba(0, 0, 0, 0)', 'transparent'])
const normColor = (c) => (TRANSPARENT.has(c) ? 'transparent' : c)
function compare(mock, real, args, ignore = {}) {
  const rows = []
  const names = [...new Set([...Object.keys(mock), ...Object.keys(real)])]
  const order = REGIONS.map((g) => g.name)
  const base = (n) => order.find((g) => n === g || n.startsWith(`${g}-`) && /-\d+$/.test(n) && n.slice(0, n.lastIndexOf('-')) === g) ?? n
  names.sort((a, b) => order.indexOf(base(a)) - order.indexOf(base(b)) || a.localeCompare(b, 'en', { numeric: true }))
  for (const n of names) {
    const g = REGIONS.find((x) => x.name === base(n))
    const m = mock[n]
    const r = real[n]
    const row = { name: n, mock: m ?? null, real: r ?? null, issues: [] }
    if (!m || !r) {
      row.issues.push(m ? '実画面に無い' : 'モックアップに無い')
      row.max = Infinity
    } else {
      const d = { x: r.rect.x - m.rect.x, y: r.rect.y - m.rect.y, w: r.rect.w - m.rect.w, h: r.rect.h - m.rect.h }
      row.d = d
      row.max = Math.max(...Object.values(d).map(Math.abs))
      for (const k of ['x', 'y', 'w', 'h']) if (Math.abs(d[k]) > args.tol) row.issues.push(`${k}${d[k] > 0 ? '+' : ''}${d[k].toFixed(1)}`)
      if (g?.style) {
        const a = m.style
        const b = r.style
        const want = (k) => g.style === true || g.style.includes(k)
        if (want('font') && Math.abs(a.fontSize - b.fontSize) > 0.5) row.issues.push(`字 ${a.fontSize}→${b.fontSize}`)
        if (want('font') && a.fontWeight !== b.fontWeight) row.issues.push(`太さ ${a.fontWeight}→${b.fontWeight}`)
        if (want('color') && normColor(a.color) !== normColor(b.color)) row.issues.push(`色 ${a.color}→${b.color}`)
        if (want('bg') && normColor(a.background) !== normColor(b.background)) row.issues.push(`背景 ${normColor(a.background)}→${normColor(b.background)}`)
        if (want('bg') && a.bgImage !== b.bgImage) row.issues.push(`背景の絵 ${a.bgImage.slice(0, 60)}→${b.bgImage.slice(0, 60)}`)
        const cap = (s, rect) => Math.min(s.radius, rect.h / 2, rect.w / 2)
        if (want('radius') && Math.abs(cap(a, m.rect) - cap(b, r.rect)) > 1) row.issues.push(`角 ${a.radius}→${b.radius}`)
        if (want('border') && Math.abs(a.borderWidth - b.borderWidth) > 0.5) row.issues.push(`枠 ${a.borderWidth}→${b.borderWidth}`)
        if (row.issues.length && row.max <= args.tol) row.max = Math.max(row.max, args.tol + 0.01)  // 見た目だけの違いも超過に数える
      }
    }
    const ig = ignore[n] ?? ignore[base(n)]
    if (ig && row.issues.length) { row.ignored = ig; row.issues = row.issues.map((x) => `（除外）${x}`) }
    row.over = row.issues.length > 0 && !row.ignored
    row.frame = FRAME.has(n)
    row.group = !!g?.group
    rows.push(row)
  }
  return rows
}

const fmtRect = (x) => (x ? `${Math.round(x.rect.x)},${Math.round(x.rect.y)} ${Math.round(x.rect.w)}×${Math.round(x.rect.h)}` : '—')
function tableMd(title, rows, args, extra = []) {
  const lines = [`# ${title}`, '', ...extra, '', '| 領域 | モックアップ x,y w×h | 実画面 x,y w×h | 差 | |', '|---|---|---|---|---|']
  for (const r of rows) {
    if (!args.all && !r.over && !r.ignored) continue
    lines.push(`| ${r.name} | ${fmtRect(r.mock)} | ${fmtRect(r.real)} | ${r.issues.join(' / ') || 'ok'} | ${r.over ? '✗' : r.ignored ? `除外: ${r.ignored}` : ''} |`)
  }
  return lines.join('\n') + '\n'
}

// ---------------- 画像（ブラウザの canvas で並べる・差を出す。依存を増やさない） ----------------
async function composeImages(browser, mockPng, realPng, rows, sideOut, diffOut) {
  const page = await browser.newPage()
  const boxes = rows.filter((r) => r.over).map((r) => ({ name: r.name, m: r.mock?.rect, r: r.real?.rect }))
  const [side, diff] = await page.evaluate(async ({ a, b, boxes }) => {
    const load = (src) => new Promise((ok) => { const i = new Image(); i.onload = () => ok(i); i.src = src })
    const [A, B] = await Promise.all([load(a), load(b)])
    const W = A.width
    const H = A.height
    const LBL = 28
    const c1 = document.createElement('canvas')
    c1.width = W * 2 + 12
    c1.height = H + LBL
    const g1 = c1.getContext('2d')
    g1.fillStyle = '#222'; g1.fillRect(0, 0, c1.width, c1.height)
    g1.drawImage(A, 0, LBL); g1.drawImage(B, W + 12, LBL)
    g1.fillStyle = '#fff'; g1.font = 'bold 18px sans-serif'
    g1.fillText('モックアップ', 8, 20); g1.fillText('実画面', W + 20, 20)
    const c2 = document.createElement('canvas')
    c2.width = W; c2.height = H
    const g2 = c2.getContext('2d')
    g2.drawImage(A, 0, 0)
    const da = g2.getImageData(0, 0, W, H)
    g2.clearRect(0, 0, W, H); g2.drawImage(B, 0, 0)
    const db = g2.getImageData(0, 0, W, H)
    const o = g2.createImageData(W, H)
    for (let i = 0; i < da.data.length; i += 4) {
      const d = Math.abs(da.data[i] - db.data[i]) + Math.abs(da.data[i + 1] - db.data[i + 1]) + Math.abs(da.data[i + 2] - db.data[i + 2])
      const gray = (da.data[i] * 0.3 + da.data[i + 1] * 0.59 + da.data[i + 2] * 0.11) * 0.35 + 165
      if (d > 48) { o.data[i] = 235; o.data[i + 1] = 30; o.data[i + 2] = 60 } else { o.data[i] = o.data[i + 1] = o.data[i + 2] = gray }
      o.data[i + 3] = 255
    }
    g2.putImageData(o, 0, 0)
    g2.lineWidth = 2
    g2.font = '12px sans-serif'
    for (const x of boxes) {
      if (x.m) { g2.strokeStyle = '#d0f'; g2.strokeRect(x.m.x + 1, x.m.y + 1, x.m.w - 2, x.m.h - 2) }
      if (x.r) { g2.strokeStyle = '#0bd'; g2.setLineDash([5, 3]); g2.strokeRect(x.r.x + 1, x.r.y + 1, x.r.w - 2, x.r.h - 2); g2.setLineDash([]) }
      const p = x.r ?? x.m
      g2.fillStyle = '#000'; g2.fillText(x.name, p.x + 4, p.y + 13)
    }
    return [c1.toDataURL('image/png'), c2.toDataURL('image/png')]
  }, { a: `data:image/png;base64,${mockPng.toString('base64')}`, b: `data:image/png;base64,${realPng.toString('base64')}`, boxes })
  fs.writeFileSync(sideOut, Buffer.from(side.split(',')[1], 'base64'))
  fs.writeFileSync(diffOut, Buffer.from(diff.split(',')[1], 'base64'))
  await page.close()
}

// ---------------- 本体 ----------------
function pickStates(args) {
  return STATES.filter((s) => (!args.screen || args.screen.some((p) => s.name.startsWith(`${p}-`) || s.name === p))
    && (!args.state || args.state.includes(s.name)))
}

async function main() {
  const args = parseArgs(process.argv.slice(2))
  if (args.help) { console.log(fs.readFileSync(fileURLToPath(import.meta.url), 'utf8').split('\n').slice(1, 30).map((l) => l.replace(/^\/\/ ?/, '')).join('\n')); return 0 }
  const states = pickStates(args)
  if (args.list) {
    for (const s of states) console.log(`${s.real?.na ? 'NA ' : '   '}${s.name}${s.real?.na ? `  （${s.real.na}）` : ''}`)
    return 0
  }
  if (!args.port) throw new Error('--port を指定する（例: --port 4520。並行で使うので人ごとに変える）')
  if (!states.length) throw new Error('絞り込みに合う状態が無い（--list で一覧）')
  const oris = args.ori ?? ['L', 'P']
  fs.mkdirSync(OUT, { recursive: true })
  const server = await startServer(args)
  const browser = await chromium.launch()
  const results = []
  try {
    for (const st of states) {
      for (const ori of st.oris ?? oris) {
        if (!oris.includes(ori)) continue
        const id = `${st.name}-${ori}`
        if (st.real?.na) { results.push({ id, state: st.name, ori, na: st.real.na }); continue }
        try {
          const mk = await openMock(browser, st, ori)
          const rl = await openReal(browser, server.url, st, ori)
          const m = await mk.page.evaluate(measureInPage, { root: mk.root, regions: regionSpecs('mock', args.frame) })
          const r = await rl.page.evaluate(measureInPage, { root: rl.root, regions: regionSpecs('real', args.frame) })
          const rows = compare(m, r, args, { ...GLOBAL_IGNORE, ...(st.ignore ?? {}) })
          const over = rows.filter((x) => x.over)
          const pageErrors = [...mk.errors.map((e) => `モックアップ: ${e}`), ...rl.errors.map((e) => `実画面: ${e}`)]
          fs.writeFileSync(path.join(OUT, `${id}.md`), tableMd(`${id}（許容 ±${args.tol} px）`, rows, args,
            [`超過 ${over.length} / ${rows.length} 領域`, ...pageErrors.map((e) => `- ページの例外: ${e}`)]))
          if (args.png) {
            const a = await mk.page.screenshot({ clip: { x: 0, y: 0, ...SIZE[ori] } })
            const b = await rl.page.screenshot({ clip: { x: 0, y: 0, ...SIZE[ori] } })
            await composeImages(browser, a, b, rows, path.join(OUT, `${id}.side.png`), path.join(OUT, `${id}.diff.png`))
          }
          await mk.page.close(); await rl.page.close()
          results.push({ id, state: st.name, ori, rows, over: over.length, total: rows.length, pageErrors })
          console.log(`${over.length ? '✗' : '✓'} ${id.padEnd(18)} 超過 ${String(over.length).padStart(2)} / ${rows.length}`
            + (over.length ? `  ${over.sort((x, y) => y.max - x.max).slice(0, 3).map((x) => `${x.name}(${x.issues[0]})`).join(' ')}` : ''))
        } catch (e) {
          results.push({ id, state: st.name, ori, error: String(e.message ?? e) })
          console.log(`! ${id.padEnd(18)} ${e.message ?? e}`)
        }
      }
    }
  } finally {
    await browser.close()
    server.stop()
  }
  writeSummary(results, args)
  fs.writeFileSync(path.join(OUT, 'results.json'), JSON.stringify(results, (k, v) => (v === Infinity ? 'missing' : v), 1))
  const bad = results.filter((x) => x.over || x.error).length
  console.log(`\n${results.filter((x) => !x.na).length} 状態を測った（NA ${results.filter((x) => x.na).length}）。超過あり ${bad}。表: ${path.relative(process.cwd(), path.join(OUT, 'summary.md'))}`)
  return bad ? 1 : 0
}

function writeSummary(results, args) {
  const L = [`# モックアップとの差（許容 ±${args.tol} px${args.frame ? '・骨組みだけ' : ''}）`, '',
    '骨組み＝regions.mjs の FRAME（ヘッダ・非常停止・操作バー・主表示・右列・一覧・手順の外枠）。',
    '番号＝番号で対にした領域（右列のボタンなど。1 個ずれると後ろも連鎖して出る）。除外＝states.mjs の ignore（理由つき）。', '',
    '| 状態 | 骨組み | 中身 | 番号 | 除外 | 大きいもの（骨組み・中身の上位 5） |', '|---|---|---|---|---|---|']
  for (const x of results) {
    if (x.na) L.push(`| ${x.id} | NA | | | | ${x.na} |`)
    else if (x.error) L.push(`| ${x.id} | エラー | | | | ${x.error.replace(/\|/g, '/').slice(0, 200)} |`)
    else {
      const ov = x.rows.filter((r) => r.over)
      const fr = ov.filter((r) => r.frame)
      const gr = ov.filter((r) => r.group)
      const ct = ov.filter((r) => !r.frame && !r.group)
      const top = [...fr, ...ct].sort((a, b) => (b.frame - a.frame) || (b.max - a.max)).slice(0, 5)
        .map((r) => `${r.name}: ${r.issues.slice(0, 2).join(' ')}`).join('<br>')
      L.push(`| ${x.id} | ${fr.length} | ${ct.length} | ${gr.length} | ${x.rows.filter((r) => r.ignored).length} | ${top} |`)
    }
  }
  fs.writeFileSync(path.join(OUT, 'summary.md'), L.join('\n') + '\n')
}

main().then((c) => process.exit(c), (e) => { console.error(String(e.message ?? e)); process.exit(2) })
