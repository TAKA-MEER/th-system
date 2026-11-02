/* 図解マニュアルの注釈を描く。
   img/shots.js（tools/capture.py が書く window.SHOTS）の矩形を読み、
   <figure class="shot" data-shot="ID"> に画像・枠・番号丸を重ねる。

   figure の属性:
     data-shot   ショット id（img/ID.png）
     data-marks  一部の番号だけ描くとき "1,3,5"（省略で全部）
     data-k      切り抜き図の表示倍率 [mm/論理px]（省略 0.17。full クラス・段組みの中・style で幅を
                 決めた図には効かない）
   <ol class="legend" data-shot="ID"> の <li data-n="N"> と番号の対応を検査する。
   食い違いは console.error と window.__manualErrors に出す（tools/make_pdf.py が拾う）。 */
(function () {
  const errs = [];
  const S = window.SHOTS || {};
  const MM = 96 / 25.4; // CSS px / mm
  const POS = {
    tl: [0, 0], t: [.5, 0], tr: [1, 0], l: [0, .5], c: [.5, .5], r: [1, .5], bl: [0, 1], b: [.5, 1], br: [1, 1],
  };

  // ページ（.sheet）とショットの組ごとに、実際に描いた番号を覚えておく（凡例との突き合わせ用）。
  // 同じショットを別のページで一部の番号だけ使うことがあるので、ページ単位で突き合わせる
  const sheets = [...document.querySelectorAll('.sheet')];
  const keyOf = (el, id) => sheets.indexOf(el.closest('.sheet')) + '/' + id;
  const drawn = {};

  document.querySelectorAll('figure.shot[data-shot]').forEach(fig => {
    const id = fig.dataset.shot;
    const d = S[id];
    // 画像と注釈は .frame にまとめる。figcaption を含めた figure 全体を基準にすると枠が下にずれる
    const frame = document.createElement('div');
    frame.className = 'frame';
    const img = document.createElement('img');
    img.src = 'img/' + id + '.png';
    img.alt = fig.dataset.alt || '';
    frame.appendChild(img);
    fig.prepend(frame);
    if (!d) {
      errs.push('img/shots.js に無いショット: ' + id);
      const e = document.createElement('div');
      e.className = 'fig-error'; e.textContent = 'ショットが見つかりません: ' + id;
      frame.appendChild(e);
      return;
    }
    // 段組み（.cols）の中では列幅いっぱい。幅を style で決めた図はそのまま
    if (!fig.classList.contains('full') && !fig.closest('.cols') && !fig.style.width) {
      const k = parseFloat(fig.dataset.k || '0.17');
      fig.style.width = (d.w * k) + 'mm';
    }
    // 画像が読み込まれる前でも枠の位置が決まるよう、縦横比を先に与える
    img.style.aspectRatio = d.w + ' / ' + d.h;

    const only = fig.dataset.marks ? fig.dataset.marks.split(',').map(Number) : null;
    const W = frame.getBoundingClientRect().width;    // 図の表示幅 [CSS px]
    const H = W * d.h / d.w;
    const r = (5.2 * MM) / 2 + 1;                    // 番号丸の半径 + 余白 [CSS px]
    const gap = (5.2 * MM) / 2 + 1.5;                // 番号丸を対象の外側へ出す量（文字を隠さない）
    const key = keyOf(fig, id);
    drawn[key] = drawn[key] || new Set();
    d.marks.forEach(m => {
      if (only && !only.includes(m.n)) return;
      drawn[key].add(m.n);
      const box = document.createElement('div');
      box.className = 'mk';
      box.style.left = (m.x / d.w * 100) + '%';
      box.style.top = (m.y / d.h * 100) + '%';
      box.style.width = (m.w / d.w * 100) + '%';
      box.style.height = (m.h / d.h * 100) + '%';
      frame.appendChild(box);

      // 番号丸は、指定した辺・角の外側に置く（角は斜め外）。図の外へ出るときは内側へ戻す
      const [fx, fy] = POS[m.pos] || POS.tl;
      const ox = (fx - .5) * 2, oy = (fy - .5) * 2;
      const diag = (ox && oy) ? Math.SQRT1_2 : 1;
      let cx = (m.x + m.w * fx) / d.w * W + ox * gap * diag;
      let cy = (m.y + m.h * fy) / d.h * H + oy * gap * diag;
      // 左右は図の外へ 3.2mm まで出してよい（ページの余白・段の間に出る）。上下は図の中に収める
      const out = 3.2 * MM;
      cx = Math.min(Math.max(cx, r - out), W - r + out);
      cy = Math.min(Math.max(cy, r), H - r);
      const n = document.createElement('div');
      n.className = 'mn';
      n.textContent = m.n;
      n.style.left = (cx / W * 100) + '%';
      n.style.top = (cy / H * 100) + '%';
      frame.appendChild(n);
    });
  });

  // 凡例と番号の突き合わせ
  const legended = new Set();
  document.querySelectorAll('ol.legend[data-shot]').forEach(ol => {
    const id = ol.dataset.shot;
    const key = keyOf(ol, id);
    legended.add(key);
    const want = drawn[key];
    if (!want) { errs.push('凡例と同じページに図が無い: ' + id); return; }
    const got = new Set([...ol.querySelectorAll(':scope > li')].map(li => Number(li.dataset.n)));
    const miss = [...want].filter(n => !got.has(n));
    const extra = [...got].filter(n => !want.has(n));
    if (miss.length) errs.push(id + ': 凡例に無い番号 ' + miss.join(','));
    if (extra.length) errs.push(id + ': 図に無い番号が凡例にある ' + extra.join(','));
  });
  Object.keys(drawn).forEach(key => {
    if (drawn[key].size && !legended.has(key)) {
      const [i, id] = key.split('/');
      errs.push(id + ': 番号を描いたのに同じページに凡例が無い（p.' + (Number(i) + 1) + '）');
    }
  });

  // 目次のページ番号（シートの通し番号）を埋める
  document.querySelectorAll('.toc a[href^="#"]').forEach(a => {
    const t = document.getElementById(a.getAttribute('href').slice(1));
    const pg = a.closest('li').querySelector('.pg');
    if (!t) { errs.push('目次のリンク先が無い: ' + a.getAttribute('href')); return; }
    const s = t.closest('.sheet');
    if (pg && s) pg.textContent = sheets.indexOf(s) + 1;
  });

  window.__manualErrors = errs;
  errs.forEach(e => console.error('[manual] ' + e));
})();
