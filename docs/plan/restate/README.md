# docs/plan/restate — 再表現による点検

**ここにある文書はどれも正ではない。** `docs/plan/` の設計メモを、
**内容を変えずに表現と構成だけ変えて**書き直し、原文に抜け・曖昧さ・誤解を招く
表現がないかを炙り出すための場所。

書き直した文書と原文が食い違っていたら、**原文が正**。
ここの記述を根拠に実装したり、`VISION.md` を更新したりしてはいけない。

| 原文 | 再表現版 | 点検結果 |
| --- | --- | --- |
| `NewDesign.md` | [NewDesign-restated.md](NewDesign-restated.md) | [NewDesign-restated-findings.md](NewDesign-restated-findings.md) |
| `NewDesign-transit.md` | [NewDesign-transit-restated.md](NewDesign-transit-restated.md) | [NewDesign-details-restated-findings.md](NewDesign-details-restated-findings.md) |
| `NewDesign-onsite.md` | [NewDesign-onsite-restated.md](NewDesign-onsite-restated.md) | 同上 |
| `NewDesign-operationalCheck.md` | [NewDesign-operationalCheck-restated.md](NewDesign-operationalCheck-restated.md) | 同上 |
| `NewDesign-calib.md` | [NewDesign-calib-restated.md](NewDesign-calib-restated.md) | 同上 |

**未着手**: `NewVISION.md` とその詳細ファイル（`onsite-*.md` / `transit-*.md`）。
これらは確定仕様ではなく**意思決定の経緯**を持つ文書なので、再表現の目的が変わる。

## やり方

1. **原文だけを読んで**書き直す。詳細ファイルは見ない。埋めた穴には 〔補完〕 の印を付ける。
2. そのあと詳細ファイルを読み、埋めた穴を分類する。
   * **(a)** 詳細ファイルに書いてある → 設計どおり。原文の欠落ではない
   * **(b)** 詳細ファイルに**違うことが**書いてある → **原文が読み手を誤らせている。要修正**
   * **(c)** どこにも書かれていない → 本当の穴

手順 1 だけで終わると (a) を穴として誤報告する。**(b) は手順 2 でしか見つからない。**

## 注意

* 書き直すと**原文の不整合が無意識に均される**（番号ずれを直す、矛盾する 2 文のうち
  一貫する方だけ採る）。矛盾を見つけたら勝手に直さず、**どちらを採ったかを明記して記録に残す**。
* 「500 ms（実際の動作をみて調整）」「できれば〜したい」「基本的に〜のみ」といった
  **含みは含みのまま残す**。規範的な文体に直すと断定に化けて情報が失われる。
* 穴の多くは既に [NewDesign-checklist.md](../NewDesign-checklist.md) が ID 付きで
  追跡している。**findings には、そこに載っていないものを中心に書く。**
