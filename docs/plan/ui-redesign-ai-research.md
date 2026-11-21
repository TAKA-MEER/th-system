> 外部の調査の写し（2026-11-21 にユーザーが Opus 5.5 max で作成し、共有）。**正ではない参考資料。**このプロジェクトへの当てはめは [ui-redesign-ai-tells.md](ui-redesign-ai-tells.md)。本文は手を入れずに写した。

# ロボット操作UIから「AIっぽさ」を取り除く：調査まとめ

Nov 21, 2026 · @yisahia787

## 要点

ロボット操作UIの「AIっぽさ」は、生成AIが選びがちな初期値、AI機能を誇示する演出、AIらしい言葉づかいの3つが重なって生まれます。直す基準は一つで、画面上の色・動き・装飾が、ロボットの状態か操作の意味に対応しているかどうかです。

最優先で直すこと：

- **色は状態のためだけに使う。** 通常時はグレー基調にし、色は異常と要対応に限ります（産業HMIのISA-101の考え方）。赤と黄の組み合わせは非常停止専用です。
- **AI演出を外す。** 紫〜青のグラデーション、発光、ガラス調、shimmer（光が横切る文字）、スパークル（四芒星）アイコンは、ロボットの状態を何も伝えません。
- **データに見える装飾を消す。** 状態と無関係な色の点、常に緑の「オンライン」表示、架空の数値は、見た目の問題を超えて誤認の原因になります。
- **擬人化しない。** 顔・目・「考え中…」・一人称の語りより、次の動作、止まった理由、再開の手順を示します。
- **文言は具体的に。** 対象・状態・数値と単位・次の操作を書き、誇張語や決まり文句を削ります。
- **迷ったら一文テスト。** その選択の理由を一文で言えなければ、それは決定ではなく初期値の残りです（[braindetox](https://braindetox.kr/ja/posts/ai_design_tell_detection.html)）。

## 「AIっぽさ」の3つの層

資料で「AIっぽい」と呼ばれるものは、3層に分けると整理できます。ロボット操作UIではどの層も好みの問題では済まず、状態の誤認や能力の過信につながります。

| 層 | 典型例 | 出どころ | ロボット操作UIでのリスク |
| --- | --- | --- | --- |
| 1. AI生成っぽい見た目 | 紫〜青のグラデーション、Inter一色、中央寄せ見出し＋カード3枚、全要素に同じ角丸と影、絵文字アイコン | 生成モデルが指示の空白を学習データの最頻値で埋める（[Anthropic](https://claude.com/blog/improving-frontend-design-through-skills)は「distributional convergence」と呼ぶ） | 重要度の差が消え、異常が目立たなくなる |
| 2. AI機能の演出 | スパークル（四芒星）アイコン、発光やshimmer、「Thinking…」表示、常設のチャット欄、顔や目のキャラクター | 「AIを使っている」と見せたい製品側の都合 | 記号の意味が曖昧になり、能力を実際より大きく見せる |
| 3. AIっぽい言葉と振る舞い | 誇張語、決まり文句、見出しの絵文字、当たり前の説明文、何が起きたか分からないエラー文 | 生成された文言をレビューせずに使う | 警告やエラーを見ても、次に何をすべきか分からない |

層1は「誰も決めていない」こと、層2と3は「中身より演出」が透けることとして批判されています。

## AIっぽく見える要素カタログ

下の2つの表は、公式文書・批評記事・コミュニティで繰り返し挙がるサインと、ロボット操作UIでの直し方です。1つ当てはまるだけでAI製とは限らず、複数が重なると「量産品」に見えます（[Adrian Krebs](https://www.adriankrebs.ch/blog/design-slop/)の分析も同じ立場）。

### 見た目

| 要素 | AIっぽいサイン | なぜ出るか | 直し方（ロボット操作UI） | 出典 |
| --- | --- | --- | --- | --- |
| 配色 | 紫〜青（インディゴ）のグラデーション、白地に紫 | Tailwind UIの初期値bg-indigo-500が学習データに大量に入った。作者は2025年8月に冗談めかして謝罪 | 背景はニュートラルなグレー。色は状態の意味に割り当てる | [Anthropic](https://claude.com/blog/improving-frontend-design-through-skills)、[Alan West](https://dev.to/alanwest/why-every-ai-built-website-looks-the-same-blame-tailwinds-indigo-500-3h2p) |
| ネオン多色 | 高彩度の色が5〜6色同時に競合 | 「モダンなSaaS」の学習例に多く、文脈を問わず使われる | ニュートラル1系統＋アクセント1色。色を共有資源として配分する | [Fountain Institute](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui) |
| 発光・ガラス調 | 暗背景に光る文字、色付きの大きな影、全パネル半透明＋ぼかし | ゲーム・暗号資産・AIのローンチサイトの文法 | 奥行きは文字のコントラストと面の段差で作る。挙動と無関係な効果は外す | [Fountain Institute](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui)、[Impeccable](https://www.mintlify.com/pbakaus/impeccable/concepts/anti-patterns) |
| 書体 | Inter・Roboto・Geist一色、階層はウェイト差だけ | 「無難」な既定値 | 役割で選ぶ。和文はUD書体など可読性優先、数値は等幅数字 | [Ask HN](https://news.ycombinator.com/item?id=49290499)、[braindetox](https://braindetox.kr/ja/posts/ai_design_tell_detection.html) |
| レイアウト | 中央寄せ見出し＋アイコン付きカード3枚、均一な余白 | テンプレート（shadcn/ui、Tailwind UI）の骨格の再生産 | 作業の優先度で面積を配分。状態・停止・現在の動作を最大に | [Alan West](https://dev.to/alanwest/how-to-fix-the-ai-generated-look-in-your-frontend-1ahh)、[Utsubo](https://www.utsubo.com/ja/blog/built-with-ai-trust-signals-2026) |
| カード・角丸・影 | 何でもカード化、カード左端の色帯、全要素に同じ大きな角丸と薄い影 | 重要度を考えずに容器と装飾を足す | 余白と近接でまとめる。角丸は2〜3段階、影は浮かせる要素だけ | [Fountain Institute](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui)、[Adrian Krebs](https://www.adriankrebs.ch/blog/design-slop/)、[braindetox](https://braindetox.kr/ja/posts/ai_design_tell_detection.html) |
| 絵文字・ポップなアイコン | ナビや見出しに絵文字、3D風・ベタ塗りのアイコン | マーケティング文の「視覚的な句読点」を学習 | 線画1系統のアイコン＋テキストラベル | [Fountain Institute](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui)、[libecity](https://library.libecity.com/articles/01KRKHK607NHNQ4DZPYG7JMGHS) |
| 意味のないステータス点 | 状態と無関係な色の点、常に緑の「オンライン」 | 「色の点＝ステータス」という連想だけが残る | すべての表示を定義済みの状態に結び、凡例を出す。状態が変わらないなら消す | [Fountain Institute](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui)、[GIGAZINE](https://gigazine.net/news/20260609-performative-ui/) |
| 目的のないモーション | 常時の微アニメ、数字のカウントアップ、shimmer | 「生きている感」を出す演出 | 動きは状態変化と操作への応答だけに使う | [Utsubo](https://www.utsubo.com/ja/blog/built-with-ai-trust-signals-2026)、[Control.com](https://control.com/technical-articles/going-gray/) |
| 見出しまわりの癖 | 全見出しの上の小ラベル、見出しの1語だけ色替え、大文字の小見出し | テンプレートの装飾パターン | 外す。強調は1画面1か所 | [テクノエッジ](https://www.techno-edge.net/article/2026/11/03/5660.html)、[Adrian Krebs](https://www.adriankrebs.ch/blog/design-slop/) |
| 偽物の中身・欠けた状態 | 架空の数値、動かないダミーフォーム。空状態・エラー・オフラインが未設計 | 体裁を優先し、例外状態は後回しになる | 実機・実データで確認。通信断や非常停止中の画面を先に作る | [テクノエッジ](https://www.techno-edge.net/article/2026/11/03/5660.html)、[Business Insider（転載）](https://dnyuz.com/2026/07/07/there-are-3-telltale-signs-that-you-used-ai-to-make-your-app-and-they-arent-pretty/) |

### 言葉と振る舞い

| サイン | 例 | 直し方 | 出典 |
| --- | --- | --- | --- |
| 誇張語・バズワード | 「次世代」「革新的」「シームレスに最適化」 | 数値、機能名、変化の前後に置き換える | [Utsubo](https://www.utsubo.com/ja/blog/built-with-ai-trust-signals-2026) |
| 技術を主語にした長い説明 | 「高度なAIアルゴリズムにより生成されました」 | 短く平易に。例：「AIで作成。使う前に確認してください」 | [SAP Fiori](https://www.sap.com/design-system/fiori-design-ios/v26-4/in-app-ai-design/foundation/ai-ui-text) |
| 当たり前の説明文 | ボタン名を言い換えただけの補足 | 消す。前提条件や注意だけを書く | [Ask HN](https://news.ycombinator.com/item?id=49290499) |
| 記号の癖 | 見出しの絵文字、em-dash（—）の多用、何でも箇条書き | 使わない。文で書けるところは文で | [Ask HN](https://news.ycombinator.com/item?id=49290499)、[Pangram](https://www.pangram.com/ja/signs-of-ai-writing) |
| 擬人化・一人称 | 「私が考えています」「お手伝いします！」、相棒やチームメイトという表現 | 機能として書く。例：「把持位置を計算中（3/5）」 | [Google開発者スタイルガイド](https://developers.google.com/style/anthropomorphism)、[Wikimedia Codex](https://doc.wikimedia.org/codex/latest/style-guide/machine-assistance.html) |
| ぼかしたエラー | 「問題が発生しました」のように丁寧だが中身がない | 何が、どこで、次に何をするかを書く | [Built In](https://builtin.com/articles/ai-design-slop-era) |
| AI機能の記号 | AI機能に四芒星アイコンだけ | ラベルを併記する。必要がなければ表示しない | [NN/g](https://www.nngroup.com/articles/ai-sparkles-icon-problem/) |
| 日本語の癖 | 前置き、過剰な丁寧さ、毎回同じ構成 | いきなり本題。言い切るところは言い切る | [天秤AI](https://tenbin.ai/media/ai_tips/ai-writing-remove-ai-slop) |

## なぜ不快に感じられるのか

不快感の正体は、画面が「誰も決めていない」「手を抜いた」と読めてしまうことです。操作UIでは、それがそのまま安全への不信と誤認につながります。

- **誰も決めていない。** 生成モデルは指示の空白を学習分布の最頻値で埋めます（[Anthropic](https://claude.com/blog/improving-frontend-design-through-skills)）。結果は悪くはないが固有性がなく、Impeccable代表のPaul Bakaus氏は「アルゴリズム版のユニクロかイケア」と表現しました（[Business Insider（転載）](https://dnyuz.com/2026/07/07/there-are-3-telltale-signs-that-you-used-ai-to-make-your-app-and-they-arent-pretty/)）。
- **手抜きのシグナル。** デザイナーの川合卓也氏は、AIっぽさの違和感は手抜きが見抜かれたサインであり、きれいでも信用できないと感じさせると述べています（[プレジデント](https://president.jp/articles/-/121884)）。NN/gも、AI疲れした利用者が「人の手で作られた」ように見えるデザインを信頼の手がかりにし始めたと報告しています（[NN/g](https://www.nngroup.com/articles/handmade-designs/)）。
- **中身より演出。** スパークルが示す「魔法」は、AIの誤りや偏りといったリスクを軽く見せる恐れがあります（[NN/g](https://www.nngroup.com/articles/ai-sparkles-icon-problem/)）。
- **操作UIでは誤認と過信になる。** 状態と結びつかない色や点は「データに見える装飾」で、使い勝手を積極的に損ねます（[Fountain Institute](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui)）。米海軍の委託研究では、単純で抽象的なグラフィックの利用者が、ゲーム風グラフィックの利用者より70%高い成績を出しました。利用者自身はゲーム風を強く好んでいたにもかかわらずです（[Automation World](https://www.automationworld.com/control/article/22031298/the-case-for-more-colorful-hmi-graphics)）。

## 公式文書・標準が示すこと

AIベンダー自身が「AIっぽさ」を名指しで避けるよう求めており、各社のガイドラインは「AIを目立たせる」より「意味と限界を明確にする」方向で一致しています。ロボット操作UIでは、産業HMIと機械安全の規格が、AI演出を外す直接の根拠になります。

### AI・UIのガイドライン

| 文書 | 発行元 | このテーマに関係する要点 |
| --- | --- | --- |
| [Improving frontend design through Skills](https://claude.com/blog/improving-frontend-design-through-skills) | Anthropic | 指示がないとLLMはInter、白地に紫のグラデーション、控えめなアニメーションに収束する。避けるものとして、使われすぎた書体、紫グラデーションなど決まりきった配色、予測できるレイアウト、文脈のない型通りのデザインを挙げる |
| [The Proliferation and Problem of the Sparkles Icon](https://www.nngroup.com/articles/ai-sparkles-icon-problem/) | Nielsen Norman Group（Kate Kaplan、2024年9月） | スパークルの意味は曖昧で、調査では参加者の誰もAIと結びつけなかった。星アイコン（保存・お気に入り）と混同されやすい。使うならラベルを付ける |
| [All That Sparkles Is AI](https://design.google/library/ai-sparkle-icon-research-pozos-schmidt) | Google Design（Rose Pozos、Lennard Schmidt） | Google製品の文脈ではSparkleはAI機能として認識された。ただしどんなAIか、何が起きるかは伝わらない。正確さが必要な場面ではラベルなど他の手がかりと組み合わせる |
| [Guidelines for Human-AI Interaction](https://www.microsoft.com/en-us/haxtoolkit/library/) | Microsoft Research（2019年） | 18の指針。何ができるかと、どれくらいうまくできるかを明確にする。効率的な却下と訂正を支える。なぜそうしたかを明確にする。全体を制御できる操作を用意する |
| [People + AI Guidebook（Mental Models）](https://pair.withgoogle.com/chapter/People%20+%20AI%20Guidebook%20-%20Mental%20Models.pdf) | Google PAIR | AIを人間らしく見せる利点と欠点を検討する。アルゴリズムであることと限界を伝え、意図しない欺瞞を避ける。根底の技術を強調しない |
| [Anthropomorphism](https://developers.google.com/style/anthropomorphism) | Google開発者ドキュメント スタイルガイド | ソフトウェアやハードウェアに人間の性質を与えない |
| [Writing for machine-assisted experiences](https://doc.wikimedia.org/codex/latest/style-guide/machine-assistance.html) | Wikimedia Codex | 技術ではなく人を中心に書く。システムを相棒・コーチ・チームメイトとして描かない |
| [AI UI Text](https://www.sap.com/design-system/fiori-design-ios/v26-4/in-app-ai-design/foundation/ai-ui-text) | SAP Fiori | AIの説明は平易で簡潔に。長い技術語の言い回しを避ける |
| [AI for Product](https://design.morningstar.com/systems/product/ai) | Morningstar Design System | AIのロゴマークはAI機能とAI生成コンテンツの識別に使い、装飾目的では使わない |

### 産業HMI・機械安全の規格

| 文書 | 要点 | 出典 |
| --- | --- | --- |
| ANSI/ISA-101（High Performance HMI） | 画面はグレー基調にし、色は警報と異常な状態に限る。色の用途は事前に決める。派手な3D、アニメーション、色の多用は画面を散らかす。現場で「Gray is Good」と教える例もある | [Control.com](https://control.com/technical-articles/going-gray/)、[ISA Interchange](https://blog.isa.org/the-high-performance-hmi) |
| 高性能HMIの実装ガイド | 状態は色だけでなく説明テキストを併記する。色覚の多様性や、環境による色の見え方の違いに備えるため | [Inductive Automation](https://docs.inductiveautomation.com/docs/7.9/visualization-and-dashboards/understanding-components/high-performance-hmi-techniques) |
| IEC 60204-1（機械の電気装置） | 押しボタンの色は美観で選ばない。表示灯の色は赤＝緊急・危険、黄＝異常、緑＝正常、青＝操作が必要、白＝その他 | [Schneider Electric FAQ](https://www.se.com/uk/en/faqs/FAQ000283554/)、[Industrial Monitor Direct](https://industrialmonitordirect.com/blogs/knowledgebase/industrial-pilot-light-color-coding-iec-nfpa-79-standards-reference) |
| ISO 13850（非常停止） | 非常停止は赤い操作部と黄色の背景。この組み合わせは非常停止専用 | [Schneider Electric FAQ](https://www.se.com/uk/en/faqs/FAQ000283554/) |

規格本文は有償のため、ここでは解説記事とメーカーFAQを出典にしています。設計に反映する前に、規格本文で確認してください。

## 現場の声：批評から愚痴まで

英語圏でも日本語圏でも、不満の中心は「どれも同じ顔」と「中身のない演出」の2つです。愚痴は具体的なので、そのまま禁止リストに使えます。

| 声の出どころ | 種類 | 言われていること |
| --- | --- | --- |
| Tailwind CSS作者 Adam Wathan氏のX投稿（2025年8月、[Alan Westの解説](https://dev.to/alanwest/why-every-ai-built-website-looks-the-same-blame-tailwinds-indigo-500-3h2p)） | 作者本人の自虐 | Tailwind UIのボタンをbg-indigo-500にしたせいで、世界中のAI生成UIがインディゴになったと冗談めかして謝罪 |
| [Show HNの分析（Adrian Krebs）](https://www.adriankrebs.ch/blog/design-slop/)、日本語の紹介は[GIGAZINE](https://www.gigazine.net/news/20260423-design-slop/) | データ分析 | Show HNの1,590サイトを自動判定し、4パターン以上が22%、2〜3パターンが32%。話を聞いたデザイナーは、カード左端の色帯をem-dashと同じくらい確実なサインと評した |
| [Ask HN: How can you tell a site/app is vibe coded?](https://news.ycombinator.com/item?id=49290499) | 愚痴・観察 | Inter、強い角丸とグラデーション、文字の暗いグロー、1pxの枠線と大きなぼかし影、紺のグラデーション、見出しの大きな絵文字、当たり前の説明文、em-dash |
| [performative-ui（GIGAZINEの紹介）](https://gigazine.net/news/20260609-performative-ui/) | パロディ | AIスタートアップの演出を茶化した実在のReact部品集。実際の状態と無関係に常に緑のStatusDot、四芒星、紫と青のグラデーション文字、glowやshimmerのボタンなど |
| [Nobody asked for the sparkle button](https://uxdesign.cc/nobody-asked-for-the-sparkle-button-ae54ab5065c6) | 批評 | どの製品にも同じAIパネルとスパークルが付き、人はオフスイッチを探している。「AI機能」ではなく「AIで作った機能」を作れという主張 |
| [The AI Purple Problem](https://www.indiehackers.com/post/the-ai-purple-problem-why-every-ai-brand-looks-the-same-6cb0aa2a02) | 愚痴・議論 | 紫の光はAIを表す定番になり、ロボットや回路入りの脳のイメージはもう寒い。コメント欄には、紫グラデーションが信頼の負債になった、同業と同じに見えるのでやめた、という声 |
| [7 Signs a UI Has Been Vibe Coded](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui) | 批評 | ネオン多色、装飾的なグロー、絵文字のUI部品、紫グラデーション、何でもカード、多色の左タブ、意味のないステータス点 |
| [プレジデント（川合卓也氏）](https://president.jp/articles/-/121884) | 一般誌の寄稿 | AIバレの違和感は手抜きがバレること。誰に何を伝えるか決める最初の5%と、最終確認の最後の5%は人がやる |
| [Utsubo](https://www.utsubo.com/ja/blog/built-with-ai-trust-signals-2026) | 制作会社のブログ | AIデフォルトの見た目は、昔の「握手するビジネスマン」のストック写真のように手抜きのサインとして読まれ始めた、という分析を紹介。12の兆候を整理 |
| [libecity（みずの氏）](https://library.libecity.com/articles/01KRKHK607NHNQ4DZPYG7JMGHS) | 個人の実践記事 | 「AIが作りました感が消えなくて、人に見せるのが恥ずかしい」という悩みを、Claude Codeを使い始めた人がほぼ全員通る道としている |
| [braindetox](https://braindetox.kr/ja/posts/ai_design_tell_detection.html) | 個人ブログ | 表紙が紫から青へ流れているだけで、本文を読む前に判断が終わってしまう。9項目のtellと、理由を一文で言えるかという判断基準を提示 |
| [Automation World](https://www.automationworld.com/products/control/blog/13311220/high-performance-hmi-hurdle) | 産業HMI側の反論 | グレーの高性能HMI画面は「退屈」で、数時間見ていると「眠くなる」という声もある |

## ロボット操作UIへの適用

ロボット操作UIでは、AIっぽさを消す作業は「状態・安全・操作を一目で正しく伝える」作業とほぼ同じです。目指すのは目立つ独自性ではなく、静かで、すべてに理由がある画面です。

Web向けのアンチAIスロップ論は「大胆で記憶に残る」デザインを勧めることが多く、Anthropicの指示例も背景にグラデーションを重ねて雰囲気を出すよう勧めています（[Anthropic](https://claude.com/blog/improving-frontend-design-through-skills)）。これはランディングページ向けでISA-101の考え方とは逆なので、操作UIにはそのまま持ち込まないでください。

### 1. 色を「意味の予約語」にする

- 通常運転中の画面はグレー基調にし、色は異常・要対応・安全状態だけに使う。
- 状態色はIEC 60204-1の意味に揃える。ブランド色やアクセント色に状態色と紛らわしい色を使わない。特に青は「操作が必要」と意味が重なる。
- 赤と黄の組み合わせは非常停止専用。画面上の装飾やソフトウェアのボタンに流用しない。
- 色だけで状態を伝えず、必ず文字と形を併記する。

### 2. 動きは状態の変化だけ

- 常時アニメーション、shimmer、数字のカウントアップ、脈打つ光は外す。
- 点滅は「注意が必要」の合図として予約する。IEC 60204-1の解説では、機械を止めない異常の注意喚起に黄の点滅を使う例が示されている（[Industrial Monitor Direct](https://industrialmonitordirect.com/blogs/knowledgebase/en-60204-1-indicator-light-color-coding-for-machine-status)）。
- 「生きている感」が欲しいなら、演出ではなく実データ（最終更新時刻、通信遅延の実測値）で示す。

### 3. 最大の面積を「状態」に使う

- 最優先の情報は、運転モード（自動・手動・ティーチ）、安全状態（非常停止・保護停止・協働速度）、現在と次の動作、速度オーバーライド、警報。
- 端の状態を先に設計する。通信断、非常停止中、保護停止、原点未復帰、センサ異常、データなし。
- 遠隔操作では情報を詰め込まない。MITがDARPA Robotics ChallengeでAtlasを操作したDirectorの論文は、直感的でないUIは不要な情報で操作者を圧倒し、重大な誤操作のリスクを高めると述べている（[Oxford Research Archive](https://ora.ox.ac.uk/objects/uuid:29b467db-4ced-4e0f-ba6b-406bb781ab81)）。同システムでは、動作計画を実行前に操作者が承認できる（[MIT CSAIL](https://groups.csail.mit.edu/robotics-center/public_papers/Marion16.pdf)）。

### 4. 擬人化しない、能力を盛らない

- 産業用途では、顔や目は逆効果になりえる。協働ロボットに目を付けると信頼はわずかに上がったが作業成績は下がった研究がある（[Pilacinskiほか](https://baes.uc.pt/bitstream/10316/112093/1/The%20robot%20eyes%20don%27t%20have%20it.%20The%20presence%20of%20eyes%20on%20collaborative%20robots%20yields%20marginally%20higher%20user%20trust%20but%20lower%20performance.pdf)）。擬人化はロボットの道具らしさを損ね、信頼や信頼性の評価を下げることもある（[Roesler 2023](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10512549/)）。
- ただし、次にどこへ動くかの予告には効果がある。目による予告で対象を早く見つけられた研究もある（[Naendrup-Poell & Onnasch 2025](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12457686/)）。予告は軌跡のプレビューや到達点の表示など、擬人化しない形で出すのが無難です。
- 「AIが判断しました」とは書かず、止まった理由と再開の手順を書く。HAXの指針どおり、何ができてどこまで確かか、なぜそうしたかを示し、却下と訂正を簡単にする。

### 5. 置き換え例

| よくあるAIっぽい表現 | 置き換え |
| --- | --- |
| 「Robot Online」の横で脈打つ緑の点 | 「接続中 遅延 12 ms 更新 14:02:31」のように実測値と時刻を出す |
| 紫グラデーションと四芒星の「AI Optimize」ボタン | 「経路を再計算」。実行前に新旧の軌跡と所要時間を並べて見せる |
| 「Thinking…」の光る文字 | 「把持位置を計算中（3/5）」と中止ボタン |
| 「Oops! Something went wrong」と泣き顔 | 「停止：エリアセンサ遮光（ゾーンB）。人の退避を確認、リセット、再開」 |
| ロボットの顔アイコンと吹き出し | 現在の動作、次の目標位置、残り時間 |
| 暗い背景にシアンの発光HUD | 明るめのグレー背景。目立つのは異常の色だけ |
| 数字がカウントアップするKPIカード | 正常範囲の帯が付いたバーと、直近の傾向 |

### 6. 「グレーは退屈」への手当て

- 現場には、グレーの画面は退屈で眠くなるという声もある（[Automation World](https://www.automationworld.com/products/control/blog/13311220/high-performance-hmi-hurdle)）。解決策は装飾ではなく、正常範囲の帯が付いたバーやゲージ、傾向グラフで正常時の情報を読みやすくすること（[ISA-101の解説](https://deepdebug-site.pages.dev/en/blog/isa-101-hmi)）。
- 個性は配色の派手さではなく、書体、用語、レイアウトの一貫性で出す。和文はUD書体など読みやすさを優先し、数値は等幅数字で桁を揃えて単位を付ける。

## 置き換えチェックリスト

レビューでは1画面ずつ確認し、チェックできなかった項目をそのまま修正対象にします。

- [ ] 画面上のすべての色を、凡例にある状態の意味で説明できる
- [ ] 通常運転中の画面に、ロゴ以外の高彩度の色がない
- [ ] 赤と黄の組み合わせを非常停止以外に使っていない
- [ ] ブランド色やアクセント色が、状態色（赤・黄・緑・青）と紛らわしくない
- [ ] グラデーション、発光、ガラス調、色付きの影がない（残すなら理由を一文で言える）
- [ ] 動き（点滅・パルス・アニメーション）がすべて状態変化か操作への応答に対応している
- [ ] 常時アニメーションと数字のカウントアップがなく、モーション削減の設定で動きが止まる
- [ ] ステータスの点やバッジが実データにつながり、最終更新時刻が分かる
- [ ] 絵文字を使っていない。アイコンは1系統で、テキストラベルが付いている
- [ ] 四芒星（スパークル）アイコンを使っていないか、必ずラベルを付けている
- [ ] 角丸は2〜3段階、影は浮かせる要素だけに使っている
- [ ] 運転モード、安全状態、現在と次の動作、停止方法が最大の面積を占める
- [ ] 通信断、非常停止中、保護停止、原点未復帰、データなしの画面がある
- [ ] 数値に単位があり、等幅数字で桁がぶれない
- [ ] 文言に誇張語、決まり文句、感嘆符、一人称の「私」がない
- [ ] エラー文に「何が」「どこで」「次に何をするか」がある
- [ ] 実機・実データで確認し、ダミー値や架空の数値が残っていない
- [ ] 各デザイン判断の理由を一文で答えられる

## AIツールで画面を作るときの指示の出し方

AIに画面を作らせること自体は問題ではありません。決めるべきことを決めずに渡すと、最頻値の画面が返ってきます。先に人が決めて文書にし、生成後は機械と人で検査します。

1. **先に人が決める。** 利用者（例：手袋をした作業者、保全担当）、使用環境（明るい工場、1〜2 m離れて見る）、最重要タスク（停止・再開・モード切替）を決めます。誰に何を伝えるかを決める最初の5%と、最終確認の最後の5%は人が担うべきだという指摘があります（[プレジデント](https://president.jp/articles/-/121884)）。
2. **規則を1ファイルに書き出す。** Google Labsが提唱するDESIGN.mdのように、色・書体・余白・部品の規則をMarkdownで置くと、Claude CodeやCursorなどが毎回参照できます（[Qiita](https://qiita.com/miruky/items/a6312c14e6352376ec00)、[libecity](https://library.libecity.com/articles/01KRKJT5QWKSA8M266AZE7CC8F)）。
3. **色はトークン名でしか使わせない。** 色・余白・角丸・書体を名前付きの変数で定義し、生の値を使わせなければ、bg-indigo-500のような既定値が入り込む余地がなくなります（[devtake](https://devtake.pages.dev/article/ai-generated-frontend-slop)）。Tailwindの既定パレットを外し、使うとビルドが失敗するようにする方法もあります（[Alan West](https://dev.to/alanwest/how-to-fix-the-ai-generated-look-in-your-frontend-1ahh)）。
4. **禁止事項は数値で書く。** 「おしゃれに」ではなく、角丸2〜4px、影はほぼ使わない、グラデーション禁止のように指定します（[libecity](https://library.libecity.com/articles/01KRKHK607NHNQ4DZPYG7JMGHS)）。
5. **検査を回す。** Impeccableは、AIモデルを使わないルールベースの検査で、AI特有の癖、文字切れ、コントラスト不足を検出します。HallmarkやTaste Skillも同じ目的のスキルです（[テクノエッジ](https://www.techno-edge.net/article/2026/11/03/5660.html)）。ただしいずれもWebページ向けなので、HMIとしての妥当性は人がレビューします。
6. **実機で仕上げる。** 実データを流し、通信断や非常停止中など端の状態をすべて表示させてから確定します。

指示ファイルの例です。自社の条件に合わせて数値と色を調整してください。

```markdown
# ロボット操作UI デザイン規則

## 前提
- 利用者: 工場の作業者と保全担当。手袋で操作し、1〜2 m離れて画面を見る
- 最優先: 状態の把握、停止、再開。装飾は不要

## 色
- 背景と通常の要素はグレーの階調だけ
- 色は状態トークンだけを使う
  - state-emergency: 赤（緊急・危険）
  - state-abnormal: 黄（異常）
  - state-normal: 緑（正常を明示する必要がある箇所だけ。通常はグレー＋文字）
  - state-action-required: 青（操作が必要）
- 赤と黄の組み合わせは非常停止の表示以外に使わない
- 禁止: グラデーション、発光、ガラス調、色付きの影、紫・インディゴ、ネオン色

## 動き
- アニメーションは状態の変化と操作への応答だけ
- 禁止: 常時アニメーション、shimmer、数字のカウントアップ、脈打つ点

## 形
- 角丸は 2px と 4px だけ。影はモーダルだけ
- アイコンは線画1系統で、必ずテキストラベルを付ける。絵文字と四芒星アイコンは禁止

## 文字と文言
- 和文はUD書体、数値は等幅数字。数値には必ず単位を付ける
- 一人称、擬人化、感嘆符、誇張語を使わない
- エラーは「何が／どこで／次の操作」の順に書く

## 必ず用意する画面状態
- 通信断、非常停止中、保護停止、原点未復帰、データなし
```

## 出典

本文で参照したページです。

### 公式文書・規格・研究

- [Improving frontend design through Skills](https://claude.com/blog/improving-frontend-design-through-skills)（Anthropic）
- [The Proliferation and Problem of the Sparkles Icon](https://www.nngroup.com/articles/ai-sparkles-icon-problem/)（Nielsen Norman Group、Kate Kaplan、2024年）
- [Handmade Designs: The New Trust Signal](https://www.nngroup.com/articles/handmade-designs/)（Nielsen Norman Group）
- [All That Sparkles Is AI](https://design.google/library/ai-sparkle-icon-research-pozos-schmidt)（Google Design）
- [HAX Design Library: Guidelines for Human-AI Interaction](https://www.microsoft.com/en-us/haxtoolkit/library/)（Microsoft）
- [People + AI Guidebook: Mental Models](https://pair.withgoogle.com/chapter/People%20+%20AI%20Guidebook%20-%20Mental%20Models.pdf)（Google PAIR）
- [Anthropomorphism](https://developers.google.com/style/anthropomorphism)（Google開発者ドキュメント スタイルガイド）
- [Writing for machine-assisted experiences](https://doc.wikimedia.org/codex/latest/style-guide/machine-assistance.html)（Wikimedia Codex）
- [AI UI Text](https://www.sap.com/design-system/fiori-design-ios/v26-4/in-app-ai-design/foundation/ai-ui-text)（SAP Fiori）
- [AI for Product](https://design.morningstar.com/systems/product/ai)（Morningstar Design System）
- [Going Gray: A New HMI Standard](https://control.com/technical-articles/going-gray/)（Control.com、Dennis Lynch、2020年）
- [The High Performance HMI](https://blog.isa.org/the-high-performance-hmi)（ISA Interchange）
- [High Performance HMI Techniques](https://docs.inductiveautomation.com/docs/7.9/visualization-and-dashboards/understanding-components/high-performance-hmi-techniques)（Inductive Automation）
- [ISA-101: High Performance HMI design](https://deepdebug-site.pages.dev/en/blog/isa-101-hmi)（DeepDebug）
- [押しボタンの色と非常停止（IEC 60204-1、ISO 13850）](https://www.se.com/uk/en/faqs/FAQ000283554/)（Schneider Electric FAQ）
- [Industrial Pilot Light Color Coding: IEC, NFPA 79 Standards Reference](https://industrialmonitordirect.com/blogs/knowledgebase/industrial-pilot-light-color-coding-iec-nfpa-79-standards-reference)（Industrial Monitor Direct）
- [EN 60204-1 Indicator Light Color Coding for Machine Status](https://industrialmonitordirect.com/blogs/knowledgebase/en-60204-1-indicator-light-color-coding-for-machine-status)（Industrial Monitor Direct）
- [The Case for More Colorful HMI Graphics](https://www.automationworld.com/control/article/22031298/the-case-for-more-colorful-hmi-graphics)（Automation World、2022年）
- [High-performance HMI hurdle](https://www.automationworld.com/products/control/blog/13311220/high-performance-hmi-hurdle)（Automation World）
- [Director（Team MIT、DARPA Robotics Challenge）](https://ora.ox.ac.uk/objects/uuid:29b467db-4ced-4e0f-ba6b-406bb781ab81)（Oxford Research Archive）、[論文PDF](https://groups.csail.mit.edu/robotics-center/public_papers/Marion16.pdf)（MIT CSAIL）
- [The robot eyes don't have it](https://baes.uc.pt/bitstream/10316/112093/1/The%20robot%20eyes%20don%27t%20have%20it.%20The%20presence%20of%20eyes%20on%20collaborative%20robots%20yields%20marginally%20higher%20user%20trust%20but%20lower%20performance.pdf)（Pilacinskiほか）
- [Anthropomorphic framing and failure comprehensibility influence different facets of trust towards industrial robots](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10512549/)（Roesler、2023年）
- [Predictive robot eyes enhance attentional guidance in cooperative human–robot interaction](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12457686/)（Naendrup-Poell、Onnasch、2025年）

### 記事・批評

- [Show HN submissions tripled and now mostly share the same vibe-coded look](https://www.adriankrebs.ch/blog/design-slop/)（Adrian Krebs）
- [Show HN分析の紹介](https://www.gigazine.net/news/20260423-design-slop/)、[performative-uiの紹介](https://gigazine.net/news/20260609-performative-ui/)（GIGAZINE）
- [7 Signs a UI Has Been Vibe Coded](https://www.thefountaininstitute.com/blog/signs-vibe-coded-ui)（The Fountain Institute）
- [Why Every AI-Built Website Looks the Same](https://dev.to/alanwest/why-every-ai-built-website-looks-the-same-blame-tailwinds-indigo-500-3h2p)、[How to fix the 'AI-generated' look in your frontend](https://dev.to/alanwest/how-to-fix-the-ai-generated-look-in-your-frontend-1ahh)（Alan West、DEV）
- [AI-generated frontend slop](https://devtake.pages.dev/article/ai-generated-frontend-slop)（devtake）
- [Anti-patterns](https://www.mintlify.com/pbakaus/impeccable/concepts/anti-patterns)（Impeccable）
- [There are 3 telltale signs that you used AI to make your app](https://dnyuz.com/2026/07/07/there-are-3-telltale-signs-that-you-used-ai-to-make-your-app-and-they-arent-pretty/)（Business Insiderの転載）
- [Nobody asked for the sparkle button](https://uxdesign.cc/nobody-asked-for-the-sparkle-button-ae54ab5065c6)（UX Collective）
- [AI Makes Bad Design Look Good Enough](https://builtin.com/articles/ai-design-slop-era)（Built In）
- [AIっぽいUIを避けるデザインスキルの紹介](https://www.techno-edge.net/article/2026/11/03/5660.html)（テクノエッジ、2026年11月）
- [AIバレしない「資料・チラシ・ポスター」のつくり方](https://president.jp/articles/-/121884)（プレジデントオンライン、川合卓也、2026年11月）
- [AIデフォルトの見た目の12の兆候](https://www.utsubo.com/ja/blog/built-with-ai-trust-signals-2026)（Utsubo、2026年6月）
- [AIライティングの兆候](https://www.pangram.com/ja/signs-of-ai-writing)（Pangram）
- [AIくささを消す](https://tenbin.ai/media/ai_tips/ai-writing-remove-ai-slop)（天秤AI）

### コミュニティ・個人の声

- [Ask HN: How can you tell a site/app is vibe coded?](https://news.ycombinator.com/item?id=49290499)（Hacker News）
- [The AI Purple Problem: Why Every AI Brand Looks the Same](https://www.indiehackers.com/post/the-ai-purple-problem-why-every-ai-brand-looks-the-same-6cb0aa2a02)（Indie Hackers）
- [Claude Codeで作るサイトのAI感を消す5つのコツ](https://library.libecity.com/articles/01KRKHK607NHNQ4DZPYG7JMGHS)、[DESIGN.mdでデザインルールを教える](https://library.libecity.com/articles/01KRKJT5QWKSA8M266AZE7CC8F)（libecity、みずの）
- [DESIGN.mdの書き方](https://qiita.com/miruky/items/a6312c14e6352376ec00)（Qiita）
- [「AIが作ったっぽい」という印象は、どこから来るのか](https://braindetox.kr/ja/posts/ai_design_tell_detection.html)（braindetox）
