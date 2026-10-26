# AGENTS.md

作業を始める前に、必ずリポジトリ直下の [CLAUDE.md](CLAUDE.md) を全文読み、その指示に従うこと。
このリポジトリの決まりごと（作業前の `git status` 確認、spec が正本であること、環境の癖など）はすべて CLAUDE.md に書いてある。
CLAUDE.md は Claude 向けの名前だが、codex を含むすべての実装エージェントに適用する。

実装エージェントとして呼ばれた場合は、[docs/plan/ImplementationPlan.md](docs/plan/ImplementationPlan.md) §2 も読むこと。
特に次の点を守る。

- 渡されたブリーフ（`.briefs/brief-<ID>.md`）を全文読み、前提に誤りがあれば実装前に指摘する。
- 自分の作業ブランチにコミットするところまでが担当。`main` へのマージ・`git push` はしない。
- コミットは細かく切る（利用枠の上限で途中で止まる前提）。
- 一時ファイルは `.briefs/tmp/` に置き、`/tmp` を使わない。
