# `fault_injection` をパッケージにするための空ファイル。
#
# 目的は「テストコードの再利用」ではなく、この配下の `conftest.py` を
# `fault_injection.conftest` という一意な名前で import させ、親の
# `test/conftest.py`（`__init__.py` が無いため import 名は裸の `conftest`）との
# `sys.modules['conftest']` 衝突を避けること。詳細はこのディレクトリの
# `conftest.py` 冒頭の docstring「なぜ fault_injection/__init__.py を置いているか」
# を参照。
