"""test_installed_scripts_executable.py — install(PROGRAMS ...) に載せたスクリプトが
実際に起動できる形になっていることを固定する。

実機で踏んだ事故（2026-09-03）:
    [ERROR] [launch]: executable 'map_downsampler.py' not found on the libexec
    directory '/root/th_ws/install/th_planning/lib/th_planning'

`map_downsampler.py` は CMakeLists の `install(PROGRAMS)` に正しく載っていて
`colcon build` も成功していたのに、**ファイルの実行ビットが立っていなかった**
（git mode 100644。他のスクリプトは全部 100755）。

`--symlink-install` では install 先が**ソースファイルへのシンボリックリンク**に
なるため、CMake の `install(PROGRAMS)` が付けるはずの実行権限は効かず、
ソース側の権限がそのまま runtime に出る。ros2 launch は実行可能でないファイルを
「見つからない」と報告するので、原因が権限だと分からない。

ビルドは通り、テストも通り、実機で launch した瞬間だけ落ちるため、
このテストが無いと同じ事故を繰り返す。
"""
import os
import re
import subprocess

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
SRC = os.path.join(REPO, "th_ws", "src")

_BLOCK = re.compile(r"install\(PROGRAMS(.*?)DESTINATION", re.S)


def _installed_programs():
    """全パッケージの install(PROGRAMS ...) に列挙されたファイルを返す。

    戻り値: [(パッケージ名, CMakeLists に書かれた相対パス, 絶対パス), ...]
    """
    rows = []
    for pkg in sorted(os.listdir(SRC)):
        cmake = os.path.join(SRC, pkg, "CMakeLists.txt")
        if not os.path.isfile(cmake):
            continue
        with open(cmake, encoding="utf-8") as f:
            text = f.read()
        for block in _BLOCK.finditer(text):
            for line in block.group(1).splitlines():
                rel = line.strip()
                if not rel or rel.startswith("#"):
                    continue
                rows.append((pkg, rel, os.path.join(SRC, pkg, rel)))
    return rows


def test_install_programs_blocks_are_found():
    """走査そのものが壊れていないことの保険（0 件なら以降のテストが無力になる）。"""
    rows = _installed_programs()
    assert len(rows) >= 5, f"install(PROGRAMS ...) の走査結果が少なすぎる: {rows}"
    names = {rel for _pkg, rel, _p in rows}
    assert any(n.endswith("map_downsampler.py") for n in names), (
        "th_planning の map_downsampler.py が install(PROGRAMS) に載っていない")


def test_installed_programs_exist():
    missing = [f"{pkg}: {rel}" for pkg, rel, path in _installed_programs()
               if not os.path.isfile(path)]
    assert not missing, (
        "install(PROGRAMS ...) に載っているのに実体が無い（ビルドが失敗する）:\n  "
        + "\n  ".join(missing))


def test_installed_programs_are_executable():
    """実行ビットが立っていること。

    立っていないと `ros2 launch` が
    「executable '<name>' not found on the libexec directory」で落ちる。
    エラー文言が「見つからない」なので権限だと気づけない。
    """
    bad = [f"{pkg}: {rel}" for pkg, rel, path in _installed_programs()
           if os.path.isfile(path) and not os.access(path, os.X_OK)]
    assert not bad, (
        "実行ビットが立っていない（chmod +x が要る。--symlink-install では "
        "install 先がソースへのシンボリックリンクになり、ソース側の権限がそのまま "
        "runtime に出る）:\n  " + "\n  ".join(bad))


def test_installed_programs_are_executable_in_git():
    """git の index 上でも 100755 であること。

    ローカルで chmod しただけでは他の clone に伝わらず、
    別の PC でチェックアウトした瞬間に同じ事故が再発する。
    """
    rows = _installed_programs()
    paths = [path for _pkg, _rel, path in rows if os.path.isfile(path)]
    if not paths:
        return
    out = subprocess.run(["git", "ls-files", "-s", "--", *paths],
                         cwd=REPO, capture_output=True, text=True)
    if out.returncode != 0:
        return   # git が使えない環境ではスキップ相当（os.access 側で担保される）
    bad = []
    for line in out.stdout.splitlines():
        parts = line.split(maxsplit=3)
        if len(parts) == 4 and parts[0] != "100755":
            bad.append(f"{parts[3]} (mode {parts[0]})")
    assert not bad, (
        "git 上で実行ビットが立っていない（`git update-index --chmod=+x <path>` か "
        "chmod してからコミットする）:\n  " + "\n  ".join(bad))


def test_installed_programs_have_shebang():
    """先頭に shebang があること。実行ビットがあっても shebang が無いと起動できない。"""
    bad = []
    for pkg, rel, path in _installed_programs():
        if not os.path.isfile(path):
            continue
        with open(path, "rb") as f:
            head = f.read(2)
        if head != b"#!":
            bad.append(f"{pkg}: {rel}")
    assert not bad, "shebang (#!) が無い:\n  " + "\n  ".join(bad)
