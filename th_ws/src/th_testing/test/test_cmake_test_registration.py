"""
test_cmake_test_registration.py
==================================
CMakeLists.txt へのテスト登録漏れを検出するガードテスト。
ROS2 なし・純粋 Python（ファイルの文字列走査のみ）で実行可能。subprocess も使わない。

背景: test_esp32_ws_protocol.py が th_testing/CMakeLists.txt に
ament_add_pytest_test で登録されておらず、colcon test で一度も実行されて
いなかった。そのため ws_protocol.py に unpack_estop_hw_flags が存在せず
collection error になっていたことに誰も気づけず、esp32_bridge ノードが
ImportError で起動不能な状態が放置された（ed67069 で修正）。このガードは
同種の事故（テストを書いたのに CMakeLists.txt への登録を忘れ、
colcon test では静かにスキップされ続ける）の再発を防ぐ。

検査内容（いずれもファイル一覧と CMakeLists.txt の記述内容の集合比較。
個数のハードコードはしない）:
  1. th_testing/test/ 配下の全 test_*.py が、いずれかの CMakeLists.txt に
     ament_add_pytest_test で登録されていること
  2. リポジトリ内(th_ws/src 配下)の全 test_*.cpp が、いずれかの
     CMakeLists.txt に ament_add_gtest で登録されていること
  3. 上記1・2の逆: CMakeLists.txt が ament_add_pytest_test / ament_add_gtest
     で参照しているファイルが実在すること（削除・リネームされたテストへの
     参照が残っていないか）
"""
import glob
import os
import re

# _SRC_ROOT は th_ws/src を指す（conftest.py の _repo_root と同じ考え方。
# docker-compose.yml がホストの `th_ws/src` だけをコンテナへバインドマウント
# しており、コンテナ内にリポジトリ全体は存在しないため、パッケージ群の
# 共通の親である th_ws/src を基準にする）。
_SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# ament_add_pytest_test(<登録名> <相対パス> ...) / ament_add_gtest(<登録名> <相対パス> ...)
# 単一行・複数行のどちらの書式にもマッチする（\s は改行も含む）。
_PYTEST_CALL_RE = re.compile(r'ament_add_pytest_test\s*\(\s*(\S+)\s+([^\s)]+)')
_GTEST_CALL_RE = re.compile(r'ament_add_gtest\s*\(\s*(\S+)\s+([^\s)]+)')


def _all_cmakelists():
    """th_ws/src 直下の各パッケージの CMakeLists.txt の絶対パス一覧。"""
    return sorted(glob.glob(os.path.join(_SRC_ROOT, '*', 'CMakeLists.txt')))


def _registered_entries(pattern):
    """(登録名, CMakeLists.txt絶対パス, 参照先ファイルの絶対パス) のリスト。"""
    entries = []
    for cmake_path in _all_cmakelists():
        with open(cmake_path, encoding='utf-8') as f:
            content = f.read()
        cmake_dir = os.path.dirname(cmake_path)
        for m in pattern.finditer(content):
            name, rel_path = m.group(1), m.group(2)
            abs_path = os.path.normpath(os.path.join(cmake_dir, rel_path))
            entries.append((name, cmake_path, abs_path))
    return entries


def _rel(path):
    return os.path.relpath(path, _SRC_ROOT)


class TestPythonTestFilesAreRegistered:
    """th_testing/test/ の test_*.py が colcon test の対象になっているか。"""

    def test_all_test_py_files_registered_via_ament_add_pytest_test(self):
        test_dir = os.path.join(_SRC_ROOT, 'th_testing', 'test')
        actual = {
            os.path.normpath(p)
            for p in glob.glob(os.path.join(test_dir, 'test_*.py'))
        }
        registered = {
            abs_path for (_name, _cmake, abs_path) in _registered_entries(_PYTEST_CALL_RE)
        }
        missing = sorted(_rel(p) for p in actual - registered)
        assert not missing, (
            "以下の test_*.py が CMakeLists.txt に ament_add_pytest_test で"
            "登録されていません（colcon test で一度も実行されません）: "
            + ", ".join(missing)
        )


class TestCppTestFilesAreRegistered:
    """リポジトリ内の全 test_*.cpp が ament_add_gtest で登録されているか。"""

    def test_all_test_cpp_files_registered_via_ament_add_gtest(self):
        actual = {
            os.path.normpath(p)
            for p in glob.glob(os.path.join(_SRC_ROOT, '**', 'test_*.cpp'), recursive=True)
        }
        registered = {
            abs_path for (_name, _cmake, abs_path) in _registered_entries(_GTEST_CALL_RE)
        }
        missing = sorted(_rel(p) for p in actual - registered)
        assert not missing, (
            "以下の test_*.cpp がいずれの CMakeLists.txt にも ament_add_gtest で"
            "登録されていません（colcon test で一度も実行されません）: "
            + ", ".join(missing)
        )


class TestCMakeListsReferencesAreValid:
    """CMakeLists.txt が参照しているテストファイルが実在するか（登録の逆方向）。"""

    def test_ament_add_pytest_test_targets_exist(self):
        missing = sorted(
            f"{_rel(cmake_path)}: {name} -> {_rel(abs_path)} が存在しません"
            for (name, cmake_path, abs_path) in _registered_entries(_PYTEST_CALL_RE)
            if not os.path.isfile(abs_path)
        )
        assert not missing, (
            "CMakeLists.txt が ament_add_pytest_test で参照しているが実在しない "
            "ファイルがあります: " + ", ".join(missing)
        )

    def test_ament_add_gtest_targets_exist(self):
        missing = sorted(
            f"{_rel(cmake_path)}: {name} -> {_rel(abs_path)} が存在しません"
            for (name, cmake_path, abs_path) in _registered_entries(_GTEST_CALL_RE)
            if not os.path.isfile(abs_path)
        )
        assert not missing, (
            "CMakeLists.txt が ament_add_gtest で参照しているが実在しない "
            "ファイルがあります: " + ", ".join(missing)
        )
