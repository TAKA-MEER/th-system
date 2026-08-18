"""
conftest.py — pytest パス設定
================================
colcon build 前でも `pytest test/` で直接実行できるよう
th_planning / th_esp32_bridge / th_perception / th_state の Python パッケージパスを
sys.path に追加する。

`_repo_root` はリポジトリルートではなく `th_ws/src` を指す。これは意図的な用法である
（th_planning 等のパッケージパスを sys.path に足すため）。書き換えないこと
（WP-STATE-01 着手時点で 45 件の既存テストがこの前提で通っている）。

正本 `Spec-modes.md` の場所解決は別の仕組み（`resolve_spec_dir()` 以下）で行う。
"""
import sys
import os

_repo_root  = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# th_planning パッケージのルートを追加
_planning     = os.path.join(_repo_root, 'th_planning')
_planning_core = os.path.join(_repo_root, 'th_planning', 'th_planning')

# th_esp32_bridge パッケージのルートを追加
_esp32_bridge      = os.path.join(_repo_root, 'th_esp32_bridge')
_esp32_bridge_core = os.path.join(_repo_root, 'th_esp32_bridge', 'th_esp32_bridge')

# th_perception パッケージの scripts (person_tracker_bridge_core.py 等) を追加
_perception_scripts = os.path.join(_repo_root, 'th_perception', 'scripts')

# th_state パッケージのルートを追加（WP-STATE-01。colcon build --symlink-install 前でも
# pytest test/test_state_core.py を単体で実行できるようにする）
_th_state = os.path.join(_repo_root, 'th_state')

for _p in [_planning, _planning_core, _esp32_bridge, _esp32_bridge_core,
           _perception_scripts, _th_state]:
    if _p not in sys.path:
        sys.path.insert(0, _p)


# ============================================================
# 正本 Spec-modes.md の場所解決（WP-STATE-01 §7）
# ============================================================
#
# `_repo_root`（= th_ws/src）を書き換えると既存 45 テストが壊れるため、
# Spec-modes.md の解決は独立した関数で行う。解決順序:
#   ① 環境変数 TH_REPO_ROOT があればその配下の docs/plan/spec
#   ② /docs/plan/spec が存在すればそれ（コンテナ内。docker-compose.yml が
#      ../docs:/docs:ro でマウントする。リポジトリルート自体はコンテナ内に無い）
#   ③ __file__ から上に辿って docs/plan/spec を探す（ホスト側。リポジトリルート配下に
#      docs/plan/spec/Spec-modes.md がある）
# どれでも見つからなければ明示的に fail する（黙ってスキップしない。§11-2 が
# 実行不能になるのを防ぐ）。
import pathlib


class SpecModesNotFoundError(RuntimeError):
    pass


def resolve_spec_dir() -> pathlib.Path:
    # ① 環境変数 TH_REPO_ROOT
    env_root = os.environ.get('TH_REPO_ROOT')
    if env_root:
        candidate = pathlib.Path(env_root) / 'docs' / 'plan' / 'spec'
        if (candidate / 'Spec-modes.md').is_file():
            return candidate
        raise SpecModesNotFoundError(
            f"TH_REPO_ROOT={env_root} が指定されているが "
            f"{candidate / 'Spec-modes.md'} が見つからない")

    # ② コンテナ内マウント /docs/plan/spec
    container_candidate = pathlib.Path('/docs/plan/spec')
    if (container_candidate / 'Spec-modes.md').is_file():
        return container_candidate

    # ③ __file__ から上に辿る（ホスト側）
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / 'docs' / 'plan' / 'spec'
        if (candidate / 'Spec-modes.md').is_file():
            return candidate

    raise SpecModesNotFoundError(
        "Spec-modes.md が見つからない。TH_REPO_ROOT を設定するか、"
        "コンテナ内で /docs (docker-compose.yml の ../docs:/docs:ro) が"
        "マウントされていることを確認してください。")


def spec_modes_path() -> pathlib.Path:
    return resolve_spec_dir() / 'Spec-modes.md'


import pytest


def pytest_configure(config):
    config.addinivalue_line(
        'markers',
        'rule(id): この遷移表 (transitions.yaml) の行 id をこのテストが担保することを宣言する'
        '（WP-STATE-01 §11-3 / DetailedDesign-state.md §11 要件3）')


@pytest.fixture(scope='session')
def spec_modes_file() -> pathlib.Path:
    """正本 Spec-modes.md への Path。見つからなければ即 fail する。"""
    try:
        return spec_modes_path()
    except SpecModesNotFoundError as e:
        pytest.fail(str(e))


# ============================================================
# th_state の config 読み込み（WP-STATE-01）
# ============================================================
def th_state_config_dir() -> pathlib.Path:
    """th_state/config への Path。`import th_state` の場所から相対に探す
    （--symlink-install なら site-packages のシンボリックリンク先がソースツリーを指すため、
    colcon build 後・pytest 直接実行のどちらでも解決できる）。"""
    import th_state
    pkg_dir = pathlib.Path(th_state.__file__).resolve().parent
    config_dir = pkg_dir.parent / 'config'
    if not (config_dir / 'transitions.yaml').is_file():
        raise FileNotFoundError(f"th_state/config/transitions.yaml が見つからない: {config_dir}")
    return config_dir


@pytest.fixture(scope='session')
def state_core_bundle():
    """(StateCore, transitions, mode_entry, attributes) を返す。"""
    import yaml
    from th_state import guards as guards_module
    from th_state.state_core import StateCore

    config_dir = th_state_config_dir()
    with open(config_dir / 'transitions.yaml', encoding='utf-8') as f:
        transitions = yaml.safe_load(f)
    with open(config_dir / 'attributes.yaml', encoding='utf-8') as f:
        attributes = yaml.safe_load(f)
    with open(config_dir / 'mode_entry.yaml', encoding='utf-8') as f:
        mode_entry = yaml.safe_load(f)

    guards = guards_module.build_guards(mode_entry)
    core = StateCore(transitions, mode_entry, attributes, guards)
    return core, transitions, mode_entry, attributes
