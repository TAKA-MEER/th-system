"""validate_cli.py — 起動可能であることを検査する CLI（WP-STATE-01 完了条件④）。

パケット §1 の「作る」表には明記されていないが、完了条件④が
`python3 -m th_state.validate_cli --config <dir>` の実行を要求しているため作成する。

  $ python3 -m th_state.validate_cli --config src/th_state/config

`transitions.yaml` / `attributes.yaml` / `mode_entry.yaml` を読み込み、
`StateCore.validate()` を呼ぶ。エラーが無ければ**何も出力せず** exit code 0。
エラーがあれば 1 行 1 件で標準エラー出力へ書き、exit code 1。
"""
import argparse
import os
import sys

import yaml

from th_state import guards as guards_module
from th_state.state_core import StateCore


def _load_config(config_dir: str):
    with open(os.path.join(config_dir, "transitions.yaml"), encoding="utf-8") as f:
        transitions = yaml.safe_load(f)
    with open(os.path.join(config_dir, "attributes.yaml"), encoding="utf-8") as f:
        attributes = yaml.safe_load(f)
    with open(os.path.join(config_dir, "mode_entry.yaml"), encoding="utf-8") as f:
        mode_entry = yaml.safe_load(f)
    return transitions, attributes, mode_entry


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="th_state の設定 (transitions/attributes/mode_entry) を検査する")
    parser.add_argument("--config", required=True,
                         help="config ディレクトリ（transitions.yaml 等を含む）")
    args = parser.parse_args(argv)

    transitions, attributes, mode_entry = _load_config(args.config)
    guards = guards_module.build_guards(mode_entry)
    core = StateCore(transitions, mode_entry, attributes, guards)

    errors = core.validate()
    if errors:
        for e in errors:
            print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
