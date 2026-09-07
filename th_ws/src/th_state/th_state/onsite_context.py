"""onsite_context.py — state_manager が Context に詰める onsite 由来派生値の純関数。

WP-ONSITE-F2。`/person/targets` / `/onsite/pins` のメッセージ値を
StateCore.Context の candidate_count / target_selected / target_confident /
pin_kinds にマッピングする。ROS2 非依存（pytest で直接テスト可能）。
"""


def derive_person_ctx(n_candidates, selected_index, confidence, conf_min):
    """(candidate_count, target_selected, target_confident) を返す。

    - target_selected: 選択中 index が -1 でなければ選択済み。
    - target_confident: 信頼度が十分かだけを表す（guard 側で
      `target_selected and target_confident` に合成される）。
    """
    return (int(n_candidates),
            int(selected_index) >= 0,
            float(confidence) >= float(conf_min))


def derive_pin_kinds(kinds):
    """Pin.kind のリストから、空文字を除いて重複排除・ソートした tuple を返す。"""
    return tuple(sorted({k for k in kinds if k}))