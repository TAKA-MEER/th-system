"""test_relay_gate.py — relay_gate.FirstComeGate の純粋コア試験（ROS・websockets 不要）。

Spec-safety.md §6.4「先着 1 台」「無通信で手放す」を、時刻を注入して縛る。
所有は接続ごとのトークンで持つ（古い接続の後片付けが新しい所有を消さない）。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "scripts"))

from relay_gate import FirstComeGate  # noqa: E402


def test_first_come_wins_and_second_is_refused():
    g = FirstComeGate()
    t1, t2 = object(), object()
    assert g.try_acquire(t1, "192.168.5.50", now_s=100.0) is True
    assert g.try_acquire(t2, "192.168.5.51", now_s=100.1) is False
    assert g.owner_addr == "192.168.5.50"


def test_same_pc_reconnect_waits_until_released():
    """同じ PC の再接続も、前の接続が手放されるまで待つ（spec §6.4）。"""
    g = FirstComeGate()
    t1, t2 = object(), object()
    assert g.try_acquire(t1, "192.168.5.50", now_s=0.0) is True
    assert g.try_acquire(t2, "192.168.5.50", now_s=0.5) is False
    assert g.release(t1) is True
    assert g.try_acquire(t2, "192.168.5.50", now_s=0.6) is True


def test_stale_owner_release_does_not_evict_new_owner():
    """無通信で手放す → 新しい PC が取る → 古い接続の後片付けが release しても
    新しい所有が消えず、3 台目が断られる（レビュー指摘 A）。"""
    g = FirstComeGate()
    t1, t2, t3 = object(), object(), object()
    assert g.try_acquire(t1, "192.168.5.50", now_s=0.0) is True
    g.note_frame(t1, now_s=1.0)
    assert g.check_release_if_idle(now_s=2.6, timeout_s=1.5) is True  # 無通信で手放す
    assert g.try_acquire(t2, "192.168.5.51", now_s=2.7) is True       # 2 台目が取る
    assert g.release(t1) is False                                      # 古い接続の後片付け
    assert g.is_owner(t2) is True
    assert g.owner_addr == "192.168.5.51"
    assert g.try_acquire(t3, "192.168.5.52", now_s=2.8) is False       # 3 台目は断られる


def test_non_owner_frames_do_not_refresh():
    """所有権を失った接続のフレームで所有が更新されない（C のゲート側）。"""
    g = FirstComeGate()
    t1, t2 = object(), object()
    assert g.try_acquire(t1, "192.168.5.50", now_s=0.0) is True
    g.check_release_if_idle(now_s=2.0, timeout_s=1.5)
    assert g.try_acquire(t2, "192.168.5.51", now_s=2.1) is True
    g.note_frame(t1, now_s=3.0)  # 古い接続からのフレーム
    # t2 の last_frame は 2.1 のままなので 1.5 s 後には空く
    assert g.check_release_if_idle(now_s=3.7, timeout_s=1.5) is True


def test_idle_release_after_timeout():
    g = FirstComeGate()
    t = object()
    assert g.try_acquire(t, "192.168.5.50", now_s=0.0) is True
    g.note_frame(t, now_s=1.0)
    # 1.5 秒ちょうどはまだ手放さない（> で判定）
    assert g.check_release_if_idle(now_s=2.5, timeout_s=1.5) is False
    assert g.check_release_if_idle(now_s=2.51, timeout_s=1.5) is True
    assert g.is_occupied() is False
    # 空いたら次の PC が取れる
    assert g.try_acquire(object(), "192.168.5.51", now_s=2.52) is True


def test_silent_client_also_released():
    """接続して何も送らないクライアントも timeout で空く。"""
    g = FirstComeGate()
    assert g.try_acquire(object(), "192.168.5.50", now_s=10.0) is True
    assert g.check_release_if_idle(now_s=11.6, timeout_s=1.5) is True


def test_frames_keep_the_slot():
    """20Hz で送り続ける間は手放されない。"""
    g = FirstComeGate()
    t = object()
    assert g.try_acquire(t, "192.168.5.50", now_s=0.0) is True
    now = 0.0
    for _ in range(100):
        now += 0.05
        g.note_frame(t, now_s=now)
        assert g.check_release_if_idle(now_s=now, timeout_s=1.5) is False
    assert g.owner_addr == "192.168.5.50"
