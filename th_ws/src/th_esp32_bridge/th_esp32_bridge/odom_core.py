"""
odom_core.py — esp32_bridge のオドメトリ計算 (ROS2 非依存)
========================================================================
scripts/esp32_bridge.py の _on_wheel_feedback() から、ROS2 の時刻型や
nav_msgs/Odometry に依存しない数値部分だけを抽出したもの
(docs/plan/detailed/DetailedDesign-reuse.md §2.6 「①オドメトリ積分・スタンプ
再同期を odom_core.py へ抽出」)。ノード側はここの関数を呼び、戻り値を ROS2
メッセージへ詰め替えるだけにする。

抽出した範囲:
  - resolve_dt()       : WHEEL_FEEDBACK の dt 検証とフォールバック
  - resync_stamp()     : ヘッダスタンプの dt 累積と、実時間乖離時の貼り直し判定
  - integrate_pose()   : 差動駆動オドメトリ積分 (x, y, yaw の更新)
  - yaw_to_quaternion_zw() : integrate_pose() の出力 (yaw) から
                             pose.orientation の z/w 成分を作る付随計算
                             (roll=pitch=0 前提の単純な三角関数なのでここに含める)

抽出しなかった範囲: TF ブロードキャスト・Odometry / WheelFeedback メッセージの
構築・共分散の設定・パラメータ変更コールバックは ROS2 型そのものが対象なので
ノード側 (scripts/esp32_bridge.py) に残した。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def resolve_dt(dt_from_esp32: "float | None", nominal_dt: float,
                dt_max: float) -> tuple[float, bool]:
    """WHEEL_FEEDBACK に載ってきた dt を検証する。

    ESP32 は velL = counts * distPerCount / dt で速度を出しており、その
    フレームが表す走行時間そのものが dt。(0, dt_max] の範囲内ならそのまま
    積分区間として使う。範囲外の値・欠落 (dt なしの旧形式フレーム) は
    nominal_dt (公称周期) へフォールバックする。

    戻り値: (実際に使う dt, ESP32 が送ってきた値をそのまま使えたか)
    使えなかった場合の判別 (フィールド自体が無かったのか、値が不正だったのか)
    は呼び出し側が `dt_from_esp32 is None` で行う (ログの出し分けに必要なため
    ここでは畳み込まない)。
    """
    if dt_from_esp32 is not None and 0.0 < dt_from_esp32 <= dt_max:
        return dt_from_esp32, True
    return nominal_dt, False


@dataclass(frozen=True)
class StampResync:
    stamp_sec: float
    resynced: bool


def resync_stamp(prev_stamp_sec: "float | None", now_sec: float, dt: float,
                  resync_threshold_sec: float) -> StampResync:
    """オドメトリのヘッダスタンプを dt で進め、実時間から乖離しすぎたら貼り直す。

    robot_localization は /odom の pose ではなく twist とヘッダスタンプの
    差分で積分するため (ekf_params.yaml の odom0_config は vx/vyaw のみ
    true)、到着時刻をそのまま貼ると遅延後にまとめて届いたフレーム群が
    ほぼ同一スタンプを持ち、積分区間がゼロになってしまう。dt ぶんずつ
    進む単調なスタンプを使い、実時間から離れすぎた場合だけ現在時刻へ
    貼り直す (異常時の安全網。正常時はスタンプは実時間の近傍に留まる)。

    prev_stamp_sec が None (初回呼び出し) なら now_sec を初期値にする。
    """
    if prev_stamp_sec is None:
        return StampResync(now_sec, False)
    advanced = prev_stamp_sec + dt
    skew = abs(now_sec - advanced)
    if skew > resync_threshold_sec:
        return StampResync(now_sec, True)
    return StampResync(advanced, False)


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def integrate_pose(pose: Pose2D, v_left: float, v_right: float,
                    wheel_base: float, dt: float) -> tuple[Pose2D, float, float]:
    """差動駆動オドメトリを dt ぶん積分する。

    戻り値: (更新後の Pose2D, v_center, omega)
    """
    v_center = (v_left + v_right) / 2.0
    omega = (v_right - v_left) / wheel_base
    dtheta = omega * dt

    new_x = pose.x + v_center * math.cos(pose.yaw + dtheta / 2.0) * dt
    new_y = pose.y + v_center * math.sin(pose.yaw + dtheta / 2.0) * dt
    new_yaw = pose.yaw + dtheta

    return Pose2D(new_x, new_y, new_yaw), v_center, omega


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    """yaw のみの姿勢 (roll=pitch=0 前提) を表す quaternion の z, w 成分。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
