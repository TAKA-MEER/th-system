#!/usr/bin/env python3
# ============================================================
# pin_registrar — 試験場内の待機場所/配電盤ピン登録 (WP-ONSITE-01)
# ============================================================
# /system/effect の begin_two_point / place_pin / reject_register を受け、
# /onsite/two_point (TwoPointPress) の 2 点押下で試験員の map 姿勢から
# ゴール①(P1) と向き(yaw) を決め、evt.register_ok / evt.two_point_done /
# evt.register_rejected を /system/event へ publish する。
# place_pin で venue/pins.yaml へ永続化し /onsite/pins (PinList, transient_local) を
# 再 publish。起動時に pins.yaml を読み込み直す（再起動後もピンが残る）。
# /onsite/edit_pin で改名・削除、/onsite/register_pin (TWO_POINT) で直接登録。
#
# 純コア (two_point_core) に方位・距離・対象妥当性・base_link→map 合成を寄せる。
# 登録の対象妥当性は走行時の 500ms 猶予を効かせない（Spec-onsite §3.5）。
import json
import math
import os
import uuid

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

import tf2_ros

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from th_system_msgs.msg import Pin, PinList, PersonStatus, StateEffect, StateEvent
from th_system_msgs.srv import EditPin, RegisterPin, TwoPointPress

from th_onsite.robot_pose_register_core import build_pin_from_robot_pose
from th_onsite.two_point_core import (
    TwoPointParams, compose_map_pose, is_target_valid, spacing_ok, two_point_yaw,
)


def _yaw_from_quat(q) -> float:
    """クォータニオン (w,x,y,z フィールドを持つ) から yaw [rad] を返す。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _load_pins(path) -> list:
    if not os.path.exists(path):
        return []
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data.get('pins', []) or []
    except Exception:
        return []


def _dump_pins(path, pins):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump({'pins': pins}, f, allow_unicode=True,
                       default_flow_style=False)
    os.replace(tmp, path)


class PinRegistrar(Node):
    def __init__(self):
        super().__init__('pin_registrar')

        # ── パラメータ ──────────────────────────────────────
        # WAIVER(demo): W-13 — 数値は registry.yaml 経由でなくノード内リテラル既定値
        self.declare_parameter('venue_dir', '/root/th_data/venue')
        self.declare_parameter('two_point_min_spacing_m', 0.30)
        self.declare_parameter('min_confidence', 0.50)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        venue_dir = self.get_parameter('venue_dir').value
        self._pins_path = os.path.join(venue_dir, 'pins.yaml')
        os.makedirs(venue_dir, exist_ok=True)
        self._params = TwoPointParams(
            min_spacing_m=float(self.get_parameter('two_point_min_spacing_m').value),
            min_confidence=float(self.get_parameter('min_confidence').value))
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # ── 状態 ────────────────────────────────────────────
        self._accepting = False          # begin_two_point 受け取り後の受付中状態
        self._kind = ""                  # "HOME" / "PANEL"
        self._p1 = None                  # (map_x, map_y)
        self._yaw = None
        # /onsite/two_point の押下は service コールバック上で受ける（再入）。
        # 直近の対象 (/person/status) を保持して押下時点で変換する。
        self._person = None              # 直近 PersonStatus
        self._pins = _load_pins(self._pins_path)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── QoS ─────────────────────────────────────────────
        effect_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                                history=QoSHistoryPolicy.KEEP_LAST)
        event_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                               history=QoSHistoryPolicy.KEEP_LAST)
        pins_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST)

        sub_cbg = ReentrantCallbackGroup()

        # ── Subscribers ─────────────────────────────────────
        self.create_subscription(StateEffect, '/system/effect', self._on_effect,
                                 effect_qos, callback_group=sub_cbg)
        # /person/status は 10Hz の処理済みストリーム。publisher（person_tracker_bridge /
        # stub）は既定 QoS（RELIABLE / VOLATILE, depth 10）なので合わせる。
        # TRANSIENT_LOCAL で購読すると VOLATILE publisher と非互換になり 1 通も来ない。
        self.create_subscription(PersonStatus, '/person/status', self._on_person,
                                 10, callback_group=sub_cbg)

        # ── Publishers ──────────────────────────────────────
        self._pub_event = self.create_publisher(
            StateEvent, '/system/event', event_qos)
        self._pub_pins = self.create_publisher(PinList, '/onsite/pins', pins_qos)
        # WP-ONSITE-02: SUMMON のゴール（2点指示の P1=yaw）を venue_navigator へ。
        # transient_local で latched し、mode が SUMMON に入った時に読めるようにする。
        self._pub_summon_goal = self.create_publisher(
            PoseStamped, '/onsite/summon_goal', pins_qos)

        # ── Services ────────────────────────────────────────
        self.create_service(TwoPointPress, '/onsite/two_point', self._on_two_point,
                            callback_group=sub_cbg)
        self.create_service(RegisterPin, '/onsite/register_pin', self._on_register_pin,
                            callback_group=sub_cbg)
        self.create_service(EditPin, '/onsite/edit_pin', self._on_edit_pin,
                            callback_group=sub_cbg)
        # WS-9Y: 前日の地図・ピンをまとめて破棄する（S-20「新しい試験日として
        # 開始」。/slam_control/discard_map とセットで呼ばれる想定。単発の
        # 編集・削除（/onsite/edit_pin）と違い、全件を一括で消す）。
        self.create_service(Trigger, '/onsite/reset_pins', self._on_reset_pins,
                            callback_group=sub_cbg)

        # 起動時に既存ピンを publish（再起動後も残る）
        self._publish_pins()
        self.get_logger().info(
            f'pin_registrar 起動 (pins_path={self._pins_path}, '
            f'pins={len(self._pins)} 件)')

    # ── 座標変換 ──────────────────────────────────────────
    def _map_pose(self):
        """map→base_link TF から (x, y, yaw) を返す。取れなければ None。

        非ブロッキング。バッファに既にあるものだけ読む（route_recorder と同じ流儀）。
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y, _yaw_from_quat(tf.transform.rotation))

    def _target_map_pose(self):
        """直近の対象(base_link 相対)を、押下時点の TF で map 座標に変換する。

        2 点指示の各押下時点で個別に変換する（ロボットが動いていても各点が正しい
        map 座標になる）。対象が無い・剛体の座標合成に失敗 → None。
        """
        if self._person is None:
            return None
        mp = self._map_pose()
        if mp is None:
            return None
        robot_x, robot_y, robot_yaw = mp
        mx, my = compose_map_pose(
            robot_x, robot_y, robot_yaw,
            float(self._person.position.x), float(self._person.position.y))
        return (mx, my)

    # ── 対象妥当性 ────────────────────────────────────────
    def _target_valid(self) -> tuple:
        """対象の妥当性判定。lost / 信頼度不足で (False, reason_key)。

        登録では走行時の 500ms 猶予を効かせない（Spec-onsite §3.5）。
        """
        if self._person is None:
            return False, "no_target"
        ok, _msg = is_target_valid(self._person.is_lost,
                                   float(self._person.confidence), self._params)
        if not ok:
            return False, "target_lost" if self._person.is_lost else "low_confidence"
        return True, "ok"

    # ── /system/event publish ─────────────────────────────
    def _emit_event(self, event: str, arg_json: str = "{}"):
        ev = StateEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.event = event
        ev.source_node = 'pin_registrar'
        ev.arg_json = arg_json
        self._pub_event.publish(ev)
        self.get_logger().info(f'/system/event 発行: {event}')

    def _publish_summon_goal(self, p1, yaw):
        """SUMMON 受理時にゴール(P1=map xy, yaw)を /onsite/summon_goal へ publish。"""
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(p1[0])
        ps.pose.position.y = float(p1[1])
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = math.sin(float(yaw) / 2.0)
        ps.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._pub_summon_goal.publish(ps)
        self.get_logger().info(
            f'/onsite/summon_goal 発行 (x={p1[0]:.3f}, y={p1[1]:.3f}, '
            f'yaw={yaw:.3f})')

    # ── /onsite/pins publish ──────────────────────────────
    def _pin_to_msg(self, p) -> Pin:
        """pin dict → Pin メッセージ。yaw→quaternion 変換もここで行う。

        _publish_pins()（一覧 publish）と ROBOT_POSE 登録の response.pin の
        両方から使う（重複コードを増やさない）。
        """
        pin = Pin()
        pin.id = p.get('id', '')
        pin.name = p.get('name', '')
        pin.kind = p.get('kind', '')
        pose = p.get('pose', {}) or {}
        pin.pose.position.x = float(pose.get('x', 0.0))
        pin.pose.position.y = float(pose.get('y', 0.0))
        pin.pose.orientation.z = math.sin(float(pose.get('yaw', 0.0)) / 2.0)
        pin.pose.orientation.w = math.cos(float(pose.get('yaw', 0.0)) / 2.0)
        pin.registered_at.sec = int(p.get('registered_at', 0))
        return pin

    def _publish_pins(self):
        pl = PinList()
        pl.header = Header()
        pl.header.stamp = self.get_clock().now().to_msg()
        for p in self._pins:
            pl.pins.append(self._pin_to_msg(p))
        self._pub_pins.publish(pl)

    # ── /system/effect 受信 ───────────────────────────────
    def _on_effect(self, msg: StateEffect):
        name = msg.name
        if name == 'begin_two_point':
            args = json.loads(msg.args_json or '{}')
            self._accepting = True
            self._kind = args.get('kind', 'PANEL')
            self._p1 = None
            self._yaw = None
            self.get_logger().info(
                f'begin_two_point 受付中開始 (kind={self._kind})')
        elif name == 'place_pin':
            self._place_pin_effect()
        elif name == 'reject_register':
            self._accepting = False
            self._p1 = None
            self._yaw = None
            self.get_logger().info('reject_register: 受付中状態を破棄 (副作用なし)')

    # ── place_pin: 直近の (P1, yaw, kind) からピン生成・永続化 ──
    def _place_pin_effect(self):
        if self._p1 is None or self._yaw is None:
            self.get_logger().warn(
                'place_pin: P1/yaw が未確定のためピン生成スキップ')
            return
        x1, y1 = self._p1
        if self._kind == 'HOME':
            # HOME は 1 個のみ（既存があれば置換）
            self._pins = [p for p in self._pins if p.get('kind') != 'HOME']
        new_pin = build_pin_from_robot_pose(
            self._kind, x1, y1, float(self._yaw), self._pins)
        self._pins.append(new_pin)
        pid = new_pin['id']
        _dump_pins(self._pins_path, self._pins)
        self._publish_pins()
        self.get_logger().info(
            f'place_pin: ピン登録 {pid} (kind={self._kind}, '
            f'x={x1:.3f}, y={y1:.3f}, yaw={self._yaw:.3f})')
        # 登録後は受付中状態を元に戻す
        self._accepting = False
        self._p1 = None
        self._yaw = None

    # ── /person/status 受信 ───────────────────────────────
    def _on_person(self, msg: PersonStatus):
        self._person = msg

    # ── /onsite/two_point (TwoPointPress) ─────────────────
    def _on_two_point(self, request, response):
        purpose = request.purpose
        index = request.index
        response.yaw = 0.0

        if not self._accepting:
            response.accepted = False
            response.reject_reason_key = 'no_pending'
            return response

        # 対象妥当性（登録は走行時の 500ms 猶予を効かせない）
        valid, reason_key = self._target_valid()
        if not valid:
            response.accepted = False
            response.reject_reason_key = reason_key
            self._emit_event('evt.register_rejected',
                             json.dumps({'reason': reason_key}))
            self.get_logger().warn(
                f'two_point 拒否 (index={index}, reason={reason_key})')
            return response

        # 対象の map 姿勢。TF が無い（SLAM 未起動）ときは no_map_tf
        tm = self._target_map_pose()
        if tm is None:
            response.accepted = False
            response.reject_reason_key = 'no_map_tf'
            self._emit_event('evt.register_rejected',
                             json.dumps({'reason': 'no_map_tf'}))
            self.get_logger().warn('two_point 拒否: no_map_tf')
            return response

        if index == 1:
            self._p1 = tm
            response.accepted = True
            response.reject_reason_key = ''
            self.get_logger().info(f'P1 記録 (map {tm[0]:.3f}, {tm[1]:.3f})')
            return response

        # index == 2
        if self._p1 is None:
            response.accepted = False
            response.reject_reason_key = 'no_first_point'
            return response

        ok, dist = spacing_ok(self._p1[0], self._p1[1], tm[0], tm[1],
                              self._params)
        if not ok:
            response.accepted = False
            response.reject_reason_key = 'two_point_too_close'
            self._emit_event('evt.register_rejected',
                             json.dumps({'reason': 'two_point_too_close',
                                         'dist': round(dist, 3)}))
            self.get_logger().warn(
                f'two_point 拒否: two_point_too_close (dist={dist:.3f})')
            return response

        yaw = two_point_yaw(self._p1[0], self._p1[1], tm[0], tm[1])
        self._yaw = yaw
        response.accepted = True
        response.yaw = float(yaw)
        response.reject_reason_key = ''

        if purpose == 'SUMMON':
            self._emit_event('evt.two_point_done',
                             json.dumps({'yaw': round(float(yaw), 3)}))
            self._publish_summon_goal(self._p1, yaw)
        else:
            self._emit_event('evt.register_ok',
                             json.dumps({'kind': self._kind,
                                         'yaw': round(float(yaw), 3)}))
        self.get_logger().info(
            f'P2 記録・確定 (map {tm[0]:.3f}, {tm[1]:.3f}, '
            f'dist={dist:.3f}, yaw={yaw:.3f}, purpose={purpose})')
        return response

    # ── /onsite/register_pin (RegisterPin) ────────────────
    def _on_register_pin(self, request, response):
        if request.method == 'PLANE':
            # WAIVER(demo): W-08 — 方式 B (LiDAR 平面登録) は未実装
            response.success = False
            response.message = '方法B(平面登録)は未実装です'
            return response
        if request.method == 'ROBOT_POSE':
            # 機体姿勢での直接登録: その場で完了する処理（2 点指示のような
            # 受付状態を経由しない）。FSM の REGISTER 状態には関与しない。
            return self._on_register_pin_robot_pose(request, response)
        if request.method != 'TWO_POINT':
            response.success = False
            response.message = f'未知の方法: {request.method}'
            return response

        # TWO_POINT: begin_two_point を受けていない直接登録。
        # 内部で受付状態を開き、2 点押下フローをそのまま通す。
        self._accepting = True
        self._kind = request.kind
        self._p1 = None
        self._yaw = None
        response.success = True
        response.message = '2点指示で直接登録を開始しました (P1 を押してください)'
        return response

    def _on_register_pin_robot_pose(self, request, response):
        """ROBOT_POSE: 機体の現在姿勢 (map→base_link TF) をそのまま登録する。

        2 点指示と違い、対象検出や 2 点押下を要さず、現在の機体位置・向きで
        その場でピンを作って永続化する。_map_pose()（TF から (x,y,yaw) を取得）
        と _place_pin_effect()（pin 生成・永続化・publish の共通処理）を
        そのまま使い回し、response.pin には登録したピンを詰める
        （_publish_pins() と同じ yaw→quaternion 変換を _pin_to_msg で共有）。
        """
        mp = self._map_pose()
        if mp is None:
            response.success = False
            response.message = '自己位置（地図座標）が取得できません'
            return response
        self._p1 = (mp[0], mp[1])
        self._yaw = mp[2]
        self._kind = request.kind
        self._place_pin_effect()
        response.success = True
        response.pin = self._pin_to_msg(self._pins[-1])
        response.message = '機体の現在姿勢で登録しました'
        return response

    # ── /onsite/edit_pin (EditPin) ────────────────────────
    def _on_edit_pin(self, request, response):
        for p in self._pins:
            if p.get('id') == request.id:
                if request.is_delete:
                    self._pins.remove(p)
                    _dump_pins(self._pins_path, self._pins)
                    self._publish_pins()
                    response.success = True
                    response.message = f'ピン {request.id} を削除しました'
                    return response
                # 改名
                p['name'] = request.new_name
                _dump_pins(self._pins_path, self._pins)
                self._publish_pins()
                response.success = True
                response.message = f'ピン {request.id} を改名しました'
                return response
        response.success = False
        response.message = f'ピン {request.id} が見つかりません'
        return response

    # ── /onsite/reset_pins (Trigger) ──────────────────────
    def _on_reset_pins(self, request, response):
        """全ピンを消す（WS-9Y「新しい試験日として開始」）。

        /onsite/edit_pin の 1 件削除と違い、pins.yaml を空にして一括で消す。
        FSM を経由しない（discard_map と同じくスタンドアロン）。
        """
        n = len(self._pins)
        self._pins = []
        _dump_pins(self._pins_path, self._pins)
        self._publish_pins()
        response.success = True
        response.message = f'ピンをすべて削除しました ({n} 件)'
        self.get_logger().info(f'reset_pins: {n} 件のピンを削除しました')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PinRegistrar()
    executor = MultiThreadedExecutor(num_threads=4)  # WS-9U 以降の下限保証
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():  # SIGTERM 時は既定シグナルハンドラが context を落としている
            rclpy.shutdown()


if __name__ == '__main__':
    main()
