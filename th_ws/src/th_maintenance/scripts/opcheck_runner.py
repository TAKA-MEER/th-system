#!/usr/bin/env python3
"""opcheck_runner.py — 始業点検（OPCHECK）の実行ノード（骨格）。

WP-MAINT-01。このコミットではパッケージ骨格の一部として起動できる最小構成。
判定関数・項目フローは完成させた check_core と合わせて後続コミットで入れる。
"""
import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node('opcheck_runner')
    node.get_logger().info('opcheck_runner 骨格: 起動しました')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()