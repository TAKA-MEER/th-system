# worlds/ — Gazebo ワールド規約

シナリオ用ワールドを追加する際は、以下の必須要素を必ず含めること
（`panel_room_no_actor.world` が参照実装）。

## 必須要素

1. **physics ブロック** — `max_step_size 0.001 / real_time_update_rate 1000`
2. **gazebo_ros_state プラグイン** — これがないと person_mover / obstacle_mover /
   gazebo_person_relay が `/gazebo/set_entity_state` `/gazebo/model_states` を
   使えず沈黙する:
   ```xml
   <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
     <ros><namespace>/gazebo</namespace></ros>
     <update_rate>10.0</update_rate>
   </plugin>
   ```
3. **sun ライト + ground_plane**
4. **inspector モデル** — 追従対象の試験員シリンダー
   （`static=true`, radius 0.25, length 1.8, z=0.9）。
   モデル名 `inspector` は gazebo_person_relay / person_mover の
   デフォルトパラメータと一致させること。
5. **wanderer モデル** — 回避検証で使うシナリオのみ配置
   （radius 0.2, length 1.7）。obstacle_mover.py が動かす。

## センサ上の注意

- ロボットの 2D LiDAR は床上 約 0.15 m・最大レンジ 12 m。
  - 高さ 0.7 m の机天板はスキャンに映らない。脚（細シリンダー）だけ映る。
  - 壁・柱の間隔が 12 m を超えるとその方向は特徴なし → SLAM/AMCL が劣化する。
    広いワールドでは柱などの特徴物を 8 m 間隔以内で非対称に配置する。

## シナリオとの対応

各ワールドは `config/scenarios/<name>.yaml` から参照される。
遮蔽板を置くワールドでは、板の端点座標をシナリオ YAML の
`occlusion_segments` に転記し、**ワールド編集時は必ず両方を同期**させること。

| ワールド | シナリオ | 概要 |
|---|---|---|
| panel_room_no_actor.world | (デフォルト) / panel_shuttle | 10×8 m 配電盤室。th_map と整合 |
| panel_room.world | (旧 actor 版・保守のみ) | walk.dae actor 使用 |
| narrow_room.world | narrow_room | 幅 1.2 m の L 字通路 + 0.9 m 狭窄部 |
| wide_area.world | wide_area | 20×16 m ホール + 非対称柱 |
| cluttered.world | cluttered | 机・椅子(細脚)・柱のある部屋 |
| lost_reacquire.world | lost_reacquire | 自立遮蔽板 3 枚の部屋 |
