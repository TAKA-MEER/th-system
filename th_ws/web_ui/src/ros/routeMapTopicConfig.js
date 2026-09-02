// ros/routeMapTopicConfig.js — pure (react-free) ROSLIB.Topic options for /map.
// Kept free of react/rosbridge so test/unit can assert the config with plain
// node --test (mutation 3 for WS-9F: dropping `compression:'cbor'` goes red).
//
// WS-9F: /route/preview の帯域圧縮に加えて、/map（OccupancyGrid）も巨大なので
// rosbridge の CBOR 圧縮で生バイト列（1 セル 1 byte）にして受信する。JSON の整数列
// （1 セル約 3 byte）に比べ約 1/3 になり、メインスレッドを固める巨大 JSON のパースも
// 避けられる。同梱の public/roslib.min.js は png / cbor / cbor-raw / none に対応。

export const ROUTE_MAP_TOPIC = '/map'
export const ROUTE_MAP_MSG = 'nav_msgs/OccupancyGrid'

// slam_toolbox 側の map_update_interval は 5.0s（slam_params.yaml）。地図 1 枚は
// 数十万セルの JSON で、受信のたびに RoutePreview が width×height の二重ループを
// 回して ImageData を作る＝メインスレッドが固まる。publish 周期より短く間引いても
// 意味が無いので、受信側もこの程度に抑えて取りこぼしの再送だけ受ける。
export const MAP_THROTTLE_MS = 2000

// routeMapTopicConfig(ros) -> ROSLIB.Topic コンストラクタへ渡すオプション群。
// queue_length / throttle_rate は従来どおり、compression:'cbor' を加えた（WS-9F）。
export function routeMapTopicConfig(ros) {
  return {
    ros,
    name: ROUTE_MAP_TOPIC,
    messageType: ROUTE_MAP_MSG,
    queue_length: 1,
    // /map は数十万セル。5秒ごとの巨大 JSON でメインスレッドが固まるので間引く。
    throttle_rate: MAP_THROTTLE_MS,
    compression: 'cbor',
  }
}
