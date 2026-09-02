// ros/routeMapTopicConfig.js — pure (react-free) ROSLIB.Topic options for the
// route-preview background map. Kept free of react/rosbridge so test/unit can
// assert the config with plain node --test (mutation 3 for WS-9F: dropping
// `compression:'cbor'` goes red; mutation: reverting the topic back to /map).
//
// WS-9G: 購読先は slam_toolbox の /map そのものではなく、map_downsampler が
// 表示用に間引いた /route/map_view を subscribe する。校舎 1 周級の /map は
// resolution 0.05m/セルで十数万〜百万セルにもなり、2.4GHz 無線を食い潰すため、
// 表示専用のコピーを factor(既定4)→0.20m/セルに畳んで配信している。
//
// WS-9F: /map（OccupancyGrid）も巨大なので、rosbridge の CBOR 圧縮で生バイト列
// （1 セル 1 byte）にして受信する。JSON の整数列（1 セル約 3 byte）に比べ約 1/3
// になり、メインスレッドを固める巨大 JSON のパースも避けられる。同梱の
// public/roslib.min.js は png / cbor / cbor-raw / none に対応。

export const ROUTE_MAP_TOPIC = '/route/map_view'
export const ROUTE_MAP_MSG = 'nav_msgs/OccupancyGrid'

// map_downsampler は publish_period_ms(既定 2s) に間引いて配信する。map 1 枚は
// 数十万セルで、受信のたびに RoutePreview が width×height の二重ループを回して
// ImageData を作る＝メインスレッドが固まる。publish 周期より短く間引いても
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
    // 数十万セルの地図でメインスレッドが固まるので間引いて受信する。
    throttle_rate: MAP_THROTTLE_MS,
    compression: 'cbor',
  }
}
