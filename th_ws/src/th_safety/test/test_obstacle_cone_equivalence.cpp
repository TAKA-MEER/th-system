// ============================================================
// test_obstacle_cone_equivalence.cpp — Python版 observe_cone()
// (mapless_follow_core.py) と C++版 observe_cone()
// (obstacle_limiter_core.cpp) の突き合わせテスト
//
// 対応する仕様（docs/plan/detailed/DetailedDesign-safety.md §3.4.3）:
//   「既存 is_path_blocked() は bool しか返さないので、最近傍距離を返す
//    形に拡張して移植する。拡張後、既存 Python 実装との等価性テストを
//    書く」
//
// ── ビット一致ではなく「包含性」を検証する（DetailedDesign-open.md N-12） ──
// 実装当初は bit-for-bit の等価性を狙っていたが、実測でそれは成り立たない
// ことが分かった。原因は集合の作り方の違い:
//   - Python 版 observe_cone(): 各ビームの中心角度が direction±half_width
//     の範囲に入っているかを個別に判定する（角度ベース）。
//   - C++ 版 observe_cone(): コーンの両端の角度を scan_geometry の
//     sector_indices() で添字に変換し（floor による丸め）、その閉区間
//     [i0, i1] を機械的に走査する（添字ベース）。floor は「その角度を
//     含むビン」を返すため、下端側 (i0) はコーンの外側（direction -
//     half_width よりさらに外）のビームを拾うことがある。
// この違いにより **C++ が選ぶビーム集合は常に Python が選ぶビーム集合の
// 上位集合になる**（half_width が angle_increment 程度以下の狭いコーンで
// 頻発する。生成器のランダム fuzz で実測: 一致多数・C++ がより近い障害物を
// 検出した件数が一定数・Python の方が近い障害物を検出した違反は 0 件。
// 実測値は実装報告に記載）。
//
// C++ が「見落とす」方向の食い違いが無いことが安全上の意味を持つ性質
// （後段のリミッタが前段の挙動判定より見落とさないことの保証）なので、
// このテストは **「C++ の最近傍距離は常に Python の最近傍距離以下」という
// 包含性**を検証する（ビット一致ではない）。あわせて、テストベクタが実際に
// 「C++ の方が近い障害物を検出した」ケースを含んでいること
// （cpp_more_conservative_count > 0）も表明する ── そうしないと、このテストは
// 分岐が起きない入力ばかりを踏んで「たまたま通っている」だけになりうる。
//
// テストベクタ (test/data/obstacle_cone_vectors.json) は
// th_testing/tools/gen_obstacle_cone_vectors.py が Python 正本
// (th_planning/th_planning/mapless_follow_core.py の observe_cone()) を
// 実際に呼び出して生成したもの（pybind11 は使わず JSON 経由で突き合わせる。
// test_scan_geometry_equivalence.cpp・B-3 と同じ方式）。
//
// このファイルは test_scan_geometry_equivalence.cpp と同じ理由で汎用 JSON
// ライブラリに依存しない自前の最小限パーサを使う。ranges 配列は
// inf/-inf/nan を文字列マーカー("inf"/"-inf"/"nan")として運ぶ拡張が
// 追加されている点だけ scan_geometry 版と異なる（JSON は Infinity/NaN を
// 標準で表現できないため）。
//
// 対象は全周スキャンに限定してある（生成器 gen_obstacle_cone_vectors.py の
// docstring・DetailedDesign-open.md N-12 参照）。そのため
// expected_covered は全ベクタで true 固定になっている。
// ============================================================
#include <gtest/gtest.h>

#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "th_safety/obstacle_limiter_core.hpp"

namespace {

// ── 最小限の JSON パーサ（このテストファイル専用。汎用性は求めない） ──
struct JsonValue {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };
  Type type = Type::kNull;
  bool bool_value = false;
  double number_value = 0.0;
  std::string string_value;
  std::vector<JsonValue> array_value;
  std::map<std::string, JsonValue> object_value;

  bool is_null() const { return type == Type::kNull; }
  bool is_string() const { return type == Type::kString; }
  double as_double() const { return number_value; }
  bool as_bool() const { return bool_value; }
  const std::string& as_string() const { return string_value; }
  const JsonValue& at(const std::string& key) const { return object_value.at(key); }
};

class JsonParser {
 public:
  explicit JsonParser(const std::string& text) : text_(text) {}

  JsonValue parse() {
    skip_ws();
    JsonValue v = parse_value();
    skip_ws();
    return v;
  }

 private:
  const std::string& text_;
  std::size_t pos_ = 0;

  char peek() const {
    if (pos_ >= text_.size()) {
      throw std::runtime_error("obstacle_cone_vectors.json: 予期しない終端");
    }
    return text_[pos_];
  }

  char get() { return text_[pos_++]; }

  void skip_ws() {
    while (pos_ < text_.size() &&
           (text_[pos_] == ' ' || text_[pos_] == '\t' ||
            text_[pos_] == '\n' || text_[pos_] == '\r')) {
      ++pos_;
    }
  }

  void expect(char c) {
    const char got = get();
    if (got != c) {
      throw std::runtime_error(
          std::string("obstacle_cone_vectors.json: '") + c + "' を期待したが '" + got + "'");
    }
  }

  JsonValue parse_value() {
    skip_ws();
    const char c = peek();
    if (c == '{') return parse_object();
    if (c == '[') return parse_array();
    if (c == '"') return parse_string_value();
    if (c == 't' || c == 'f') return parse_bool();
    if (c == 'n') return parse_null();
    return parse_number();
  }

  JsonValue parse_object() {
    JsonValue v;
    v.type = JsonValue::Type::kObject;
    expect('{');
    skip_ws();
    if (peek() == '}') {
      get();
      return v;
    }
    while (true) {
      skip_ws();
      const std::string key = parse_string();
      skip_ws();
      expect(':');
      JsonValue val = parse_value();
      v.object_value.emplace(key, std::move(val));
      skip_ws();
      const char c = get();
      if (c == ',') continue;
      if (c == '}') break;
      throw std::runtime_error("obstacle_cone_vectors.json: object の区切りが不正");
    }
    return v;
  }

  JsonValue parse_array() {
    JsonValue v;
    v.type = JsonValue::Type::kArray;
    expect('[');
    skip_ws();
    if (peek() == ']') {
      get();
      return v;
    }
    while (true) {
      JsonValue val = parse_value();
      v.array_value.push_back(std::move(val));
      skip_ws();
      const char c = get();
      if (c == ',') continue;
      if (c == ']') break;
      throw std::runtime_error("obstacle_cone_vectors.json: array の区切りが不正");
    }
    return v;
  }

  std::string parse_string() {
    expect('"');
    std::string s;
    while (true) {
      const char c = get();
      if (c == '"') break;
      if (c == '\\') {
        const char e = get();
        switch (e) {
          case '"': s += '"'; break;
          case '\\': s += '\\'; break;
          case '/': s += '/'; break;
          case 'n': s += '\n'; break;
          case 't': s += '\t'; break;
          case 'r': s += '\r'; break;
          default: s += e; break;
        }
      } else {
        s += c;
      }
    }
    return s;
  }

  JsonValue parse_string_value() {
    JsonValue v;
    v.type = JsonValue::Type::kString;
    v.string_value = parse_string();
    return v;
  }

  JsonValue parse_bool() {
    JsonValue v;
    v.type = JsonValue::Type::kBool;
    if (text_.compare(pos_, 4, "true") == 0) {
      v.bool_value = true;
      pos_ += 4;
    } else if (text_.compare(pos_, 5, "false") == 0) {
      v.bool_value = false;
      pos_ += 5;
    } else {
      throw std::runtime_error("obstacle_cone_vectors.json: bool リテラルが不正");
    }
    return v;
  }

  JsonValue parse_null() {
    if (text_.compare(pos_, 4, "null") != 0) {
      throw std::runtime_error("obstacle_cone_vectors.json: null リテラルが不正");
    }
    pos_ += 4;
    JsonValue v;
    v.type = JsonValue::Type::kNull;
    return v;
  }

  JsonValue parse_number() {
    const std::size_t start = pos_;
    if (peek() == '-') get();
    while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) get();
    if (pos_ < text_.size() && text_[pos_] == '.') {
      get();
      while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) get();
    }
    if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
      get();
      if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) get();
      while (pos_ < text_.size() && std::isdigit(static_cast<unsigned char>(text_[pos_]))) get();
    }
    JsonValue v;
    v.type = JsonValue::Type::kNumber;
    v.number_value = std::stod(text_.substr(start, pos_ - start));
    return v;
  }
};

JsonValue load_vectors_json() {
#ifndef TH_SAFETY_TEST_DATA_DIR
#error "TH_SAFETY_TEST_DATA_DIR が定義されていない（CMakeLists.txt を確認）"
#endif
  const std::string path = std::string(TH_SAFETY_TEST_DATA_DIR) + "/obstacle_cone_vectors.json";
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("obstacle_cone_vectors.json を開けない: " + path);
  }
  std::ostringstream oss;
  oss << f.rdbuf();
  // JsonParser::text_ は const std::string& (参照) で持っている。名前付き
  // 変数に一度受けてから渡すことで一時オブジェクトの寿命を関数の終わりまで
  // 延ばす（test_scan_geometry_equivalence.cpp と同じ注意点）。
  const std::string json_text = oss.str();
  JsonParser parser(json_text);
  return parser.parse();
}

const JsonValue& AllVectors() {
  static const JsonValue vectors = load_vectors_json();
  return vectors;
}

// ranges 配列の要素を double に変換する。数値はそのまま、文字列
// "inf"/"-inf"/"nan" は対応する特殊値に変換する（生成器の _enc() と対）。
double range_value_from_json(const JsonValue& v) {
  if (v.is_string()) {
    const std::string& s = v.as_string();
    if (s == "inf") return std::numeric_limits<double>::infinity();
    if (s == "-inf") return -std::numeric_limits<double>::infinity();
    if (s == "nan") return std::numeric_limits<double>::quiet_NaN();
    throw std::runtime_error("obstacle_cone_vectors.json: 未知の range 文字列マーカー: " + s);
  }
  return v.as_double();
}

th_safety::ScanSnapshot scan_from_json(const JsonValue& vec) {
  th_safety::ScanSnapshot scan;
  scan.received = true;
  scan.stamp_sec = 0.0;
  const JsonValue& scan_json = vec.at("scan");
  scan.geometry.angle_min = scan_json.at("angle_min").as_double();
  scan.geometry.angle_increment = scan_json.at("angle_increment").as_double();
  scan.geometry.num_ranges =
      static_cast<std::size_t>(scan_json.at("num_ranges").as_double());

  const auto& ranges_json = vec.at("ranges").array_value;
  scan.ranges.reserve(ranges_json.size());
  for (const JsonValue& r : ranges_json) {
    scan.ranges.push_back(range_value_from_json(r));
  }
  return scan;
}

}  // namespace

// Python 正本 (mapless_follow_core.observe_cone()) が実際に計算した「最近傍
// 距離」と、C++ 版 (obstacle_limiter_core.hpp の observe_cone()) の計算結果
// を突き合わせる。ヘッダコメントのとおりビット一致は主張しない ──
// 主張するのは「C++ の最近傍距離は常に Python の最近傍距離以下(=より保守的、
// 見落としが無い)」という包含性で、加えてテストベクタが実際にその分岐
// (C++ の方が近い障害物を検出するケース)を含んでいることも表明する。
TEST(ObstacleConeEquivalence, CppNearestIsAlwaysAtMostAsFarAsPython) {
  try {
    const JsonValue& root = AllVectors();
    const auto& vectors = root.at("vectors").array_value;
    ASSERT_FALSE(vectors.empty())
        << "テストベクタが空。gen_obstacle_cone_vectors.py の再生成が必要";

    int violation_count = 0;          // Python の方が近い障害物を検出(あってはならない)
    int exact_match_count = 0;        // 両者が(許容誤差内で)一致
    int cpp_more_conservative_count = 0;  // C++ の方が近い障害物を検出(想定内の分岐)
    int covered_mismatch_count = 0;

    for (const JsonValue& vec : vectors) {
      const std::string name = vec.at("name").as_string();
      const th_safety::ScanSnapshot scan = scan_from_json(vec);
      const double direction_rad = vec.at("direction_rad").as_double();
      const double half_width_rad = vec.at("half_width_rad").as_double();
      const bool expected_covered = vec.at("expected_covered").as_bool();
      const JsonValue& expected_nearest_json = vec.at("expected_nearest_m");

      const th_safety::ConeObservation actual =
          th_safety::observe_cone(scan, direction_rad, half_width_rad);

      if (actual.covered != expected_covered) {
        ++covered_mismatch_count;
        ADD_FAILURE() << name << ": covered 不一致。期待=" << expected_covered
                      << " 実際=" << actual.covered
                      << "（全周スキャン限定のテストベクタで発生するのは想定外。"
                      << "scan の geometry を作り損ねている可能性）";
        continue;
      }

      // Python 側「見つからなかった(None)」は「+infinity」として扱う
      // （C++ 側の「covered かつ有限レンジが1つも無い」と対になる中立値。
      // observe_cone_core.hpp の N-11 と同じ考え方）。
      const double python_nearest =
          expected_nearest_json.is_null() ? std::numeric_limits<double>::infinity()
                                           : expected_nearest_json.as_double();
      const double cpp_nearest = actual.nearest_m;

      constexpr double kEps = 1e-9;
      if (cpp_nearest > python_nearest + kEps) {
        // 違反: C++ が Python より遠い障害物しか見ていない = 見落とし。
        // 包含性(C++ ⊇ Python のビーム集合)が成り立つなら本来起こり得ない。
        ++violation_count;
        ADD_FAILURE() << name << ": 包含性違反。Python=" << python_nearest
                      << " C++=" << cpp_nearest << "(C++の方が遠い=見落とし)";
      } else if (cpp_nearest < python_nearest - kEps) {
        ++cpp_more_conservative_count;
      } else {
        ++exact_match_count;
      }
    }

    RecordProperty("vector_count", static_cast<int>(vectors.size()));
    RecordProperty("exact_match_count", exact_match_count);
    RecordProperty("cpp_more_conservative_count", cpp_more_conservative_count);
    RecordProperty("violation_count", violation_count);
    RecordProperty("covered_mismatch_count", covered_mismatch_count);

    EXPECT_EQ(violation_count, 0)
        << "包含性違反が0件であること(C++がPythonより遠い障害物しか見ない"
        << "=見落としがあってはならない)";
    // このテストが分岐(添字ベース vs 角度ベースの食い違い)を実際に踏んで
    // いることの担保。0件なら「たまたま通っている」だけの弱いテストに
    // 退化しているので、テストベクタ側の不備として検出する。
    EXPECT_GT(cpp_more_conservative_count, 0)
        << "食い違いが起きるケースをテストベクタが1件も踏んでいない。"
        << "gen_obstacle_cone_vectors.py の狭コーン/境界ケースを確認すること";
  } catch (const std::exception& e) {
    FAIL() << "テストベクタの読み込み/検証中に例外: " << e.what();
  }
}
