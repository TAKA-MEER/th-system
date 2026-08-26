// ============================================================
// test_scan_geometry_equivalence.cpp — Python版 scan_geometry.py と
// C++版 scan_geometry.hpp の等価性テスト（B-3）
//
// 対応する仕様（DetailedDesign-wp2.md WP-CALIB-01 §7）:
//   test_scan_geometry_equivalence（gtest, th_safety）:
//     B-3。Python と C++ が同じ添字を返す
//
// テストベクタ (test/data/scan_geometry_vectors.json) は
// th_testing/tools/gen_scan_geometry_vectors.py が Python 正本
// (th_perception/th_perception/scan_geometry.py) を実際に呼び出して
// 生成したもの（このパケットの指示により pybind11 は使わない。JSON
// 経由で突き合わせる方式）。
//
// このファイルは汎用 JSON ライブラリに依存しない。ベクタは自分たちの
// 生成器が書いた既知のスキーマだけを含むので、必要最小限の JSON パーサを
// このテストファイル内だけに自前で実装している（新規の外部依存
// （nlohmann-json 等）が対象 Docker イメージに入っているか未確認のため、
// 依存を増やさない判断をした。ホストでは Docker のビルド環境を確認
// できていないため、この判断が適切かどうかも含めてビルド確認が必要）。
// ============================================================
#include <gtest/gtest.h>

#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "th_safety/scan_geometry.hpp"

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
  double as_double() const { return number_value; }
  long as_long() const { return static_cast<long>(number_value); }
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
      throw std::runtime_error("scan_geometry_vectors.json: 予期しない終端");
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
          std::string("scan_geometry_vectors.json: '") + c + "' を期待したが '" + got + "'");
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
      throw std::runtime_error("scan_geometry_vectors.json: object の区切りが不正");
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
      throw std::runtime_error("scan_geometry_vectors.json: array の区切りが不正");
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
      throw std::runtime_error("scan_geometry_vectors.json: bool リテラルが不正");
    }
    return v;
  }

  JsonValue parse_null() {
    if (text_.compare(pos_, 4, "null") != 0) {
      throw std::runtime_error("scan_geometry_vectors.json: null リテラルが不正");
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
  const std::string path = std::string(TH_SAFETY_TEST_DATA_DIR) + "/scan_geometry_vectors.json";
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("scan_geometry_vectors.json を開けない: " + path);
  }
  std::ostringstream oss;
  oss << f.rdbuf();
  // JsonParser::text_ は const std::string& (参照) で持っている。
  // oss.str() を直接 JsonParser(...) に渡すと、返された一時オブジェクトが
  // フルコンプリート文の終わりで破棄され、text_ がダングリング参照になる
  // （名前付き変数に一度受けてから渡すことでこの一時オブジェクトの寿命を
  // 関数の終わりまで延ばす）。
  const std::string json_text = oss.str();
  JsonParser parser(json_text);
  return parser.parse();
}

th_safety::ScanGeometryInfo scan_from_json(const JsonValue& scan_json) {
  th_safety::ScanGeometryInfo scan;
  scan.angle_min = scan_json.at("angle_min").as_double();
  scan.angle_increment = scan_json.at("angle_increment").as_double();
  scan.num_ranges = static_cast<std::size_t>(scan_json.at("num_ranges").as_long());
  return scan;
}

const JsonValue& AllVectors() {
  static const JsonValue vectors = load_vectors_json();
  return vectors;
}

}  // namespace

// Python 正本 (scan_geometry.py) が実際に計算した期待値と、C++ 移植版
// (scan_geometry.hpp) の計算結果が全ベクタで一致することを確認する（B-3）。
TEST(ScanGeometryEquivalence, AllVectorsMatchPythonReference) {
  try {
    const JsonValue& root = AllVectors();
    const auto& vectors = root.at("vectors").array_value;
    ASSERT_FALSE(vectors.empty())
        << "テストベクタが空。gen_scan_geometry_vectors.py の再生成が必要";

    for (const JsonValue& vec : vectors) {
      const std::string name = vec.at("name").as_string();
      const std::string type = vec.at("type").as_string();
      const th_safety::ScanGeometryInfo scan = scan_from_json(vec.at("scan"));

      if (type == "angle_to_index") {
        const double angle_rad = vec.at("angle_rad").as_double();
        const JsonValue& expected = vec.at("expected_index");
        const auto actual = th_safety::angle_to_index(angle_rad, scan);

        if (expected.is_null()) {
          EXPECT_FALSE(actual.has_value())
              << name << ": 範囲外(None)を期待したが値 " << (actual ? *actual : 0) << " を返した";
        } else {
          ASSERT_TRUE(actual.has_value()) << name << ": 値を期待したが範囲外(None)だった";
          EXPECT_EQ(static_cast<long>(*actual), expected.as_long()) << name;
        }
      } else if (type == "sector_indices") {
        const double a0_deg = vec.at("a0_deg").as_double();
        const double a1_deg = vec.at("a1_deg").as_double();
        const JsonValue& expected_i0 = vec.at("expected_i0");
        const JsonValue& expected_i1 = vec.at("expected_i1");
        const auto [i0, i1] = th_safety::sector_indices(a0_deg, a1_deg, scan);

        if (expected_i0.is_null()) {
          EXPECT_FALSE(i0.has_value()) << name << " (i0): 範囲外(None)を期待した";
        } else {
          ASSERT_TRUE(i0.has_value()) << name << " (i0): 値を期待した";
          EXPECT_EQ(static_cast<long>(*i0), expected_i0.as_long()) << name << " (i0)";
        }

        if (expected_i1.is_null()) {
          EXPECT_FALSE(i1.has_value()) << name << " (i1): 範囲外(None)を期待した";
        } else {
          ASSERT_TRUE(i1.has_value()) << name << " (i1): 値を期待した";
          EXPECT_EQ(static_cast<long>(*i1), expected_i1.as_long()) << name << " (i1)";
        }
      } else {
        FAIL() << name << ": 未知のベクタ種別 " << type;
      }
    }
  } catch (const std::exception& e) {
    FAIL() << "テストベクタの読み込み/検証中に例外: " << e.what();
  }
}
