#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "uavnav/core/event.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/jsonl_sink.hpp"

using namespace uavnav::events;

namespace {

// ---- Minimal strict JSON reader (RFC 8259) used to prove every line is valid JSON.
// Only what the sink emits is supported: flat object, nested "values" object,
// strings, numbers and null. Anything else fails the parse.
struct Json {
  enum class Kind { kNull, kNumber, kString, kObject } kind{Kind::kNull};
  double number{0.0};
  std::string text;  // number lexeme or decoded string
  std::vector<std::pair<std::string, Json>> members;

  const Json* find(std::string_view key) const {
    for (const auto& [k, v] : members)
      if (k == key) return &v;
    return nullptr;
  }
};

class Parser {
 public:
  explicit Parser(std::string_view s) : s_(s) {}

  std::optional<Json> parse() {
    Json out;
    if (!value(out)) return std::nullopt;
    if (i_ != s_.size()) return std::nullopt;  // trailing garbage
    return out;
  }

 private:
  bool eof() const { return i_ >= s_.size(); }

  bool value(Json& out) {
    if (eof()) return false;
    const char c = s_[i_];
    if (c == '{') return object(out);
    if (c == '"') {
      out.kind = Json::Kind::kString;
      return string(out.text);
    }
    if (s_.substr(i_, 4) == "null") {
      i_ += 4;
      out.kind = Json::Kind::kNull;
      return true;
    }
    return number(out);
  }

  bool object(Json& out) {
    out.kind = Json::Kind::kObject;
    ++i_;  // '{'
    if (!eof() && s_[i_] == '}') {
      ++i_;
      return true;
    }
    while (true) {
      std::string key;
      if (eof() || s_[i_] != '"' || !string(key)) return false;
      if (eof() || s_[i_] != ':') return false;
      ++i_;
      Json v;
      if (!value(v)) return false;
      out.members.emplace_back(std::move(key), std::move(v));
      if (eof()) return false;
      if (s_[i_] == ',') {
        ++i_;
        continue;
      }
      if (s_[i_] == '}') {
        ++i_;
        return true;
      }
      return false;
    }
  }

  static int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  bool string(std::string& out) {
    ++i_;  // opening quote
    while (!eof()) {
      const auto c = static_cast<unsigned char>(s_[i_++]);
      if (c == '"') return true;
      if (c < 0x20) return false;  // raw control character is invalid JSON
      if (c != '\\') {
        out.push_back(static_cast<char>(c));
        continue;
      }
      if (eof()) return false;
      const char e = s_[i_++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          if (i_ + 4 > s_.size()) return false;
          int cp = 0;
          for (int k = 0; k < 4; ++k) {
            const int h = hex(s_[i_++]);
            if (h < 0) return false;
            cp = cp * 16 + h;
          }
          if (cp > 0x7f) return false;  // the sink only emits \u00XX for control chars
          out.push_back(static_cast<char>(cp));
          break;
        }
        default: return false;
      }
    }
    return false;  // unterminated
  }

  // number = [ "-" ] int [ frac ] [ exp ]; int = "0" / digit1-9 *digit
  bool number(Json& out) {
    const std::size_t start = i_;
    if (!eof() && s_[i_] == '-') ++i_;
    if (eof() || s_[i_] < '0' || s_[i_] > '9') return false;
    if (s_[i_] == '0') {
      ++i_;
    } else {
      while (!eof() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    if (!eof() && s_[i_] == '.') {
      ++i_;
      if (eof() || s_[i_] < '0' || s_[i_] > '9') return false;
      while (!eof() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    if (!eof() && (s_[i_] == 'e' || s_[i_] == 'E')) {
      ++i_;
      if (!eof() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
      if (eof() || s_[i_] < '0' || s_[i_] > '9') return false;
      while (!eof() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
    }
    out.kind = Json::Kind::kNumber;
    out.text = std::string(s_.substr(start, i_ - start));
    out.number = std::strtod(out.text.c_str(), nullptr);
    return true;
  }

  std::string_view s_;
  std::size_t i_{0};
};

std::optional<Json> parse_json(std::string_view s) { return Parser(s).parse(); }

// ---- Fixtures ----

EventRecord make_record() {
  EventRecord r;
  r.t_steady_ns = 5;
  r.t_ros_ns = 7;
  r.component = Component::kSupervisor;
  r.event = "Commit";
  r.state_before = "RUNNING";
  r.state_after = "RUNNING";
  r.reason = "OK";
  r.identity = EventIdentity{1, 2, 3, 4, 5};
  r.add_value("prefix_s", 1.5);
  return r;
}

// A scratch file path unique to this process and test; removes only that file (and
// the directory when the test asked for one) on destruction.
class TempPath {
 public:
  explicit TempPath(std::string_view leaf, bool in_new_dir = false) {
    static std::atomic<int> counter{0};
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::ostringstream name;
    name << "uavnav_jsonl_" << info->test_suite_name() << "_" << info->name() << "_"
         << counter.fetch_add(1) << "_" << std::filesystem::path(leaf).stem().string();
    root_ = std::filesystem::temp_directory_path() / name.str();
    if (in_new_dir) {
      dir_ = root_;
      path_ = root_ / "nested" / "deeper" / leaf;
    } else {
      path_ = std::filesystem::temp_directory_path() / (name.str() + std::string(leaf).substr(leaf.rfind('.')));
    }
  }
  ~TempPath() {
    std::error_code ec;
    if (!dir_.empty()) {
      std::filesystem::remove_all(dir_, ec);  // a directory this test created
    } else {
      std::filesystem::remove(path_, ec);
    }
  }
  TempPath(const TempPath&) = delete;
  TempPath& operator=(const TempPath&) = delete;
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path root_, dir_, path_;
};

std::vector<std::string> read_lines(const std::filesystem::path& p) {
  std::ifstream in(p, std::ios::binary);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) lines.push_back(line);
  return lines;
}

std::string file_bytes(const std::filesystem::path& p) {
  std::ifstream in(p, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// JSON text of the "values" member of a single-value record.
std::string line_with_value(double v) {
  EventRecord r = make_record();
  r.value_count = 0;
  r.add_value("x", v);
  return to_json_line(r);
}

}  // namespace

TEST(JsonlSink, EncodesAllFieldsInOrder) {
  const EventRecord r = make_record();
  EXPECT_EQ(to_json_line(r),
            R"({"t_steady_ns":5,"t_ros_ns":7,"component":"supervisor","event":"Commit",)"
            R"("state_before":"RUNNING","state_after":"RUNNING","reason":"OK","mission_id":1,"lio_epoch":2,)"
            R"("request_id":3,"bundle_id":4,"world_revision":5,"values":{"prefix_s":1.5}})");
}

TEST(JsonlSink, EmptyValuesIsEmptyObject) {
  EventRecord r = make_record();
  r.value_count = 0;
  const std::string line = to_json_line(r);
  EXPECT_NE(line.find(R"("values":{})"), std::string::npos);
  EXPECT_TRUE(parse_json(line).has_value());
}

TEST(JsonlSink, NegativeAndExtremeIntegersAreExact) {
  EventRecord r = make_record();
  r.t_steady_ns = std::numeric_limits<std::int64_t>::min();
  r.t_ros_ns = std::numeric_limits<std::int64_t>::max();
  r.identity.mission_id = std::numeric_limits<std::uint64_t>::max();
  r.identity.lio_epoch = std::numeric_limits<std::uint32_t>::max();
  const std::string line = to_json_line(r);
  EXPECT_NE(line.find(R"("t_steady_ns":-9223372036854775808,)"), std::string::npos);
  EXPECT_NE(line.find(R"("t_ros_ns":9223372036854775807,)"), std::string::npos);
  EXPECT_NE(line.find(R"("mission_id":18446744073709551615,)"), std::string::npos);
  EXPECT_NE(line.find(R"("lio_epoch":4294967295,)"), std::string::npos);
  EXPECT_TRUE(parse_json(line).has_value());
}

TEST(JsonlSink, NonFiniteValuesBecomeNull) {
  EventRecord r = make_record();
  r.value_count = 0;
  r.add_value("a", std::numeric_limits<double>::quiet_NaN());
  r.add_value("b", std::numeric_limits<double>::infinity());
  r.add_value("c", -std::numeric_limits<double>::infinity());
  r.add_value("d", -std::numeric_limits<double>::quiet_NaN());
  r.add_value("e", 2.0);
  const std::string line = to_json_line(r);
  EXPECT_NE(line.find(R"("values":{"a":null,"b":null,"c":null,"d":null,"e":2})"), std::string::npos) << line;
  EXPECT_EQ(line.find("nan"), std::string::npos);
  EXPECT_EQ(line.find("inf"), std::string::npos);
  const auto json = parse_json(line);
  ASSERT_TRUE(json.has_value()) << line;
  const Json* values = json->find("values");
  ASSERT_NE(values, nullptr);
  ASSERT_EQ(values->members.size(), 5u);
  EXPECT_EQ(values->members[0].second.kind, Json::Kind::kNull);
  EXPECT_EQ(values->members[4].second.kind, Json::Kind::kNumber);
}

TEST(JsonlSink, FiniteDoublesAreValidJsonAndRoundTrip) {
  const double samples[] = {0.0,
                            -0.0,
                            1.0,
                            -1.0,
                            1.5,
                            0.1,
                            1.0 / 3.0,
                            123456789.0,
                            1e21,
                            1e300,
                            -1e300,
                            1e-300,
                            std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::lowest(),
                            std::numeric_limits<double>::min(),
                            std::numeric_limits<double>::denorm_min(),
                            std::numeric_limits<double>::epsilon()};
  for (const double v : samples) {
    const std::string line = line_with_value(v);
    const auto json = parse_json(line);
    ASSERT_TRUE(json.has_value()) << "invalid JSON for " << v << ": " << line;
    const Json* values = json->find("values");
    ASSERT_NE(values, nullptr);
    ASSERT_EQ(values->members.size(), 1u);
    const Json& x = values->members[0].second;
    ASSERT_EQ(x.kind, Json::Kind::kNumber) << line;
    EXPECT_EQ(x.number, v) << "round trip failed: " << x.text;
    EXPECT_EQ(std::signbit(x.number), std::signbit(v)) << x.text;
    EXPECT_EQ(x.text.find("0x"), std::string::npos);
    EXPECT_EQ(x.text.find("inf"), std::string::npos);
    EXPECT_EQ(x.text.find("nan"), std::string::npos);
  }
}

TEST(JsonlSink, NegativeZeroKeepsItsSign) {
  EXPECT_NE(line_with_value(-0.0).find(R"("x":-0})"), std::string::npos);
}

TEST(JsonlSink, EscapesQuotesBackslashAndNewline) {
  EventRecord r = make_record();
  r.reason = "a\"b\\c\nd";
  const std::string line = to_json_line(r);
  EXPECT_NE(line.find(R"("reason":"a\"b\\c\nd")"), std::string::npos) << line;
  EXPECT_EQ(line.find('\n'), std::string::npos);
  const auto json = parse_json(line);
  ASSERT_TRUE(json.has_value());
  EXPECT_EQ(json->find("reason")->text, "a\"b\\c\nd");
}

TEST(JsonlSink, EscapesTabCarriageReturnAndOtherControlChars) {
  EventRecord r = make_record();
  r.reason = "t\tr\rb\bf\fx\x01y\x1f";
  r.event = std::string_view("nul\0in", 6);
  const std::string line = to_json_line(r);
  EXPECT_NE(line.find(R"("reason":"t\tr\rb\bf\fx\u0001y\u001f")"), std::string::npos) << line;
  EXPECT_NE(line.find(R"("event":"nul\u0000in")"), std::string::npos) << line;
  for (const char c : line) EXPECT_GE(static_cast<unsigned char>(c), 0x20) << "raw control char in line";
  const auto json = parse_json(line);
  ASSERT_TRUE(json.has_value()) << line;
  EXPECT_EQ(json->find("reason")->text, "t\tr\rb\bf\fx\x01y\x1f");
}

TEST(JsonlSink, EscapesValueKeysAndEveryStringField) {
  EventRecord r = make_record();
  r.state_before = "s\"1";
  r.state_after = "s\\2";
  r.value_count = 0;
  r.add_value("k\"\n\\", 1.0);
  const std::string line = to_json_line(r);
  EXPECT_EQ(line.find('\n'), std::string::npos);
  const auto json = parse_json(line);
  ASSERT_TRUE(json.has_value()) << line;
  EXPECT_EQ(json->find("state_before")->text, "s\"1");
  EXPECT_EQ(json->find("state_after")->text, "s\\2");
  EXPECT_EQ(json->find("values")->members[0].first, "k\"\n\\");
}

TEST(JsonlSink, FullRecordWithSixteenValuesIsValidJson) {
  EventRecord r = make_record();
  r.value_count = 0;
  for (std::size_t i = 0; i < r.values.size(); ++i) r.add_value("v", static_cast<double>(i) * 0.25);
  const auto json = parse_json(to_json_line(r));
  ASSERT_TRUE(json.has_value());
  EXPECT_EQ(json->find("values")->members.size(), r.values.size());
}

TEST(JsonlSink, WritesOneLinePerRecordAndAppends) {
  TempPath tmp("events.jsonl");
  {
    auto sink = JsonlSink::open(tmp.path());
    ASSERT_TRUE(sink.has_value());
    const EventRecord batch[] = {make_record(), make_record()};
    ASSERT_TRUE((*sink)->write(batch).has_value());
    // Flushed per batch: readable while the sink is still open.
    EXPECT_EQ(read_lines(tmp.path()).size(), 2u);
  }
  {
    auto sink = JsonlSink::open(tmp.path());
    ASSERT_TRUE(sink.has_value());
    const EventRecord one[] = {make_record()};
    ASSERT_TRUE((*sink)->write(one).has_value());
  }
  const auto lines = read_lines(tmp.path());
  ASSERT_EQ(lines.size(), 3u);
  for (const auto& line : lines) {
    EXPECT_EQ(line, to_json_line(make_record()));
    EXPECT_TRUE(parse_json(line).has_value());
  }
  EXPECT_EQ(file_bytes(tmp.path()).back(), '\n');
}

TEST(JsonlSink, EmptyBatchWritesNothing) {
  TempPath tmp("empty.jsonl");
  auto sink = JsonlSink::open(tmp.path());
  ASSERT_TRUE(sink.has_value());
  EXPECT_TRUE((*sink)->write(std::span<const EventRecord>{}).has_value());
  EXPECT_TRUE(file_bytes(tmp.path()).empty());
}

TEST(JsonlSink, HostileStringsNeverSplitALine) {
  TempPath tmp("hostile.jsonl");
  auto sink = JsonlSink::open(tmp.path());
  ASSERT_TRUE(sink.has_value());
  EventRecord a = make_record();
  a.reason = "line1\nline2\r\nline3";
  EventRecord b = make_record();
  b.reason = "\x01\x02\"\\";
  const EventRecord batch[] = {a, b};
  ASSERT_TRUE((*sink)->write(batch).has_value());
  const auto lines = read_lines(tmp.path());
  ASSERT_EQ(lines.size(), 2u);
  for (const auto& line : lines) EXPECT_TRUE(parse_json(line).has_value()) << line;
}

TEST(JsonlSink, OpenCreatesParentDirectories) {
  TempPath tmp("nested.jsonl", /*in_new_dir=*/true);
  ASSERT_FALSE(std::filesystem::exists(tmp.path().parent_path()));
  auto sink = JsonlSink::open(tmp.path());
  ASSERT_TRUE(sink.has_value());
  const EventRecord one[] = {make_record()};
  ASSERT_TRUE((*sink)->write(one).has_value());
  EXPECT_EQ(read_lines(tmp.path()).size(), 1u);
}

TEST(JsonlSink, OpenAcceptsBareFilenameWithoutParentDirectory) {
  // A relative path with no directory part must not try to create "".
  TempPath tmp("bare.jsonl");
  const auto old_cwd = std::filesystem::current_path();
  std::error_code ec;
  std::filesystem::current_path(tmp.path().parent_path(), ec);
  ASSERT_FALSE(ec);
  const auto sink = JsonlSink::open(tmp.path().filename());
  std::filesystem::current_path(old_cwd, ec);
  ASSERT_TRUE(sink.has_value());
}

TEST(JsonlSink, OpenFailsOnUnwritablePath) {
  const auto sink = JsonlSink::open("/proc/uavnav_forbidden/x.jsonl");
  ASSERT_FALSE(sink.has_value());
  EXPECT_EQ(sink.error(), SinkError::kIo);
}

TEST(JsonlSink, OpenFailsWhenPathIsADirectory) {
  const auto sink = JsonlSink::open(std::filesystem::temp_directory_path());
  ASSERT_FALSE(sink.has_value());
  EXPECT_EQ(sink.error(), SinkError::kIo);
}

TEST(JsonlSink, OpenFailsWhenParentIsARegularFile) {
  TempPath tmp("parentfile.jsonl");
  { std::ofstream(tmp.path()) << "x"; }
  const auto sink = JsonlSink::open(tmp.path() / "child.jsonl");
  ASSERT_FALSE(sink.has_value());
  EXPECT_EQ(sink.error(), SinkError::kIo);
}

TEST(JsonlSink, OpenDoesNotThrowOnEmptyPath) {
  EXPECT_NO_THROW((void)JsonlSink::open(std::filesystem::path{}));
  SUCCEED();
}

TEST(JsonlSink, WriteReportsIoWhenStreamFailsAndDoesNotThrow) {
  // /dev/full accepts open() but every flush fails with ENOSPC.
  if (!std::filesystem::exists("/dev/full")) GTEST_SKIP() << "/dev/full not available";
  auto sink = JsonlSink::open("/dev/full");
  ASSERT_TRUE(sink.has_value());
  const EventRecord batch[] = {make_record()};
  uavnav::Result<void, SinkError> first;
  EXPECT_NO_THROW(first = (*sink)->write(batch));
  ASSERT_FALSE(first.has_value());
  EXPECT_EQ(first.error(), SinkError::kIo);
  // Still failing, still not throwing, on later batches.
  uavnav::Result<void, SinkError> second;
  EXPECT_NO_THROW(second = (*sink)->write(batch));
  ASSERT_FALSE(second.has_value());
  EXPECT_EQ(second.error(), SinkError::kIo);
}

TEST(JsonlSink, WorksAsRecorderSink) {
  TempPath tmp("recorder.jsonl");
  {
    auto sink = JsonlSink::open(tmp.path());
    ASSERT_TRUE(sink.has_value());
    EventRecorder recorder(std::move(*sink));
    EXPECT_TRUE(recorder.emit(make_record()));
    EXPECT_TRUE(recorder.emit(make_record()));
    recorder.flush();
  }
  EXPECT_EQ(read_lines(tmp.path()).size(), 2u);
}
