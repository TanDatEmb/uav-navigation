#include "uavnav/core/jsonl_sink.hpp"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>

namespace uavnav::events {

namespace {

void append_string(std::string& out, std::string_view s) {
  static constexpr char kHex[] = "0123456789abcdef";
  out.push_back('"');
  for (const char ch : s) {
    const auto c = static_cast<unsigned char>(ch);
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      default:
        if (c < 0x20) {  // every other control character: \u00XX
          out += "\\u00";
          out.push_back(kHex[c >> 4]);
          out.push_back(kHex[c & 0x0f]);
        } else {
          out.push_back(ch);  // printable ASCII and UTF-8 bytes pass through
        }
    }
  }
  out.push_back('"');
}

template <class Int>
void append_int(std::string& out, Int v) {
  static_assert(std::is_integral_v<Int>);
  char buf[24];  // longest: "-9223372036854775808" (20 chars)
  const auto res = std::to_chars(buf, buf + sizeof buf, v);
  out.append(buf, res.ptr);
}

// Shortest round-trip form. std::to_chars(double) without a format yields decimal or
// scientific notation such as "1.5", "-0", "1e+300", "5e-324", all valid JSON numbers.
// NaN and infinity have no JSON representation and become null.
void append_double(std::string& out, double v) {
  if (!std::isfinite(v)) {
    out += "null";
    return;
  }
  char buf[40];  // longest shortest-form double is about 24 chars
  const auto res = std::to_chars(buf, buf + sizeof buf, v);
  if (res.ec != std::errc{}) {  // cannot happen with this buffer; keep the line valid anyway
    out += "null";
    return;
  }
  out.append(buf, res.ptr);
}

void append_key(std::string& out, std::string_view key) {
  append_string(out, key);
  out.push_back(':');
}

void append_record(std::string& out, const EventRecord& r) {
  out += R"({"t_steady_ns":)";
  append_int(out, r.t_steady_ns);
  out += R"(,"t_ros_ns":)";
  append_int(out, r.t_ros_ns);
  out += R"(,"component":)";
  append_string(out, to_string(r.component));
  out += R"(,"event":)";
  append_string(out, r.event);
  out += R"(,"state_before":)";
  append_string(out, r.state_before);
  out += R"(,"state_after":)";
  append_string(out, r.state_after);
  out += R"(,"reason":)";
  append_string(out, r.reason);
  out += R"(,"mission_id":)";
  append_int(out, r.identity.mission_id);
  out += R"(,"lio_epoch":)";
  append_int(out, r.identity.lio_epoch);
  out += R"(,"request_id":)";
  append_int(out, r.identity.request_id);
  out += R"(,"bundle_id":)";
  append_int(out, r.identity.bundle_id);
  out += R"(,"world_revision":)";
  append_int(out, r.identity.world_revision);
  out += R"(,"values":{)";
  // Clamp: value_count is a public field, never read past the array.
  const std::size_t n = r.value_count < r.values.size() ? r.value_count : r.values.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (i != 0) out.push_back(',');
    append_key(out, r.values[i].key);
    append_double(out, r.values[i].value);
  }
  out += "}}";
}

}  // namespace

std::string to_json_line(const EventRecord& record) {
  std::string line;
  line.reserve(256);
  append_record(line, record);
  return line;
}

Result<std::unique_ptr<JsonlSink>, SinkError> JsonlSink::open(const std::filesystem::path& file) noexcept {
  try {
    if (file.empty()) return std::unexpected(SinkError::kIo);
    std::error_code ec;
    const auto parent = file.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, ec);  // ok if it already exists
      if (ec) return std::unexpected(SinkError::kIo);
    }
    std::ofstream out;
    out.exceptions(std::ios::goodbit);  // report failures through the stream state
    // No hidden buffer: a filebuf that fails to flush keeps the unwritten bytes and
    // replays them on the next flush or at close, which would duplicate and tear
    // lines. Unbuffered, each write() below goes straight to the OS and a failed batch
    // stays lost. Must be called before open().
    out.rdbuf()->pubsetbuf(nullptr, 0);
    out.open(file, std::ios::out | std::ios::app | std::ios::binary);
    if (!out.is_open() || !out.good()) return std::unexpected(SinkError::kIo);
    return std::unique_ptr<JsonlSink>(new JsonlSink(std::move(out)));
  } catch (...) {
    return std::unexpected(SinkError::kIo);
  }
}

Result<void, SinkError> JsonlSink::write(std::span<const EventRecord> batch) {
  if (batch.empty()) return {};
  try {
    buffer_.clear();
    if (tail_ == Tail::kMaybePartialLine) buffer_.push_back('\n');  // terminate a partial line from a failed write
    for (const EventRecord& record : batch) {
      append_record(buffer_, record);
      buffer_.push_back('\n');
    }
    out_.write(buffer_.data(), static_cast<std::streamsize>(buffer_.size()));
    out_.flush();
    if (!out_.good()) {
      out_.clear();  // let a later batch try again (e.g. space freed)
      tail_ = Tail::kMaybePartialLine;
      return std::unexpected(SinkError::kIo);
    }
    tail_ = Tail::kClean;
    return {};
  } catch (...) {  // allocation failure or a stream configured to throw
    out_.clear();
    tail_ = Tail::kMaybePartialLine;
    return std::unexpected(SinkError::kIo);
  }
}

}  // namespace uavnav::events
