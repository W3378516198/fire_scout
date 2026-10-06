#pragma once
#include <cstdint>
#include <limits>
#include <optional>

namespace fire_scout {

// PointCloud2 has no separate snapshot id. Reserve at most one millisecond
// after the latest radar observation for unique, atomically pairable headers.
// This never reads wall/ROS time: sonar-only updates cannot refresh old radar
// observations indefinitely. Exhaustion preserves the caller's dirty state
// until a new observation advances source_ns.
class SnapshotStamp {
public:
  static constexpr int64_t max_offset_ns = 1000000;

  std::optional<int64_t> next(int64_t source_ns) {
    if (source_ns < 0) return std::nullopt;
    int64_t output = source_ns;
    if (last_ && output <= *last_) {
      if (*last_ == std::numeric_limits<int64_t>::max()) return std::nullopt;
      output = *last_ + 1;
    }
    // Both operands are nonnegative and output >= source_ns: no overflow.
    if (output - source_ns > max_offset_ns) return std::nullopt;
    last_ = output;
    return output;
  }

  void reset() { last_.reset(); }

private:
  std::optional<int64_t> last_;
};
} // namespace fire_scout
