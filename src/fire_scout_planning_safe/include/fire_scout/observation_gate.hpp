#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>

namespace fire_scout {
// One evidence vote per timestamp per sensor topic. Mark only after successful
// integration: a scan waiting for its timestamped TF must remain retryable.
class ObservationStampGate {
public:
  bool fresh(const std::string &source, int64_t stamp) const {
    if (stamp <= 0) return false;
    const auto found = integrated_.find(source);
    return found == integrated_.end() || stamp > found->second;
  }
  void integrated(const std::string &source, int64_t stamp) {
    if (fresh(source, stamp)) integrated_[source] = stamp;
  }
  void reset() { integrated_.clear(); }
private:
  std::unordered_map<std::string, int64_t> integrated_;
};
} // namespace fire_scout
