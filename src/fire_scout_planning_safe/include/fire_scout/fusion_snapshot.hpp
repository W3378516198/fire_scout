#pragma once

#include "map_snapshot.hpp"
#include <optional>

namespace fire_scout::fusion_snapshot {

inline bool less(Key a, Key b) {
  return a.z != b.z ? a.z < b.z : a.y != b.y ? a.y < b.y : a.x < b.x;
}

inline std::vector<Key> sortedKeys(const KeySet &cells) {
  std::vector<Key> result(cells.begin(), cells.end());
  std::sort(result.begin(), result.end(), less);
  return result;
}

// An unrotated frame maps voxel centres to the same integer offset everywhere.
// Near a rounding boundary retain the general TF path: floating point roundoff
// there can make neighbouring centres quantize differently. The margin covers
// double precision error across the complete signed 32-bit voxel range.
inline std::optional<Key> translationShift(Vec3 translation, double resolution) {
  if (!finite(translation) || !std::isfinite(resolution) || resolution < .05 || resolution > .5)
    return std::nullopt;
  Key result;
  const double value[3] = {translation.x, translation.y, translation.z};
  int *component[3] = {&result.x, &result.y, &result.z};
  for (size_t i = 0; i < 3; ++i) {
    const double centre = value[i] / resolution + .5;
    const double offset = std::floor(centre), fraction = centre - offset;
    if (fraction < 1e-4 || fraction > 1. - 1e-4 ||
        offset < INT32_MIN || offset > INT32_MAX)
      return std::nullopt;
    *component[i] = int(offset);
  }
  return result;
}

// common_free is sorted, unique, and contains no voxel with any occupied
// evidence. It is shared by all observers. Only the small occupied-evidence
// subset needs personalized confidence decisions and a separate sort. The
// result is byte-for-byte the regular canonical snapshot after translation.
inline std::vector<uint8_t> encode(const KeySet &occupied, const KeySet &extra_free,
                                   const std::vector<Key> &common_free,
                                   double resolution, Key shift = {}) {
  using namespace map_snapshot;
  if (!std::isfinite(resolution) || resolution < .05 || resolution > .5 ||
      occupied.size() > UINT32_MAX || common_free.size() > UINT32_MAX ||
      extra_free.size() > UINT32_MAX - common_free.size())
    throw std::runtime_error("Invalid sparse fusion snapshot resolution/count");
  const auto sorted_occupied = sortedKeys(occupied);
  std::vector<Key> sorted_extra;
  sorted_extra.reserve(extra_free.size());
  for (Key k : extra_free) if (!occupied.count(k)) sorted_extra.push_back(k);
  std::sort(sorted_extra.begin(), sorted_extra.end(), less);
  std::vector<uint8_t> out(record_bytes, 0);
  write32(out.data(), magic); write32(out.data() + 4, version);
  write32(out.data() + 8, uint32_t(occupied.size()));
  write32(out.data() + 20, floatBits(float(resolution)));
  Key start{}, last{};
  uint32_t run_length = 0, run_state = 0, free_count = 0;
  const auto flush = [&]() {
    if (!run_length) return;
    const size_t offset = out.size(); out.resize(offset + record_bytes, 0);
    auto *p = out.data() + offset;
    write32(p, uint32_t(start.x)); write32(p + 4, uint32_t(start.y));
    write32(p + 8, uint32_t(start.z)); write32(p + 12, run_length);
    write32(p + 16, run_state); write32(p + 20, floatBits(float(resolution)));
  };
  const auto append = [&](Key voxel, uint32_t state) {
    const int64_t x = int64_t(voxel.x) + shift.x;
    const int64_t y = int64_t(voxel.y) + shift.y;
    const int64_t z = int64_t(voxel.z) + shift.z;
    if (x < INT32_MIN || x > INT32_MAX || y < INT32_MIN || y > INT32_MAX ||
        z < INT32_MIN || z > INT32_MAX)
      throw std::runtime_error("Fusion voxel translation overflow");
    const Key translated{int(x), int(y), int(z)};
    if (run_length && run_state == state && translated == last) return;
    if (run_length && run_state == state && translated.z == last.z &&
        translated.y == last.y && int64_t(translated.x) == int64_t(last.x) + 1 &&
        run_length < UINT32_MAX) {
      ++run_length;
    } else {
      flush(); start = translated; run_length = 1; run_state = state;
    }
    last = translated;
    if (state == 2) ++free_count;
  };
  for (Key k : sorted_occupied) append(k, 1);
  size_t i = 0, j = 0;
  while (i < common_free.size() || j < sorted_extra.size()) {
    if (j == sorted_extra.size() || (i < common_free.size() && less(common_free[i], sorted_extra[j])))
      append(common_free[i++], 2);
    else append(sorted_extra[j++], 2);
  }
  flush(); write32(out.data() + 12, free_count);
  return out;
}

}  // namespace fire_scout::fusion_snapshot
