#pragma once

#include "grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

namespace fire_scout {

// A yaw-aligned box around another UAV, expressed in the common world frame.
// Length is along the vehicle's body x axis, width along body y, and height
// along world z.  Full dimensions (not half extents) are used in parameters.
struct PeerUavBox {
  Vec3 center{};
  double yaw{0};
  double length{0};
  double width{0};
  double height{0};

  bool valid() const {
    return finite(center) && std::isfinite(yaw) && std::isfinite(length) &&
           std::isfinite(width) && std::isfinite(height) && length > 0 &&
           width > 0 && height > 0;
  }
};

// A short, bounded history of peer poses. Local radar maps are accumulated,
// so filtering only the newest peer pose can leave a moving UAV's previous
// body returns behind as a static obstacle. The history deliberately stores
// only spatially distinct samples and is capped by both time and count.
struct TimedPeerUavBox {
  PeerUavBox box{};
  double stamp{0};
};

struct PeerUavBoxHistoryConfig {
  double duration{0.80};
  double position_step{0.15};
  double yaw_step{15.0 * pi / 180.0};
  size_t max_boxes{12};

  bool valid() const {
    return std::isfinite(duration) && duration >= 0 && duration <= 5.0 &&
           std::isfinite(position_step) && position_step > 0 &&
           position_step <= 2.0 && std::isfinite(yaw_step) && yaw_step > 0 &&
           yaw_step <= pi && max_boxes > 0 && max_boxes <= 100;
  }
};

class PeerUavBoxHistory {
public:
  explicit PeerUavBoxHistory(PeerUavBoxHistoryConfig config = {})
      : config_(config) {}

  bool valid() const { return config_.valid(); }

  void clear() { samples_.clear(); }

  void add(const PeerUavBox &box, double stamp) {
    if (!valid() || !box.valid() || !std::isfinite(stamp)) return;
    if (!samples_.empty() && stamp < samples_.back().stamp) samples_.clear();
    prune(stamp);
    if (config_.duration <= 0) return;
    bool distinct = samples_.empty();
    if (!distinct) {
      const auto &last = samples_.back().box;
      const double dx = box.center.x - last.center.x;
      const double dy = box.center.y - last.center.y;
      const double dz = box.center.z - last.center.z;
      const double yaw_delta = std::abs(wrap(box.yaw - last.yaw));
      distinct = std::sqrt(dx * dx + dy * dy + dz * dz) >=
                     config_.position_step ||
                 yaw_delta >= config_.yaw_step;
    }
    if (!distinct) return;
    samples_.push_back({box, stamp});
    while (samples_.size() > config_.max_boxes) samples_.pop_front();
  }

  std::vector<PeerUavBox> boxes(double now) {
    prune(now);
    std::vector<PeerUavBox> result;
    result.reserve(samples_.size());
    for (const auto &sample : samples_) result.push_back(sample.box);
    return result;
  }

  size_t size() const { return samples_.size(); }

private:
  void prune(double now) {
    if (!std::isfinite(now)) return;
    while (!samples_.empty() &&
           (now < samples_.front().stamp ||
            now - samples_.front().stamp > config_.duration))
      samples_.pop_front();
  }

  PeerUavBoxHistoryConfig config_{};
  std::deque<TimedPeerUavBox> samples_;
};

inline bool insidePeerUavBox(Vec3 point, const PeerUavBox &box,
                             double tolerance = 1e-9) {
  if (!finite(point) || !box.valid() || !std::isfinite(tolerance) ||
      tolerance < 0)
    return false;
  const double dx = point.x - box.center.x;
  const double dy = point.y - box.center.y;
  const double c = std::cos(box.yaw);
  const double s = std::sin(box.yaw);
  const double body_x = c * dx + s * dy;
  const double body_y = -s * dx + c * dy;
  return std::abs(body_x) <= 0.5 * box.length + tolerance &&
         std::abs(body_y) <= 0.5 * box.width + tolerance &&
         std::abs(point.z - box.center.z) <= 0.5 * box.height + tolerance;
}

// Erase only voxels whose centres are inside the oriented box.  Iterating the
// small box AABB is substantially cheaper than scanning a multi-million-cell
// map for every peer UAV.
inline size_t erasePeerUavBox(KeySet &voxels, double resolution,
                              const PeerUavBox &box) {
  if (voxels.empty() || !box.valid() || !std::isfinite(resolution) ||
      resolution <= 0)
    return 0;
  const double c = std::abs(std::cos(box.yaw));
  const double s = std::abs(std::sin(box.yaw));
  const double extent_x = 0.5 * (c * box.length + s * box.width);
  const double extent_y = 0.5 * (s * box.length + c * box.width);
  const Vec3 low{box.center.x - extent_x, box.center.y - extent_y,
                 box.center.z - 0.5 * box.height};
  const Vec3 high{box.center.x + extent_x, box.center.y + extent_y,
                  box.center.z + 0.5 * box.height};
  const Key begin = key(low, resolution);
  const Key end = key(high, resolution);

  size_t removed = 0;
  for (int x = begin.x; x <= end.x; ++x)
    for (int y = begin.y; y <= end.y; ++y)
      for (int z = begin.z; z <= end.z; ++z) {
        const Key voxel{x, y, z};
        if (insidePeerUavBox(center(voxel, resolution), box))
          removed += voxels.erase(voxel);
      }
  return removed;
}

inline size_t erasePeerUavBoxes(KeySet &voxels, double resolution,
                                const std::vector<PeerUavBox> &boxes) {
  size_t removed = 0;
  for (const auto &box : boxes)
    removed += erasePeerUavBox(voxels, resolution, box);
  return removed;
}

// Move peer-box voxels into a separate uncertainty set instead of turning the
// complete box into measured free space.  The fusion node uses this to keep a
// dynamic UAV return from becoming a static wall while avoiding the opposite
// error of claiming that an occluded wall behind the UAV is free.
inline size_t extractPeerUavBox(KeySet &voxels, KeySet &uncertain,
                                double resolution, const PeerUavBox &box) {
  if (voxels.empty() || !box.valid() || !std::isfinite(resolution) ||
      resolution <= 0)
    return 0;
  const double c = std::abs(std::cos(box.yaw));
  const double s = std::abs(std::sin(box.yaw));
  const double extent_x = 0.5 * (c * box.length + s * box.width);
  const double extent_y = 0.5 * (s * box.length + c * box.width);
  const Key begin = key({box.center.x - extent_x, box.center.y - extent_y,
                         box.center.z - 0.5 * box.height},
                        resolution);
  const Key end = key({box.center.x + extent_x, box.center.y + extent_y,
                       box.center.z + 0.5 * box.height},
                      resolution);

  size_t moved = 0;
  for (int x = begin.x; x <= end.x; ++x)
    for (int y = begin.y; y <= end.y; ++y)
      for (int z = begin.z; z <= end.z; ++z) {
        const Key voxel{x, y, z};
        const auto found = voxels.find(voxel);
        if (found == voxels.end() ||
            !insidePeerUavBox(center(voxel, resolution), box))
          continue;
        uncertain.insert(voxel);
        voxels.erase(found);
        ++moved;
      }
  return moved;
}

inline size_t extractPeerUavBoxes(KeySet &voxels, KeySet &uncertain,
                                  double resolution,
                                  const std::vector<PeerUavBox> &boxes) {
  size_t moved = 0;
  for (const auto &box : boxes)
    moved += extractPeerUavBox(voxels, uncertain, resolution, box);
  return moved;
}

}  // namespace fire_scout
