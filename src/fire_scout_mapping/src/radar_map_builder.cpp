#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <array>
#include <chrono>


class RadarMapBuilder : public rclcpp::Node
{
public:
  RadarMapBuilder()
  : Node("radar_map_builder")
  {
    target_frame_ =
      this->declare_parameter<std::string>(
        "target_frame",
        "scout1/odom");

    left_topic_ =
      this->declare_parameter<std::string>(
        "left_topic",
        "/scout1/radar/left/mmwave_points");

    right_topic_ =
      this->declare_parameter<std::string>(
        "right_topic",
        "/scout1/radar/right/mmwave_points");

    map_topic_ =
      this->declare_parameter<std::string>(
        "map_topic",
        "/scout1/map/radar_cloud");

    voxel_size_ =
      this->declare_parameter<double>(
        "voxel_size",
        0.15);

    max_voxels_ =
      this->declare_parameter<int64_t>(
        "max_voxels",
        300000);

    publish_rate_hz_ =
      this->declare_parameter<double>(
        "publish_rate_hz",
        2.0);

    tf_timeout_sec_ =
      this->declare_parameter<double>(
        "tf_timeout_sec",
        0.10);

    if (voxel_size_ <= 0.0) {
      throw std::runtime_error("voxel_size must be > 0");
    }

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be > 0");
    }

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(
        this->get_clock());

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(
        *tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS();

    left_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        left_topic_,
        sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          processCloud(msg, "left");
        });

    right_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        right_topic_,
        sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          processCloud(msg, "right");
        });

    auto map_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    map_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        map_topic_,
        map_qos);

    const auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(
          1.0 / publish_rate_hz_));

    publish_timer_ =
      this->create_wall_timer(
        period,
        std::bind(
          &RadarMapBuilder::publishMap,
          this));

    RCLCPP_INFO(
      this->get_logger(),
      "Radar map builder started.");

    RCLCPP_INFO(
      this->get_logger(),
      "Target frame: %s",
      target_frame_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Left topic: %s",
      left_topic_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Right topic: %s",
      right_topic_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Map topic: %s",
      map_topic_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Voxel size: %.3f m",
      voxel_size_);

    RCLCPP_INFO(
      this->get_logger(),
      "Max voxels: %ld",
      max_voxels_);
  }

private:

  struct VoxelKey
  {
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator==(const VoxelKey & other) const
    {
      return
        x == other.x &&
        y == other.y &&
        z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t seed = 0;

      auto combine =
        [&seed](int64_t value)
        {
          std::size_t h =
            std::hash<int64_t>{}(value);

          seed ^=
            h +
            0x9e3779b97f4a7c15ULL +
            (seed << 6) +
            (seed >> 2);
        };

      combine(key.x);
      combine(key.y);
      combine(key.z);

      return seed;
    }
  };

  struct VoxelCell
  {
    double x;
    double y;
    double z;

    uint32_t count;
  };


  void processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const std::string & sensor_name)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s point cloud has empty frame_id.",
        sensor_name.c_str());

      return;
    }

    sensor_msgs::msg::PointCloud2 transformed;

    try {
      const rclcpp::Time cloud_stamp(
        msg->header.stamp);

      auto transform =
        tf_buffer_->lookupTransform(
          target_frame_,
          msg->header.frame_id,
          cloud_stamp,
          rclcpp::Duration::from_seconds(
            tf_timeout_sec_));

      tf2::doTransform(
        *msg,
        transformed,
        transform);
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "%s TF failed: %s",
        sensor_name.c_str(),
        ex.what());

      return;
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float>
        iter_x(transformed, "x");

      sensor_msgs::PointCloud2ConstIterator<float>
        iter_y(transformed, "y");

      sensor_msgs::PointCloud2ConstIterator<float>
        iter_z(transformed, "z");

      std::lock_guard<std::mutex> lock(map_mutex_);

      for (
        ;
        iter_x != iter_x.end();
        ++iter_x, ++iter_y, ++iter_z)
      {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;

        if (
          !std::isfinite(x) ||
          !std::isfinite(y) ||
          !std::isfinite(z))
        {
          continue;
        }

        VoxelKey key;

        key.x =
          static_cast<int64_t>(
            std::floor(
              static_cast<double>(x) /
              voxel_size_));

        key.y =
          static_cast<int64_t>(
            std::floor(
              static_cast<double>(y) /
              voxel_size_));

        key.z =
          static_cast<int64_t>(
            std::floor(
              static_cast<double>(z) /
              voxel_size_));

        auto it = voxels_.find(key);

        if (it == voxels_.end()) {

          if (
            static_cast<int64_t>(voxels_.size())
            >= max_voxels_)
          {
            ++dropped_new_voxels_;
            continue;
          }

          voxels_.emplace(
            key,
            VoxelCell{
              static_cast<double>(x),
              static_cast<double>(y),
              static_cast<double>(z),
              1});
        }
        else {

          auto & cell = it->second;

          if (cell.count < UINT32_MAX) {
            ++cell.count;
          }

          const double n =
            static_cast<double>(cell.count);

          cell.x +=
            (static_cast<double>(x) - cell.x) / n;

          cell.y +=
            (static_cast<double>(y) - cell.y) / n;

          cell.z +=
            (static_cast<double>(z) - cell.z) / n;
        }
      }
    }
    catch (const std::runtime_error & ex) {

      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Invalid PointCloud2 fields: %s",
        ex.what());
    }
  }


  void publishMap()
  {
    std::vector<std::array<float, 3>> points;

    {
      std::lock_guard<std::mutex> lock(map_mutex_);

      points.reserve(voxels_.size());

      for (const auto & item : voxels_) {

        const auto & cell = item.second;

        points.push_back({
          static_cast<float>(cell.x),
          static_cast<float>(cell.y),
          static_cast<float>(cell.z)
        });
      }
    }

    sensor_msgs::msg::PointCloud2 output;

    output.header.stamp =
      this->get_clock()->now();

    output.header.frame_id =
      target_frame_;

    sensor_msgs::PointCloud2Modifier modifier(output);

    modifier.setPointCloud2FieldsByString(
      1,
      "xyz");

    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float>
      iter_x(output, "x");

    sensor_msgs::PointCloud2Iterator<float>
      iter_y(output, "y");

    sensor_msgs::PointCloud2Iterator<float>
      iter_z(output, "z");

    for (const auto & point : points) {

      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    output.height = 1;
    output.width =
      static_cast<uint32_t>(points.size());

    output.is_dense = true;

    map_pub_->publish(output);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Radar map: %zu voxels, dropped new voxels: %lu",
      points.size(),
      dropped_new_voxels_);
  }


  std::string target_frame_;
  std::string left_topic_;
  std::string right_topic_;
  std::string map_topic_;

  double voxel_size_;
  int64_t max_voxels_;
  double publish_rate_hz_;
  double tf_timeout_sec_;

  std::unique_ptr<tf2_ros::Buffer>
    tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener>
    tf_listener_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    left_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    right_sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    map_pub_;

  rclcpp::TimerBase::SharedPtr
    publish_timer_;

  std::unordered_map<
    VoxelKey,
    VoxelCell,
    VoxelKeyHash>
    voxels_;

  std::mutex map_mutex_;

  uint64_t dropped_new_voxels_ = 0;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<RadarMapBuilder>());

  rclcpp::shutdown();

  return 0;
}
