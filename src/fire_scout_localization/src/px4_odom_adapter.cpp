#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

class Px4OdomAdapter : public rclcpp::Node
{
public:
  Px4OdomAdapter()
  : Node("px4_odom_adapter")
  {
    declare_parameter<std::string>("runtime_version", "0.2.0-frd-tf-recovery");

    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/scout1/fmu/out/vehicle_odometry");

    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/scout1/odom");

    odom_frame_ = declare_parameter<std::string>(
      "odom_frame", "scout1/odom");

    base_frame_ = declare_parameter<std::string>(
      "base_frame", "scout1/base_link");

    accept_frd_pose_frame_ = declare_parameter<bool>(
      "accept_frd_pose_frame", true);

    input_timeout_ = declare_parameter<double>(
      "input_timeout", 2.0);

    diagnostic_period_ = declare_parameter<double>(
      "diagnostic_period", 1.0);

    if (input_topic_.empty() || output_topic_.empty() || odom_frame_.empty() ||
        base_frame_.empty() || !std::isfinite(input_timeout_) || input_timeout_ <= 0.0 ||
        !std::isfinite(diagnostic_period_) || diagnostic_period_ < 0.2 ||
        diagnostic_period_ > 10.0) {
      throw std::runtime_error("Invalid PX4 odometry adapter parameters");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(20).reliable());

    px4_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Px4OdomAdapter::odomCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    diagnostic_timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int64_t>(std::llround(diagnostic_period_ * 1000.0))),
      std::bind(&Px4OdomAdapter::diagnosticTick, this));

    RCLCPP_INFO(get_logger(), "PX4 EKF odometry adapter started");
    RCLCPP_INFO(get_logger(), "Input : %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "TF: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(
      get_logger(), "Accepted PX4 pose frames: NED%s",
      accept_frd_pose_frame_ ? " and FRD" : " only");
  }

private:
  static tf2::Vector3 nedToEnu(const tf2::Vector3 & v)
  {
    return tf2::Vector3(v.y(), v.x(), -v.z());
  }

  static tf2::Vector3 frdToFlu(const tf2::Vector3 & v)
  {
    return tf2::Vector3(v.x(), -v.y(), -v.z());
  }

  static tf2::Quaternion px4QuaternionToRos(
    const px4_msgs::msg::VehicleOdometry & msg,
    bool pose_is_ned)
  {
    tf2::Quaternion q_px4(
      msg.q[1], msg.q[2], msg.q[3], msg.q[0]);

    tf2::Quaternion q_flu_to_frd;
    q_flu_to_frd.setRPY(M_PI, 0.0, 0.0);

    tf2::Quaternion q_world_to_ros;
    if (pose_is_ned) {
      q_world_to_ros.setRPY(M_PI, 0.0, M_PI_2);
    } else {
      // PX4's fixed FRD navigation frame becomes a fixed FLU ROS frame.
      q_world_to_ros.setRPY(M_PI, 0.0, 0.0);
    }

    tf2::Quaternion q_ros = q_world_to_ros * q_px4 * q_flu_to_frd;

    q_ros.normalize();
    return q_ros;
  }

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    ++received_messages_;
    last_input_wall_ = std::chrono::steady_clock::now();
    last_pose_frame_ = msg->pose_frame;
    pending_msg_ = msg;
    publishMessage(*msg);
  }

  bool publishMessage(const px4_msgs::msg::VehicleOdometry & msg)
  {
    const bool pose_is_ned =
      msg.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    const bool pose_is_frd =
      msg.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
    if (!pose_is_ned && !(pose_is_frd && accept_frd_pose_frame_)) {
      ++rejected_frame_messages_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "VehicleOdometry pose_frame=%u is unsupported (NED=%u FRD=%u); "
        "odometry and TF are not being published.",
        static_cast<unsigned>(msg.pose_frame),
        static_cast<unsigned>(px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED),
        static_cast<unsigned>(px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD));
      return false;
    }
    if (pose_is_frd && !frd_notice_printed_) {
      RCLCPP_WARN(
        get_logger(),
        "PX4 is publishing POSE_FRAME_FRD. It is now converted to a ROS FLU local "
        "odom frame. For multi-UAV world fusion, verify each world->odom yaw against "
        "the vehicle's initial heading.");
      frd_notice_printed_ = true;
    }

    const double quaternion_norm_squared =
      static_cast<double>(msg.q[0]) * msg.q[0] +
      static_cast<double>(msg.q[1]) * msg.q[1] +
      static_cast<double>(msg.q[2]) * msg.q[2] +
      static_cast<double>(msg.q[3]) * msg.q[3];
    if (!std::isfinite(msg.position[0]) ||
        !std::isfinite(msg.position[1]) ||
        !std::isfinite(msg.position[2]) ||
        !std::isfinite(msg.q[0]) || !std::isfinite(msg.q[1]) ||
        !std::isfinite(msg.q[2]) || !std::isfinite(msg.q[3]) ||
        !std::isfinite(quaternion_norm_squared) || quaternion_norm_squared < 1e-12) {
      ++rejected_invalid_messages_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "VehicleOdometry contains an invalid position or quaternion; message ignored.");
      return false;
    }

    const auto stamp = this->get_clock()->now();
    if (stamp.nanoseconds() == 0) {
      ++clock_wait_messages_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 2000,
        "ROS simulation clock is still zero. Waiting for /clock before publishing %s -> %s.",
        odom_frame_.c_str(), base_frame_.c_str());
      return false;
    }

    if (last_published_source_timestamp_ != 0 &&
        msg.timestamp + 1000000ULL < last_published_source_timestamp_) {
      RCLCPP_WARN(
        get_logger(), "PX4 timestamp reset detected (%llu -> %llu); accepting new epoch.",
        static_cast<unsigned long long>(last_published_source_timestamp_),
        static_cast<unsigned long long>(msg.timestamp));
      last_published_source_timestamp_ = 0;
    }
    if (msg.timestamp != 0 && msg.timestamp == last_published_source_timestamp_) {
      return false;
    }

    const tf2::Vector3 p_px4(
      msg.position[0], msg.position[1], msg.position[2]);
    const tf2::Vector3 p_ros = pose_is_ned ? nedToEnu(p_px4) : frdToFlu(p_px4);
    const tf2::Quaternion q_ros = px4QuaternionToRos(msg, pose_is_ned);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = p_ros.x();
    odom.pose.pose.position.y = p_ros.y();
    odom.pose.pose.position.z = p_ros.z();
    odom.pose.pose.orientation.x = q_ros.x();
    odom.pose.pose.orientation.y = q_ros.y();
    odom.pose.pose.orientation.z = q_ros.z();
    odom.pose.pose.orientation.w = q_ros.w();

    odom.pose.covariance[0] =
      std::isfinite(msg.position_variance[pose_is_ned ? 1 : 0]) ?
      msg.position_variance[pose_is_ned ? 1 : 0] : 0.05;
    odom.pose.covariance[7] =
      std::isfinite(msg.position_variance[pose_is_ned ? 0 : 1]) ?
      msg.position_variance[pose_is_ned ? 0 : 1] : 0.05;
    odom.pose.covariance[14] =
      std::isfinite(msg.position_variance[2]) ? msg.position_variance[2] : 0.05;
    odom.pose.covariance[21] =
      std::isfinite(msg.orientation_variance[0]) ? msg.orientation_variance[0] : 0.02;
    odom.pose.covariance[28] =
      std::isfinite(msg.orientation_variance[1]) ? msg.orientation_variance[1] : 0.02;
    odom.pose.covariance[35] =
      std::isfinite(msg.orientation_variance[2]) ? msg.orientation_variance[2] : 0.02;

    tf2::Vector3 velocity_body_flu(0.0, 0.0, 0.0);

    const bool finite_velocity =
      std::isfinite(msg.velocity[0]) && std::isfinite(msg.velocity[1]) &&
      std::isfinite(msg.velocity[2]);
    if (finite_velocity && msg.velocity_frame ==
        px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      const tf2::Vector3 velocity_frd(
        msg.velocity[0], msg.velocity[1], msg.velocity[2]);
      velocity_body_flu = frdToFlu(velocity_frd);
    } else if (finite_velocity && pose_is_ned && msg.velocity_frame ==
               px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
      const tf2::Vector3 velocity_world_ros = nedToEnu(
        tf2::Vector3(msg.velocity[0], msg.velocity[1], msg.velocity[2]));
      const tf2::Matrix3x3 rotation(q_ros.inverse());
      velocity_body_flu = rotation * velocity_world_ros;
    } else if (finite_velocity && pose_is_frd && msg.velocity_frame ==
               px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD) {
      const tf2::Vector3 velocity_world_ros = frdToFlu(
        tf2::Vector3(msg.velocity[0], msg.velocity[1], msg.velocity[2]));
      const tf2::Matrix3x3 rotation(q_ros.inverse());
      velocity_body_flu = rotation * velocity_world_ros;
    }

    odom.twist.twist.linear.x = velocity_body_flu.x();
    odom.twist.twist.linear.y = velocity_body_flu.y();
    odom.twist.twist.linear.z = velocity_body_flu.z();

    const tf2::Vector3 angular_frd(
      msg.angular_velocity[0],
      msg.angular_velocity[1],
      msg.angular_velocity[2]);
    const tf2::Vector3 angular_flu =
      std::isfinite(msg.angular_velocity[0]) &&
      std::isfinite(msg.angular_velocity[1]) &&
      std::isfinite(msg.angular_velocity[2]) ?
      frdToFlu(angular_frd) : tf2::Vector3(0.0, 0.0, 0.0);

    odom.twist.twist.angular.x = angular_flu.x();
    odom.twist.twist.angular.y = angular_flu.y();
    odom.twist.twist.angular.z = angular_flu.z();

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = p_ros.x();
    tf.transform.translation.y = p_ros.y();
    tf.transform.translation.z = p_ros.z();
    tf.transform.rotation.x = q_ros.x();
    tf.transform.rotation.y = q_ros.y();
    tf.transform.rotation.z = q_ros.z();
    tf.transform.rotation.w = q_ros.w();

    tf_broadcaster_->sendTransform(tf);
    ++published_messages_;
    last_publish_wall_ = std::chrono::steady_clock::now();
    last_published_source_timestamp_ = msg.timestamp;
    last_published_pose_frame_ = msg.pose_frame;
    return true;
  }

  void diagnosticTick()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    const auto ros_now = get_clock()->now();
    if (ros_now.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 3000,
        "TF unavailable: /clock has not advanced (use_sim_time is enabled). "
        "Check the scout1 Gazebo bridge and GZ_PARTITION.");
      return;
    }

    if (received_messages_ == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 3000,
        "TF unavailable: no PX4 VehicleOdometry received on %s. Check the PX4 DDS "
        "namespace, Micro XRCE agent, ROS_DOMAIN_ID and message version.",
        input_topic_.c_str());
      return;
    }

    const double input_age =
      std::chrono::duration<double>(wall_now - last_input_wall_).count();
    if (input_age > input_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 3000,
        "TF stale: %s has received no VehicleOdometry for %.2f s.",
        input_topic_.c_str(), input_age);
      return;
    }

    if (pending_msg_ && pending_msg_->timestamp != last_published_source_timestamp_) {
      publishMessage(*pending_msg_);
    }

    if (published_messages_ == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 3000,
        "TF unavailable despite PX4 input: last_pose_frame=%u rejected_frame=%llu "
        "rejected_invalid=%llu clock_wait=%llu.",
        static_cast<unsigned>(last_pose_frame_),
        static_cast<unsigned long long>(rejected_frame_messages_),
        static_cast<unsigned long long>(rejected_invalid_messages_),
        static_cast<unsigned long long>(clock_wait_messages_));
      return;
    }

    const double publish_age =
      std::chrono::duration<double>(wall_now - last_publish_wall_).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, 5000,
      "ODOM_TF_OK input=%s frame=%u received=%llu published=%llu age=%.3fs tf=%s->%s",
      input_topic_.c_str(), static_cast<unsigned>(last_published_pose_frame_),
      static_cast<unsigned long long>(received_messages_),
      static_cast<unsigned long long>(published_messages_), publish_age,
      odom_frame_.c_str(), base_frame_.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool accept_frd_pose_frame_{true};
  bool frd_notice_printed_{false};
  double input_timeout_{2.0};
  double diagnostic_period_{1.0};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  uint64_t received_messages_{0};
  uint64_t published_messages_{0};
  uint64_t rejected_frame_messages_{0};
  uint64_t rejected_invalid_messages_{0};
  uint64_t clock_wait_messages_{0};
  uint64_t last_published_source_timestamp_{0};
  uint8_t last_pose_frame_{0};
  uint8_t last_published_pose_frame_{0};
  std::chrono::steady_clock::time_point last_input_wall_{};
  std::chrono::steady_clock::time_point last_publish_wall_{};
  px4_msgs::msg::VehicleOdometry::SharedPtr pending_msg_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdomAdapter>());
  rclcpp::shutdown();
  return 0;
}
