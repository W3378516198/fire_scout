#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class ScoutTakeoff : public rclcpp::Node
{
public:
  ScoutTakeoff()
  : Node("scout_takeoff")
  {
    px4_prefix_ =
      declare_parameter<std::string>(
        "px4_prefix",
        "/scout1");

    target_system_ =
      declare_parameter<int>(
        "target_system",
        1);

    takeoff_height_ =
      declare_parameter<double>(
        "takeoff_height",
        1.5);

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>(
        px4_prefix_ + "/fmu/in/offboard_control_mode",
        10);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        px4_prefix_ + "/fmu/in/trajectory_setpoint",
        10);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>(
        px4_prefix_ + "/fmu/in/vehicle_command",
        10);

    timer_ =
      create_wall_timer(
        100ms,
        std::bind(
          &ScoutTakeoff::timerCallback,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Scout takeoff node started");

    RCLCPP_INFO(
      get_logger(),
      "PX4 prefix: %s",
      px4_prefix_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Target system: %d",
      target_system_);

    RCLCPP_INFO(
      get_logger(),
      "Takeoff height: %.2f m",
      takeoff_height_);
  }

private:
  uint64_t timestampUs() 
  {
    return static_cast<uint64_t>(
      get_clock()->now().nanoseconds() / 1000ULL);
  }


  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = timestampUs();

    msg.position = true;

    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub_->publish(msg);
  }


  void publishTrajectorySetpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = timestampUs();

    /*
     * PX4 local coordinates are NED.
     *
     * x = North
     * y = East
     * z = Down
     *
     * Therefore:
     *
     * z = -1.5
     *
     * means 1.5 m above the takeoff origin.
     */

    msg.position = {
      0.0f,
      0.0f,
      static_cast<float>(-takeoff_height_)
    };

    msg.yaw = 0.0f;

    trajectory_setpoint_pub_->publish(msg);
  }


  void publishVehicleCommand(
    uint16_t command,
    float param1 = 0.0f,
    float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = timestampUs();

    msg.param1 = param1;
    msg.param2 = param2;

    msg.command = command;

    msg.target_system =
      static_cast<uint8_t>(target_system_);

    msg.target_component = 1;

    msg.source_system =
      static_cast<uint8_t>(target_system_);

    msg.source_component = 1;

    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
  }


  void arm()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0f);

    RCLCPP_INFO(
      get_logger(),
      "ARM command sent");
  }


  void engageOffboardMode()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_DO_SET_MODE,
      1.0f,
      6.0f);

    RCLCPP_INFO(
      get_logger(),
      "OFFBOARD command sent");
  }


  void timerCallback()
  {
    /*
     * PX4 requires continuous OffboardControlMode
     * messages before entering Offboard.
     */

    publishOffboardControlMode();

    publishTrajectorySetpoint();

    if (counter_ == 10) {

      engageOffboardMode();

      arm();
    }

    if (counter_ < 11) {
      counter_++;
    }
  }


private:
  std::string px4_prefix_;

  int target_system_;

  double takeoff_height_;

  int counter_{0};

  rclcpp::Publisher<
    px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_pub_;

  rclcpp::Publisher<
    px4_msgs::msg::VehicleCommand>::SharedPtr
    vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<ScoutTakeoff>());

  rclcpp::shutdown();

  return 0;
}