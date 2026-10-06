#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class ScoutEnterWindow : public rclcpp::Node
{
public:
  ScoutEnterWindow()
  : Node("scout_enter_window")
  {
    px4_prefix_ =
      declare_parameter<std::string>("px4_prefix", "/scout1");

    target_system_ =
      declare_parameter<int>("target_system", 1);

    takeoff_height_ =
      declare_parameter<double>("takeoff_height", 1.5);

    horizontal_speed_ =
      declare_parameter<double>("horizontal_speed", 0.35);

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

    // PX4 NED coordinates.
    // Start directly above the spawn position.
    cmd_x_ = 0.0f;
    cmd_y_ = 0.0f;
    cmd_z_ = static_cast<float>(-takeoff_height_);

    timer_ =
      create_wall_timer(
        100ms,
        std::bind(&ScoutEnterWindow::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Scout window-entry mission started");
    RCLCPP_INFO(get_logger(), "PX4 prefix: %s", px4_prefix_.c_str());
    RCLCPP_INFO(get_logger(), "Target system: %d", target_system_);
    RCLCPP_INFO(
      get_logger(),
      "Horizontal speed: %.2f m/s",
      horizontal_speed_);

    RCLCPP_INFO(
      get_logger(),
      "Mission: takeoff -> pre-window -> cross window -> indoor hover");
  }

private:
  enum class State
  {
    PRESTREAM,
    TAKEOFF,
    PRE_WINDOW,
    PRE_WINDOW_HOLD,
    CROSS_WINDOW,
    INDOOR,
    HOLD
  };

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

    msg.position = {
      cmd_x_,
      cmd_y_,
      cmd_z_
    };

    /*
     * PX4 NED yaw:
     *
     * 0 rad     = North
     * pi/2 rad  = East
     *
     * Window A is east of the spawn position,
     * so make the Scout face toward the window.
     */
    msg.yaw = 1.5707963f;

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
  }

  void engageOffboardMode()
  {
    publishVehicleCommand(
      px4_msgs::msg::VehicleCommand::
      VEHICLE_CMD_DO_SET_MODE,
      1.0f,
      6.0f);
  }

  bool moveToward(
    float target_x,
    float target_y,
    float target_z)
  {
    const float dx = target_x - cmd_x_;
    const float dy = target_y - cmd_y_;
    const float dz = target_z - cmd_z_;

    const float distance =
      std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 0.01f) {
      cmd_x_ = target_x;
      cmd_y_ = target_y;
      cmd_z_ = target_z;
      return true;
    }

    // 100 ms timer
    const float dt = 0.1f;

    const float step =
      static_cast<float>(horizontal_speed_) * dt;

    if (distance <= step) {
      cmd_x_ = target_x;
      cmd_y_ = target_y;
      cmd_z_ = target_z;
      return true;
    }

    cmd_x_ += dx / distance * step;
    cmd_y_ += dy / distance * step;
    cmd_z_ += dz / distance * step;

    return false;
  }

  void changeState(
    State new_state,
    const std::string & description)
  {
    state_ = new_state;
    state_ticks_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "Mission state -> %s",
      description.c_str());
  }

  void timerCallback()
  {
    // Required continuously while using PX4 Offboard.
    publishOffboardControlMode();
    publishTrajectorySetpoint();

    ++state_ticks_;

    switch (state_) {

      case State::PRESTREAM:
      {
        /*
         * Send valid OffboardControlMode + setpoints
         * for 2 seconds before requesting Offboard.
         */
        if (state_ticks_ >= 20) {

          engageOffboardMode();
          arm();

          RCLCPP_INFO(
            get_logger(),
            "OFFBOARD + ARM requested");

          changeState(
            State::TAKEOFF,
            "TAKEOFF");
        }

        break;
      }


      case State::TAKEOFF:
      {
        /*
         * Hold:
         * NED x = 0
         * NED y = 0
         * NED z = -1.5
         *
         * Give the vehicle 6 seconds to rise and stabilize.
         */
        if (state_ticks_ >= 60) {

          changeState(
            State::PRE_WINDOW,
            "APPROACH_PRE_WINDOW");
        }

        break;
      }


      case State::PRE_WINDOW:
      {
        /*
         * Spawn world x = -12.5
         *
         * Target world x ≈ -11.0
         *
         * Delta East = +1.5 m
         *
         * Gazebo East -> PX4 NED y.
         */
        const bool reached =
          moveToward(
            0.0f,
            1.5f,
            static_cast<float>(-takeoff_height_));

        if (reached) {
          changeState(
            State::PRE_WINDOW_HOLD,
            "PRE_WINDOW_HOLD");
        }

        break;
      }


      case State::PRE_WINDOW_HOLD:
      {
        // Stabilize about 1 m in front of the wall.
        if (state_ticks_ >= 20) {

          changeState(
            State::CROSS_WINDOW,
            "CROSS_WINDOW");
        }

        break;
      }


      case State::CROSS_WINDOW:
      {
        /*
         * Move continuously through the opening.
         *
         * NED y = 3.5 corresponds roughly to
         * Gazebo world x = -9.0.
         */
        const bool reached =
          moveToward(
            0.0f,
            3.5f,
            static_cast<float>(-takeoff_height_));

        if (reached) {

          changeState(
            State::INDOOR,
            "MOVE_INDOOR");
        }

        break;
      }


      case State::INDOOR:
      {
        /*
         * Final indoor point:
         *
         * Gazebo world x ≈ -8.0
         * world y ≈ 0
         */
        const bool reached =
          moveToward(
            0.0f,
            4.5f,
            static_cast<float>(-takeoff_height_));

        if (reached) {

          changeState(
            State::HOLD,
            "INDOOR_HOLD");
        }

        break;
      }


      case State::HOLD:
      {
        // Continue publishing the final setpoint forever.
        break;
      }
    }
  }

private:
  std::string px4_prefix_;

  int target_system_{1};

  double takeoff_height_{1.5};
  double horizontal_speed_{0.35};

  float cmd_x_{0.0f};
  float cmd_y_{0.0f};
  float cmd_z_{-1.5f};

  State state_{State::PRESTREAM};

  int state_ticks_{0};

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
    std::make_shared<ScoutEnterWindow>());

  rclcpp::shutdown();

  return 0;
}