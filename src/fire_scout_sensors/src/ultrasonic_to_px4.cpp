#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "px4_msgs/msg/distance_sensor.hpp"


using std::placeholders::_1;


class UltrasonicToPX4 : public rclcpp::Node
{

public:

    UltrasonicToPX4()
    : Node("ultrasonic_to_px4")
    {

        input_topic_ =
            declare_parameter<std::string>(
                "input_topic",
                "/scout1/ultrasonic/front/scan");


        output_topic_ =
            declare_parameter<std::string>(
                "output_topic",
                "/scout1/fmu/in/distance_sensor");


        range_min_ =
            declare_parameter<double>(
                "min_distance",
                0.02);


        range_max_ =
            declare_parameter<double>(
                "max_distance",
                5.0);



        sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                input_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &UltrasonicToPX4::scanCallback,
                    this,
                    _1));


        pub_ =
        create_publisher<px4_msgs::msg::DistanceSensor>(
            output_topic_,
            rclcpp::SensorDataQoS());


        RCLCPP_INFO(
            get_logger(),
            "Ultrasonic to PX4 started");


        RCLCPP_INFO(
            get_logger(),
            "Input : %s",
            input_topic_.c_str());


        RCLCPP_INFO(
            get_logger(),
            "Output: %s",
            output_topic_.c_str());

    }



private:


    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {


        if(msg->ranges.empty())
            return;



        /*
          超声波只有一个方向

          LaserScan可能有多个点

          这里取最近距离
        */


        float distance =
            *std::min_element(
                msg->ranges.begin(),
                msg->ranges.end());


        if(!std::isfinite(distance))
            return;



        distance =
            std::clamp(
                distance,
                (float)range_min_,
                (float)range_max_);



        px4_msgs::msg::DistanceSensor out;


        out.timestamp =
            this->get_clock()
            ->now()
            .nanoseconds()
            /
            1000;



        /*
          PX4单位:
          mm

          ROS LaserScan:
          m
        */


        out.current_distance =
            distance * 1000.0f;



        out.min_distance =
            range_min_ * 1000.0f;


        out.max_distance =
            range_max_ * 1000.0f;



        /*
          FRONT方向

          PX4 MAV_SENSOR_ORIENTATION:
          ROTATION_NONE = 0

          前向安装
        */

        out.orientation = 0;



        /*
          MAV_DISTANCE_SENSOR_ULTRASOUND

          PX4 enum:
          0 = LASER
          1 = ULTRASOUND
        */

        out.type = 1;



        out.variance = 0.02f;



        pub_->publish(out);

    }



private:


    std::string input_topic_;
    std::string output_topic_;


    double range_min_;
    double range_max_;


    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan>::SharedPtr sub_;


    rclcpp::Publisher<
        px4_msgs::msg::DistanceSensor>::SharedPtr pub_;

};



int main(
    int argc,
    char ** argv)
{

    rclcpp::init(argc, argv);


    auto node =
        std::make_shared<UltrasonicToPX4>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}
