#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


class MmWaveEnricher : public rclcpp::Node
{
public:
  MmWaveEnricher()
  : Node("mmwave_enricher"),
    rng_(std::random_device{}())
  {
    input_topic_ =
      declare_parameter<std::string>(
        "input_topic",
        "/scout1/radar/front/raw_points");

    odom_topic_ =
      declare_parameter<std::string>(
        "odom_topic",
        "/scout1/sim/ground_truth_odom");

    output_topic_ =
      declare_parameter<std::string>(
        "output_topic",
        "/scout1/radar/front/mmwave_points");

    output_frame_ =
      declare_parameter<std::string>(
        "output_frame",
        "scout1/radar_link");


    /*
     * Radar extrinsic:
     *
     * Translation of Radar origin expressed in base_link.
     */
    radar_x_ =
      declare_parameter<double>(
        "radar_x",
        0.15);

    radar_y_ =
      declare_parameter<double>(
        "radar_y",
        0.0);

    radar_z_ =
      declare_parameter<double>(
        "radar_z",
        0.12);

    /*
     * Radar yaw relative to base_link.
     *
     * Front radar = 0
     * Rear radar  = pi
     */
    radar_yaw_ =
      declare_parameter<double>(
        "radar_yaw",
        0.0);


    /*
     * Maximum allowed time difference between
     * Radar measurement and ground-truth odometry.
     */
    max_odom_time_diff_ =
      declare_parameter<double>(
        "max_odom_time_diff",
        0.03);


    /*
     * Doppler measurement noise.
     *
     * First-stage simulation:
     * sigma = 0.05 m/s
     */
    doppler_noise_std_ =
      declare_parameter<double>(
        "doppler_noise_std",
        0.05);


    /*
     * Approximate SNR model.
     *
     * This is NOT an electromagnetic radar simulation.
     */
    snr_at_1m_db_ =
      declare_parameter<double>(
        "snr_at_1m_db",
        35.0);

    snr_noise_std_db_ =
      declare_parameter<double>(
        "snr_noise_std_db",
        1.5);

    min_snr_db_ =
      declare_parameter<double>(
        "min_snr_db",
        5.0);

    max_snr_db_ =
      declare_parameter<double>(
        "max_snr_db",
        40.0);


    /*
     * Pseudo-RCS model.
     *
     * Placeholder until the real radar model / hardware
     * provides an actual reflectivity / RCS-like quantity.
     */
    rcs_mean_dbsm_ =
      declare_parameter<double>(
        "rcs_mean_dbsm",
        0.0);

    rcs_std_db_ =
      declare_parameter<double>(
        "rcs_std_db",
        3.0);


    /*
     * Ground-truth odometry:
     * 100 Hz in current simulation.
     *
     * Keep a short time history so each 15 Hz Radar
     * frame can use the nearest odometry measurement.
     */
    auto odom_qos =
      rclcpp::QoS(
        rclcpp::KeepLast(300));

    odom_qos.best_effort();


    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        odom_qos,
        std::bind(
          &MmWaveEnricher::odomCallback,
          this,
          std::placeholders::_1));


    /*
     * Radar point cloud uses sensor-data QoS.
     */
    cloud_sub_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &MmWaveEnricher::cloudCallback,
          this,
          std::placeholders::_1));


    output_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_,
        rclcpp::SensorDataQoS());


    RCLCPP_INFO(
      get_logger(),
      "mmWave enricher started");

    RCLCPP_INFO(
      get_logger(),
      "Input : %s",
      input_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Odom  : %s",
      odom_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Output: %s",
      output_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Extrinsic xyz=(%.3f %.3f %.3f), yaw=%.3f rad",
      radar_x_,
      radar_y_,
      radar_z_,
      radar_yaw_);
  }


private:

  struct Vec3
  {
    double x;
    double y;
    double z;
  };


  struct MmWavePoint
  {
    float x;
    float y;
    float z;

    float range;
    float azimuth;
    float elevation;

    float doppler;
    float snr;
    float rcs;
  };


  static double stampToSeconds(
    const builtin_interfaces::msg::Time & stamp)
  {
    return
      static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) *
      1e-9;
  }


  static Vec3 add(
    const Vec3 & a,
    const Vec3 & b)
  {
    return {
      a.x + b.x,
      a.y + b.y,
      a.z + b.z
    };
  }


  static Vec3 cross(
    const Vec3 & a,
    const Vec3 & b)
  {
    return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x
    };
  }


  /*
   * Convert a vector expressed in base_link
   * into the Radar sensor coordinate frame.
   *
   * Radar pose relative to body only contains yaw
   * in the current hardware layout.
   */
  Vec3 bodyToRadar(
    const Vec3 & v) const
  {
    const double c =
      std::cos(radar_yaw_);

    const double s =
      std::sin(radar_yaw_);


    /*
     * R_radar_body = R_body_radar^T
     */
    return {
      c * v.x + s * v.y,
      -s * v.x + c * v.y,
      v.z
    };
  }


  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_buffer_.push_back(msg);

    /*
     * At 100 Hz, 300 messages ~= 3 seconds.
     */
    while (odom_buffer_.size() > 300) {
      odom_buffer_.pop_front();
    }
  }


  nav_msgs::msg::Odometry::SharedPtr
  findNearestOdom(
    const builtin_interfaces::msg::Time & stamp,
    double & best_dt)
  {
    if (odom_buffer_.empty()) {
      return nullptr;
    }


    const double cloud_time =
      stampToSeconds(stamp);


    nav_msgs::msg::Odometry::SharedPtr
      best_msg = nullptr;

    best_dt =
      std::numeric_limits<double>::max();


    for (const auto & odom : odom_buffer_)
    {
      const double odom_time =
        stampToSeconds(
          odom->header.stamp);

      const double dt =
        std::abs(
          odom_time -
          cloud_time);

      if (dt < best_dt)
      {
        best_dt = dt;
        best_msg = odom;
      }
    }


    if (best_dt > max_odom_time_diff_) {
      return nullptr;
    }


    return best_msg;
  }


  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    double odom_dt = 0.0;

    auto odom =
      findNearestOdom(
        cloud->header.stamp,
        odom_dt);


    if (!odom)
    {
      ++missing_odom_count_;

      if (
        missing_odom_count_ % 30 == 1)
      {
        RCLCPP_WARN(
          get_logger(),
          "No synchronized ground-truth odometry for Radar frame");
      }

      return;
    }


    missing_odom_count_ = 0;


    /*
     * According to nav_msgs/Odometry convention,
     * twist is expressed in child_frame_id.
     *
     * Here:
     * child_frame_id = base_link
     *
     * Therefore these are body-frame velocities.
     */
    const Vec3 v_base{
      odom->twist.twist.linear.x,
      odom->twist.twist.linear.y,
      odom->twist.twist.linear.z
    };


    const Vec3 omega_base{
      odom->twist.twist.angular.x,
      odom->twist.twist.angular.y,
      odom->twist.twist.angular.z
    };


    /*
     * Radar lever arm expressed in base_link.
     */
    const Vec3 r_base_radar{
      radar_x_,
      radar_y_,
      radar_z_
    };


    /*
     * Velocity of Radar sensor origin:
     *
     * v_radar =
     * v_base + omega x r
     */
    const Vec3 rotational_velocity =
      cross(
        omega_base,
        r_base_radar);


    const Vec3 v_radar_body =
      add(
        v_base,
        rotational_velocity);


    /*
     * Express Radar velocity in its own frame,
     * because point coordinates are in Radar frame.
     */
    const Vec3 v_radar =
      bodyToRadar(
        v_radar_body);


    std::normal_distribution<double>
      doppler_noise(
        0.0,
        doppler_noise_std_);

    std::normal_distribution<double>
      snr_noise(
        0.0,
        snr_noise_std_db_);

    std::normal_distribution<double>
      rcs_noise(
        rcs_mean_dbsm_,
        rcs_std_db_);


    std::vector<MmWavePoint>
      points;

    points.reserve(
      static_cast<std::size_t>(
        cloud->width) *
      static_cast<std::size_t>(
        cloud->height));


    try
    {
      sensor_msgs::PointCloud2ConstIterator<float>
        iter_x(*cloud, "x");

      sensor_msgs::PointCloud2ConstIterator<float>
        iter_y(*cloud, "y");

      sensor_msgs::PointCloud2ConstIterator<float>
        iter_z(*cloud, "z");


      for (;
           iter_x != iter_x.end();
           ++iter_x,
           ++iter_y,
           ++iter_z)
      {
        const double x = *iter_x;
        const double y = *iter_y;
        const double z = *iter_z;


        if (
          !std::isfinite(x) ||
          !std::isfinite(y) ||
          !std::isfinite(z))
        {
          continue;
        }


        const double horizontal_range =
          std::sqrt(
            x * x +
            y * y);


        const double range =
          std::sqrt(
            x * x +
            y * y +
            z * z);


        if (range < 1e-4) {
          continue;
        }


        const double azimuth =
          std::atan2(
            y,
            x);

        const double elevation =
          std::atan2(
            z,
            horizontal_range);


        /*
         * Unit line-of-sight vector.
         */
        const double ux =
          x / range;

        const double uy =
          y / range;

        const double uz =
          z / range;


        /*
         * Static-world Doppler model.
         *
         * Positive / negative sign convention:
         *
         * v_d = -LOS dot sensor_velocity
         *
         * A stationary reflector appears to move
         * opposite to the Radar sensor velocity.
         */
        double doppler =
          -(
            ux * v_radar.x +
            uy * v_radar.y +
            uz * v_radar.z);


        doppler +=
          doppler_noise(rng_);


        /*
         * Approximate SNR model.
         *
         * Not electromagnetic truth.
         * Only provides a first-stage quality field.
         */
        const double safe_range =
          std::max(
            range,
            1.0);

        double snr =
          snr_at_1m_db_
          - 20.0 *
            std::log10(
              safe_range)
          + snr_noise(rng_);


        snr =
          std::max(
            min_snr_db_,
            std::min(
              max_snr_db_,
              snr));


        /*
         * Placeholder pseudo-RCS.
         */
        const double rcs =
          rcs_noise(rng_);


        MmWavePoint p;

        p.x =
          static_cast<float>(x);

        p.y =
          static_cast<float>(y);

        p.z =
          static_cast<float>(z);

        p.range =
          static_cast<float>(range);

        p.azimuth =
          static_cast<float>(azimuth);

        p.elevation =
          static_cast<float>(elevation);

        p.doppler =
          static_cast<float>(doppler);

        p.snr =
          static_cast<float>(snr);

        p.rcs =
          static_cast<float>(rcs);


        points.push_back(p);
      }
    }
    catch (const std::runtime_error & e)
    {
      RCLCPP_ERROR(
        get_logger(),
        "PointCloud2 field error: %s",
        e.what());

      return;
    }


    sensor_msgs::msg::PointCloud2 output;

    /*
     * Preserve actual Radar measurement timestamp.
     */
    output.header.stamp =
      cloud->header.stamp;

    output.header.frame_id =
      output_frame_;


    sensor_msgs::PointCloud2Modifier
      modifier(output);


    modifier.setPointCloud2Fields(
      9,

      "x",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "y",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "z",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "range",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "azimuth",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "elevation",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "doppler",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "snr",
      1,
      sensor_msgs::msg::PointField::FLOAT32,

      "rcs",
      1,
      sensor_msgs::msg::PointField::FLOAT32);


    modifier.resize(
      points.size());


    sensor_msgs::PointCloud2Iterator<float>
      out_x(output, "x");

    sensor_msgs::PointCloud2Iterator<float>
      out_y(output, "y");

    sensor_msgs::PointCloud2Iterator<float>
      out_z(output, "z");

    sensor_msgs::PointCloud2Iterator<float>
      out_range(output, "range");

    sensor_msgs::PointCloud2Iterator<float>
      out_azimuth(output, "azimuth");

    sensor_msgs::PointCloud2Iterator<float>
      out_elevation(output, "elevation");

    sensor_msgs::PointCloud2Iterator<float>
      out_doppler(output, "doppler");

    sensor_msgs::PointCloud2Iterator<float>
      out_snr(output, "snr");

    sensor_msgs::PointCloud2Iterator<float>
      out_rcs(output, "rcs");


    double sum_doppler = 0.0;
    double sum_abs_doppler = 0.0;
    double sum_snr = 0.0;

    double min_doppler =
      std::numeric_limits<double>::max();

    double max_doppler =
      -std::numeric_limits<double>::max();


    for (const auto & p : points)
    {
      *out_x = p.x;
      *out_y = p.y;
      *out_z = p.z;

      *out_range = p.range;
      *out_azimuth = p.azimuth;
      *out_elevation = p.elevation;

      *out_doppler = p.doppler;
      *out_snr = p.snr;
      *out_rcs = p.rcs;


      sum_doppler +=
        p.doppler;

      sum_abs_doppler +=
        std::abs(
          p.doppler);

      sum_snr +=
        p.snr;

      min_doppler =
        std::min(
          min_doppler,
          static_cast<double>(
            p.doppler));

      max_doppler =
        std::max(
          max_doppler,
          static_cast<double>(
            p.doppler));


      ++out_x;
      ++out_y;
      ++out_z;

      ++out_range;
      ++out_azimuth;
      ++out_elevation;

      ++out_doppler;
      ++out_snr;
      ++out_rcs;
    }


    output_pub_->publish(
      output);


    ++frame_count_;


    /*
     * Diagnostic output roughly every 30 Radar frames
     * (~2 seconds at 15 Hz).
     */
    if (
      frame_count_ % 30 == 0 &&
      !points.empty())
    {
      const double n =
        static_cast<double>(
          points.size());


      RCLCPP_INFO(
        get_logger(),
        "mmWave | points=%zu "
        "odom_dt=%.4fs "
        "doppler mean=%.3f "
        "abs=%.3f "
        "range=[%.3f %.3f] m/s "
        "SNR=%.1fdB",
        points.size(),
        odom_dt,
        sum_doppler / n,
        sum_abs_doppler / n,
        min_doppler,
        max_doppler,
        sum_snr / n);
    }
  }


private:

  std::string input_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string output_frame_;


  double radar_x_;
  double radar_y_;
  double radar_z_;
  double radar_yaw_;

  double max_odom_time_diff_;

  double doppler_noise_std_;

  double snr_at_1m_db_;
  double snr_noise_std_db_;
  double min_snr_db_;
  double max_snr_db_;

  double rcs_mean_dbsm_;
  double rcs_std_db_;


  std::deque<
    nav_msgs::msg::Odometry::SharedPtr>
    odom_buffer_;


  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    output_pub_;


  std::mt19937 rng_;

  uint64_t frame_count_{0};
  uint64_t missing_odom_count_{0};
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  rclcpp::spin(
    std::make_shared<
      MmWaveEnricher>());

  rclcpp::shutdown();

  return 0;
}
