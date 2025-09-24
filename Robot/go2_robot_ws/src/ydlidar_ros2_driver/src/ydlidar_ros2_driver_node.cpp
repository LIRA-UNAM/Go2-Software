/* * YDLIDAR SYSTEM
 * YDLIDAR ROS 2 Node *
 * Copyright 2017 - 2020 EAI TEAM
 * http://www.eaibot.com *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"  // SDK YDLIDAR
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>
#include <limits>
#include <cmath>
#include <string>
#include <algorithm>  // (por si lo necesitas para otras cosas)

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"

class YDLidarNode : public rclcpp::Node
{
public:
  YDLidarNode() : Node("ydlidar_ros2_driver_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting YDLIDAR ROS2 Node (TG-ready, TF-stamped, canonical geometry)");

    // ---------------- Parámetros ----------------
    std::string serial_port = "/dev/ydlidar";
    this->declare_parameter<std::string>("serial_port", serial_port);
    (void)this->get_parameter("serial_port", serial_port);

    int serial_baudrate = 512000;  // Serie TG
    this->declare_parameter<int>("serial_baudrate", serial_baudrate);
    (void)this->get_parameter("serial_baudrate", serial_baudrate);

    int lidar_type = TYPE_TOF;     // TG es ToF
    this->declare_parameter<int>("lidar_type", lidar_type);
    (void)this->get_parameter("lidar_type", lidar_type);

    int device_type = YDLIDAR_TYPE_SERIAL;
    this->declare_parameter<int>("device_type", device_type);
    (void)this->get_parameter("device_type", device_type);

    bool auto_reconnect = true;
    this->declare_parameter<bool>("auto_reconnect", auto_reconnect);
    (void)this->get_parameter("auto_reconnect", auto_reconnect);

    frame_id_ = "laser_frame";
    this->declare_parameter<std::string>("frame_id", frame_id_);
    (void)this->get_parameter("frame_id", frame_id_);

    std::string scan_topic = "scan";
    this->declare_parameter<std::string>("scan_topic", scan_topic);
    (void)this->get_parameter("scan_topic", scan_topic);

    // ---------------- Configurar SDK ----------------
    laser_ = std::make_shared<CYdLidar>();
    laser_->setlidaropt(LidarPropSerialPort, serial_port.c_str(), serial_port.size());
    laser_->setlidaropt(LidarPropSerialBaudrate, &serial_baudrate, sizeof(int));
    laser_->setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));
    laser_->setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));
    laser_->setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

    if (!laser_->initialize())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize YDLIDAR: %s", laser_->DescribeError());
      rclcpp::shutdown();
      return;
    }
    laser_->turnOn();

    // ---------------- TF2 (para estampar con odom→base_link) ----------------
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------------- Publisher ----------------
    rclcpp::QoS qos = rclcpp::SensorDataQoS();  // recomendado para datos de sensores
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, qos);

    // ---------------- Timer (~20 Hz) ----------------
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&YDLidarNode::publishLaserScan, this));
  }

  ~YDLidarNode()
  {
    if (laser_)
    {
      laser_->turnOff();
      laser_->disconnecting();
    }
  }

private:
  void publishLaserScan()
  {
    LaserScan scan;
    if (!laser_->doProcessSimple(scan))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to get scan");
      return;
    }

    sensor_msgs::msg::LaserScan scan_msg;

    // Estampado temporal: intenta usar el sello de TF odom→base_link (como pediste)
    try
    {
      auto odom_tf = tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time(0));
      scan_msg.header.stamp = odom_tf.header.stamp;
      RCLCPP_DEBUG(this->get_logger(), "Using odom→base_link timestamp");
    }
    catch (const tf2::TransformException &ex)
    {
      scan_msg.header.stamp = this->get_clock()->now(); // fallback
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "TF odom→base_link no disponible: %s. Usando reloj local.", ex.what());
    }

    scan_msg.header.frame_id = frame_id_;

    // --- Config del SDK del cuadro actual ---
    const double cur_min       = scan.config.min_angle;
    const double cur_max       = scan.config.max_angle;
    const double cur_inc       = scan.config.angle_increment;
    const double cur_scan_time = scan.config.scan_time;
    const double cur_time_inc  = scan.config.time_increment;
    const double cur_rmin      = scan.config.min_range;
    const double cur_rmax      = scan.config.max_range;

    // --- Inicializa geometría canónica en el primer frame ---
    if (!have_canon_)
    {
      int size_guess = std::max(1, static_cast<int>(std::floor((cur_max - cur_min) / cur_inc) + 1));
      double inc_fixed = (size_guess > 1)
                          ? (cur_max - cur_min) / static_cast<double>(size_guess - 1)
                          : cur_inc;

      canon_min_  = cur_min;
      canon_max_  = cur_max;
      canon_inc_  = inc_fixed;
      canon_size_ = size_guess;
      have_canon_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Canonical scan geometry set: size=%d min=%.6f max=%.6f inc=%.9f",
                  canon_size_, canon_min_, canon_max_, canon_inc_);
    }

    // --- Publica SIEMPRE con la geometría canónica ---
    scan_msg.angle_min       = canon_min_;
    scan_msg.angle_max       = canon_max_;
    scan_msg.angle_increment = canon_inc_;
    scan_msg.scan_time       = cur_scan_time;
    scan_msg.time_increment  = (cur_time_inc > 0.0)
                               ? cur_time_inc
                               : (canon_size_ > 1 && cur_scan_time > 0.0
                                  ? cur_scan_time / static_cast<double>(canon_size_ - 1)
                                  : 0.0);
    scan_msg.range_min = cur_rmin;
    scan_msg.range_max = cur_rmax;

    scan_msg.ranges.assign(canon_size_, std::numeric_limits<float>::infinity());
    scan_msg.intensities.assign(canon_size_, 0.0f);

    // --- Relleno indexando en la rejilla canónica ---
    for (const auto &p : scan.points)
    {
      const double rel = (p.angle - canon_min_) / canon_inc_;
      int index = static_cast<int>(std::lround(rel));
      if (index < 0) index = 0;
      else if (index >= canon_size_) index = canon_size_ - 1;

      scan_msg.ranges[index]      = p.range;
      scan_msg.intensities[index] = p.intensity;
    }

    laser_pub_->publish(scan_msg);
  }

  // --- Miembros ---
  std::shared_ptr<CYdLidar> laser_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Geometría canónica (congelada en el primer frame)
  bool   have_canon_{false};
  double canon_min_{0.0};
  double canon_max_{0.0};
  double canon_inc_{0.0};
  int    canon_size_{0};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YDLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
