/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>, Jonas Wuehr
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <l3xz_sweep_scanner/SweepScannerNode.hpp>
#include <string>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

SweepScannerNode::SweepScannerNode()
        : Node("l3xz_sweep_scanner")
{
  declare_parameter("serial_port", "/dev/ttyUSB0");
  declare_parameter("rotation_speed", 1);
  declare_parameter("sample_rate", 500);
  declare_parameter("frame_id", "laser_frame");
 
  std::string serial_port = get_parameter("serial_port").as_string();
  _frame_id = get_parameter("frame_id").as_string();
  _rotation_speed = get_parameter("rotation_id").as_int();
  _sample_rate = get_parameter("sample_rate").as_int();
  
  RCLCPP_INFO(get_logger(), "node config:\n  port : %s\n  speed: %d Hz\n  rate : %d Hz\n  frame: %s", serial_port.c_str(), _rotation_speed, _sample_rate, _frame_id.c_str());

  _scanner = std::make_shared<sweep::sweep>(serial_port.c_str());
  RCLCPP_INFO(get_logger(), "configuring scanse sweep.");
  _scanner->set_motor_speed(_rotation_speed);
  while(!_scanner->get_motor_ready())
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  _scanner->set_sample_rate(_sample_rate);
  _scanner->start_scanning();

  RCLCPP_INFO(get_logger(), "starting data aquisition.");

  _lidar_pub = create_publisher<sensor_msgs::msg::LaserScan>(_frame_id, 10);
  _lidar_pub_timer = create_wall_timer(std::chrono::milliseconds(1000 / _sample_rate), [this](){this->lidarTimerCallback();});
}

SweepScannerNode::~SweepScannerNode()
{
  _scanner->stop_scanning();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void SweepScannerNode::lidarTimerCallback()
{
  /* Obtain a full scan. */
  sweep::scan const scan = _scanner->get_scan();

  sensor_msgs::msg::LaserScan laser_scan_msg;
  /* Populate sensor_msgs/LaserScan header. */
  laser_scan_msg.header.frame_id = _frame_id;

  /* Populate sensor_msgs/LaserScan data. */
  float const samples_per_rotation = static_cast<float>(_sample_rate) / static_cast<float>(_rotation_speed);

  laser_scan_msg.angle_min       = 0.0;
  laser_scan_msg.angle_max       = 2.0 * M_PI;
  laser_scan_msg.angle_increment = laser_scan_msg.angle_max / samples_per_rotation;
  laser_scan_msg.time_increment  = 1.0 / static_cast<float>(_sample_rate);
  laser_scan_msg.range_min       = 0.0;
  laser_scan_msg.range_max       = 40.0;

  laser_scan_msg.ranges.assign(scan.samples.size(), std::numeric_limits<float>::infinity());

  size_t idx = 0;
  for (auto [angle_milli_deg, distance_cm, signal_strength] : scan.samples) {
    laser_scan_msg.ranges[idx] = static_cast<float>(distance_cm) / 100.0;
    idx++;
  }

  /* Publish the laser scan. */
  _lidar_pub->publish(laser_scan_msg);
}
