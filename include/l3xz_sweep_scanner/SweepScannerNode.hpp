/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>, Jonas Wuehr
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#ifndef SWEEP_SCANNER_NODE
#define SWEEP_SCANNER_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sweep/sweep.hpp>

#include <string>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SweepScannerNode : public rclcpp::Node
{
public:
   SweepScannerNode();
   ~SweepScannerNode();

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_pub;
  rclcpp::TimerBase::SharedPtr _lidar_pub_timer;
  
  std::shared_ptr<sweep::sweep> _scanner;
   
  std::string _frame_id;
  int _rotation_speed, _sample_rate;

  void lidarTimerCallback();
};
#endif // SWEEP_SCANNER_NODE
