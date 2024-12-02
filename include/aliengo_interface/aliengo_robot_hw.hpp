/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef ALIENGO_ROBOT_HW_H
#define ALIENGO_ROBOT_HW_H

#include <base_hardware_interface/base_robot_hw.h>
#include <aliengo_hal/aliengo_hal.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>

namespace aliengo2ros
{

class AliengoRobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  AliengoRobotHw();
  virtual ~AliengoRobotHw();

  void init();
  void read();
  void write();

private:

  /** @brief Map Aliengo internal joint indices to WoLF joints order */
  std::array<unsigned int, 12> aliengo_motor_idxs_
          {{
          aliengohal::FL_0, aliengohal::FL_1, aliengohal::FL_2, // LF
          aliengohal::RL_0, aliengohal::RL_1, aliengohal::RL_2, // LH
          aliengohal::FR_0, aliengohal::FR_1, aliengohal::FR_2, // RF
          aliengohal::RR_0, aliengohal::RR_1, aliengohal::RR_2, // RH
          }};

  /** @brief Aliengo-HAL */
  aliengohal::LowLevelInterface aliengo_interface_;
  aliengohal::LowState aliengo_state_ = {0};
  aliengohal::LowCmd aliengo_lowcmd_ = {0};

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();


  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_acc_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_euler_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> contact_state_pub_;


  std::vector<double> imu_euler_raw_;
  std::vector<double> imu_orientation_raw_;
  std::vector<double> remove_euler_;
  std::vector<double> remove_quaternion_;

  bool is_remove_yaw_set_ = false;
  unsigned int base_pub_counter = 0;

};

}

#endif
