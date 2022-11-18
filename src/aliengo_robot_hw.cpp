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

#include "aliengo_interface/aliengo_robot_hw.hpp"

namespace aliengo2ros
{

using namespace hardware_interface;

int64_t utime_now() {

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
        throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
    uint32_t sec	= timeofday.tv_sec;
    uint32_t nsec = timeofday.tv_usec * 1000;

    return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

AliengoRobotHw::AliengoRobotHw()
{

    robot_name_ = "aliengo";
}

AliengoRobotHw::~AliengoRobotHw()
{

}

void AliengoRobotHw::init()
{


    // Hardware interfaces: Joints
    auto joint_names = loadJointNamesFromSRDF();
    if(joint_names.size()>0)
    {
      WolfRobotHwInterface::initializeJointsInterface(joint_names);
      registerInterface(&joint_state_interface_);
      registerInterface(&joint_effort_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
      return;
    }

    // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
      imu_euler_raw_.resize(3);
      imu_euler_raw_[0] = 0.0;
      imu_euler_raw_[1] = 0.0;
      imu_euler_raw_[2] = 0.0;

      imu_orientation_raw_.resize(4);
      imu_orientation_raw_[0] = 1.0;
      imu_orientation_raw_[1] = 0.0;
      imu_orientation_raw_[2] = 0.0;
      imu_orientation_raw_[3] = 0.0;

      remove_euler_.resize(3);
      remove_euler_[0] = 0.0;
      remove_euler_[1] = 0.0;
      remove_euler_[2] = 0.0;
      remove_quaternion_.resize(4);
      remove_quaternion_[0] = 1.0;
      remove_quaternion_[1] = 0.0;
      remove_quaternion_[2] = 0.0;
      remove_quaternion_[3] = 0.0;
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }

    aliengo_interface_.InitCmdData(aliengo_lowcmd_);
    startup_routine();

    ros::NodeHandle root_nh;
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh,	"/aliengo/ground_truth", 1));
    imu_acc_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(root_nh,	"/aliengo/trunk_imu", 1));
    imu_euler_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(root_nh,	"/aliengo/euler_imu", 1));


}

void AliengoRobotHw::read()
{
    // Get robot data
    aliengo_state_ = aliengo_interface_.ReceiveObservation();

    // ------
    // Joints
    // ------
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
        joint_position_[jj] = static_cast<double>(aliengo_state_.motorState[aliengo_motor_idxs_[jj]].q)     ;
        joint_velocity_[jj] = static_cast<double>(aliengo_state_.motorState[aliengo_motor_idxs_[jj]].dq)    ;
        joint_effort_[jj]   = static_cast<double>(aliengo_state_.motorState[aliengo_motor_idxs_[jj]].tauEst);
    }

    // ---
    // IMU
    // ---
    if (not is_remove_yaw_set_)
    {
      // These lines remove init yaw of the robot
      remove_euler_[2] = -static_cast<double>(go1_state_.imu.rpy[2]);
      remove_quaternion_[0] = sin(remove_euler_[2]/2); // w
      // remove_quaternion[1] = 0.                     // x
      // remove_quaternion[2] = 0.                     // y
      remove_quaternion_[3] = cos(remove_euler_[2]/2); // z
      is_remove_yaw_set_ = true;
    }

    imu_orientation_raw_[0] = static_cast<double>(go1_state_.imu.quaternion[0]);  // w
    imu_orientation_raw_[1] = static_cast<double>(go1_state_.imu.quaternion[1]);  // x
    imu_orientation_raw_[2] = static_cast<double>(go1_state_.imu.quaternion[2]);  // y
    imu_orientation_raw_[3] = static_cast<double>(go1_state_.imu.quaternion[3]);  // z

    imu_orientation_[0] = remove_quaternion_[0] * imu_orientation_raw_[0] - remove_quaternion_[3] * imu_orientation_raw_[3];
    imu_orientation_[1] = remove_quaternion_[0] * imu_orientation_raw_[1] - remove_quaternion_[3] * imu_orientation_raw_[2];
    imu_orientation_[2] = remove_quaternion_[0] * imu_orientation_raw_[2] + remove_quaternion_[3] * imu_orientation_raw_[1];
    imu_orientation_[3] = remove_quaternion_[0] * imu_orientation_raw_[3] + remove_quaternion_[3] * imu_orientation_raw_[0];

    imu_euler_raw_[0] = static_cast<double>(go1_state_.imu.rpy[0]);  // R
    imu_euler_raw_[1] = static_cast<double>(go1_state_.imu.rpy[1]);  // P
    imu_euler_raw_[2] = static_cast<double>(go1_state_.imu.rpy[2]);  // Y
    imu_euler_[0] = imu_euler_raw_[0] + remove_euler_[0];
    imu_euler_[1] = imu_euler_raw_[1] + remove_euler_[1];
    imu_euler_[2] = imu_euler_raw_[2] + remove_euler_[2];

    imu_ang_vel_[0] = static_cast<double>(go1_state_.imu.gyroscope[0]);
    imu_ang_vel_[1] = static_cast<double>(go1_state_.imu.gyroscope[1]);
    imu_ang_vel_[2] = static_cast<double>(go1_state_.imu.gyroscope[2]);

    imu_lin_acc_[0] = static_cast<double>(go1_state_.imu.accelerometer[0]);
    imu_lin_acc_[1] = static_cast<double>(go1_state_.imu.accelerometer[1]);
    imu_lin_acc_[2] = static_cast<double>(go1_state_.imu.accelerometer[2]);


    // Publish the IMU data NOTE: missing covariances
    if(odom_pub_.get() && odom_pub_->trylock())
    {
      odom_pub_->msg_.pose.pose.orientation.w         = imu_orientation_[0];
      odom_pub_->msg_.pose.pose.orientation.x         = imu_orientation_[1];
      odom_pub_->msg_.pose.pose.orientation.y         = imu_orientation_[2];
      odom_pub_->msg_.pose.pose.orientation.z         = imu_orientation_[3];
      odom_pub_->msg_.twist.twist.angular.x    = imu_ang_vel_[0];
      odom_pub_->msg_.twist.twist.angular.y    = imu_ang_vel_[1];
      odom_pub_->msg_.twist.twist.angular.z    = imu_ang_vel_[2];

      odom_pub_->msg_.header.stamp = ros::Time::now();
      odom_pub_->unlockAndPublish();
    }


    if(imu_acc_pub_.get() && imu_acc_pub_->trylock())
    {
      imu_acc_pub_->msg_.x = imu_lin_acc_[0];
      imu_acc_pub_->msg_.y = imu_lin_acc_[1];
      imu_acc_pub_->msg_.z = imu_lin_acc_[2];
      
      imu_acc_pub_->unlockAndPublish();
    }

    if(imu_euler_pub_.get() && imu_euler_pub_->trylock())
    {
      imu_euler_pub_->msg_.x = imu_euler_[0];
      imu_euler_pub_->msg_.y = imu_euler_[1];
      imu_euler_pub_->msg_.z = imu_euler_[2];
      
      imu_euler_pub_->unlockAndPublish();
    }
}

void AliengoRobotHw::write()
{
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
      aliengo_lowcmd_.motorCmd[aliengo_motor_idxs_[jj]].tau = static_cast<float>(joint_effort_command_[jj]  );

    aliengo_interface_.SendLowCmd(aliengo_lowcmd_);
}

void AliengoRobotHw::send_zero_command()
{
    std::array<float, 60> zero_command = {0};
    // aliengo_interface_->SendCommand(zero_command);
    aliengo_interface_.SendCommand(zero_command);
}

void AliengoRobotHw::startup_routine()
{
    send_zero_command();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace
