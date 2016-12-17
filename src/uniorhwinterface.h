/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, AVRORA ROBOTICS LLC
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AVRORA ROBOTICS LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Vladimir Leushkin
   Desc:
*/

#ifndef UNIORHWINTERFACE_H
#define UNIORWINTERFACE_H

#include <ros/console.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "ur_hardware_driver_config.h"
#include "umotoudpinterface.h"

using namespace hardware_interface;
using namespace joint_limits_interface;
using namespace umoto_interface;

class UniorHW : public hardware_interface::RobotHW
{
public:
  UniorHW();

  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);
private:
  void registerSteeringJoint();
  void registerVelocityJoint();

  UMotoUDPInterface umoto_interface_;

  JointStateInterface joint_state_interface_;
  VelocityJointInterface joint_vel_command_interface_;
  PositionJointInterface joint_pos_command_interface_;

  VelocityJointSoftLimitsInterface limits_vel_interface_;
  PositionJointSoftLimitsInterface limits_pos_interface_;

  double steering_wheel_pose_ = 0;
  double steering_wheel_vel_ = 0;
  double steering_wheel_effort_ = 0;
  double steering_wheel_cmd_ = 0;

  double rear_wheel_pose_ = 0;
  double rear_wheel_vel_ = 0;
  double rear_wheel_effort_ = 0;
  double rear_wheel_cmd_ = 0;

  const int MAX_COMM_ERRORS = 5;
  int communication_error_cnt_ = 0;
};

#endif // UNIORHWINTERFACE_H
