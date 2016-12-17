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

#include "uniorhwinterface.h"

UniorHW::UniorHW()
{

  JointStateHandle state_steering_handle("steering_joint", &steering_wheel_pose_,
                                         &steering_wheel_vel_,
                                         &steering_wheel_effort_);
  joint_state_interface_.registerHandle(state_steering_handle);

  JointStateHandle state_velocity_b("velocity_joint", &rear_wheel_pose_,
                                    &rear_wheel_vel_,
                                    &rear_wheel_effort_);
  joint_state_interface_.registerHandle(state_velocity_b);

  registerInterface(&joint_state_interface_);

  registerSteeringJoint();
  registerVelocityJoint();

  ROS_INFO_STREAM_NAMED("UMotoInteface", "UMoto address "<< ur_hardware_driver_config.umoto_ip << " port "
                        << ur_hardware_driver_config.umoto_port);
  umoto_interface_.connectToIP(ur_hardware_driver_config.umoto_ip, ur_hardware_driver_config.umoto_port);

  try
  {
    UMotoVersion resp = umoto_interface_.requestUMotoVersion();
    ROS_INFO_STREAM_NAMED("UMotoInteface", "HW version: "<<
                          (int)resp.version[0]<< "." <<
                                                      (int)resp.version[1]<< "." <<
                                                      (int)resp.version[2]<< "." <<
                                                      (int)resp.version[3]);
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("UMotoInteface", "Unable to get HW version: "<<e.what());
  }

}

void UniorHW::read(const ros::Time& time, const ros::Duration& period)
{
  // data reading implemented in write method
}

void UniorHW::write(const ros::Time& time, const ros::Duration& period)
{
  limits_pos_interface_.enforceLimits(period);
  limits_vel_interface_.enforceLimits(period);

  steering_wheel_effort_ = steering_wheel_cmd_; // effort contains actual control effort
  rear_wheel_effort_ = rear_wheel_cmd_;

  UMotoControl control;
  control.steering = steering_wheel_cmd_;
  control.velocity = rear_wheel_cmd_;

  try
  {
    UMotoData data_packet = umoto_interface_.sendControl(control);
    steering_wheel_pose_ = data_packet.steering_enc;
    rear_wheel_vel_ = data_packet.velocity;

    communication_error_cnt_ = 0;
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM_NAMED("UniorHWInterface", "UMoto communication error : "<<e.what());
    communication_error_cnt_++;

    if (communication_error_cnt_>=MAX_COMM_ERRORS)
    {
      ROS_FATAL_STREAM_NAMED("UniorHWInterface", "Please, check UMoto connection. Node shutdown..");
      ros::shutdown();
    }
  }
}

void UniorHW::registerSteeringJoint()
{
  JointHandle pos_handle_a(joint_state_interface_.getHandle("steering_joint"), &steering_wheel_cmd_);
  joint_pos_command_interface_.registerHandle(pos_handle_a);

  // TODO: use urdf params
  JointLimits limits_pos;
  limits_pos.has_position_limits = true;
  limits_pos.min_position = -110;
  limits_pos.max_position = 110;
  limits_pos.has_velocity_limits = true;
  limits_pos.max_velocity = 800.0;

  SoftJointLimits soft_limits_pos;
  soft_limits_pos.min_position = limits_pos.min_position;
  soft_limits_pos.max_position = limits_pos.max_position;
  soft_limits_pos.k_position = 10.0;

  PositionJointSoftLimitsHandle pos_limits_handle(joint_pos_command_interface_.getHandle("steering_joint"),
                                                  limits_pos, soft_limits_pos);
  limits_pos_interface_.registerHandle(pos_limits_handle);
  registerInterface(&joint_pos_command_interface_);
}

void UniorHW::registerVelocityJoint()
{
  JointHandle pos_handle_b(joint_state_interface_.getHandle("velocity_joint"), &rear_wheel_cmd_);
  joint_vel_command_interface_.registerHandle(pos_handle_b);

  // TODO: use urdf params
  JointLimits limits_vel;
  limits_vel.has_velocity_limits = true;
  limits_vel.max_velocity = 400.0;

  limits_vel.has_acceleration_limits = true;
  limits_vel.max_acceleration = 6000;

  SoftJointLimits soft_limits_vel;
  soft_limits_vel.k_position = 1;

  VelocityJointSoftLimitsHandle vel_limits_handle(joint_vel_command_interface_.getHandle("velocity_joint"),
                                                  limits_vel, soft_limits_vel);
  limits_vel_interface_.registerHandle(vel_limits_handle);

  registerInterface(&joint_vel_command_interface_);
}
