/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, KB AVRORA LLC
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
 *   * Neither the name of KB AVRORA LLC nor the names of its
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

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <controller_manager/controller_manager.h>

#include "uniorhwinterface.h"
#include "ur_hardware_driver_config.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ur_hardware_driver");

  ros::NodeHandle node_handle;
  ur_hardware_driver_config.init();

  UniorHW interface;

  controller_manager::ControllerManager cm(&interface);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(50);

  ros::Time prev_time = ros::Time::now();

  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration duration = time - prev_time;

    //interface.read(time, duration); // performed in write() method due protocol limitations
    cm.update(time, duration);
    interface.write(time, duration);

    prev_time = time;

    rate.sleep();
  }

  return 0;
}
