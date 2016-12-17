#ifndef UR_HARDWARE_DRIVER_CONFIG_H
#define UR_HARDWARE_DRIVER_CONFIG_H

/** ATTENTION
This file was generated from the config file ur_hardware_driver_config.yaml
and contains set of _default_ parameters.
DO NOT DARE TO EDIT ANY OF THIS
*/

#include "ros/ros.h"

class UrHardwareDriverConfig
{

public:
  void init()
  {
    ros::NodeHandle parameter_node_handle("~");
    parameter_node_handle.param<double>("steering_enc_coeff", steering_enc_coeff, 0.00390625);
    parameter_node_handle.param<double>("steering_zero_offset", steering_zero_offset, 0.0);
    parameter_node_handle.param<std::string>("umoto_ip", umoto_ip, "192.168.1.190");
    parameter_node_handle.param<int>("umoto_port", umoto_port, 20000);
    parameter_node_handle.param<double>("velocity_enc_coeff", velocity_enc_coeff, 0.0009765625);
  }

  double steering_enc_coeff = 0.00390625;
  double steering_zero_offset = 0.0;
  std::string umoto_ip = "192.168.1.190";
  int umoto_port = 20000;
  double velocity_enc_coeff = 0.0009765625;

};

extern UrHardwareDriverConfig ur_hardware_driver_config;

#endif
