/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <signal.h>
#include "manager/adapter_manager.h"
using namespace robosense::lidar;
std::mutex g_mtx;
std::condition_variable g_cv;
static void sigHandler(int sig)
{
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
#ifdef ROS_FOUND
  ros::shutdown();
#endif
  g_cv.notify_all();
}

template <typename T>
T setParameter(ros::NodeHandle h, YAML::Node config, std::string paramNamespace, std::string paramName, T defaultValue)
{
  T param;
  yamlRead<T>(config, paramName, param, defaultValue);
  h.param<T>(paramNamespace + paramName, param, param);

  return param;
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function
  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
           << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

  std::shared_ptr<AdapterManager> demo_ptr = std::make_shared<AdapterManager>();
  YAML::Node config;
  try
  {
    config = YAML::LoadFile((std::string)PROJECT_PATH + "/config/config.yaml");
  }
  catch (...)
  {
    RS_ERROR << "Config file format wrong! Please check the format(e.g. indentation) " << RS_REND;
    return -1;
  }

#ifdef ROS_FOUND  ///< if ROS is found, call the ros::init function
  ros::init(argc, argv, "rslidar_sdk_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  bool read_ros_params = false;
  nh.param<bool>("read_ros_params", read_ros_params, read_ros_params);

  if (read_ros_params)
  {
    // COMMON CONFIG
  
    YAML::Node common_config = yamlSubNodeAbort(config, "common");


    config["common"]["msg_source"]              = setParameter(nh, common_config, "common/", "msg_source", 0);
    config["common"]["send_packet_ros"]         = setParameter(nh, common_config, "common/", "send_packet_ros", false);
    config["common"]["send_point_cloud_ros"]    = setParameter(nh, common_config, "common/", "send_point_cloud_ros", true);
    config["common"]["send_packet_proto"]       = setParameter(nh, common_config, "common/", "send_packet_proto", false);
    config["common"]["send_point_cloud_proto"]  = setParameter(nh, common_config, "common/", "send_point_cloud_proto", false);
    config["common"]["pcap_path"]               = setParameter(nh, common_config, "common/", "pcap_path", std::string("/home/robosense/lidar.pcap"));

    // LIDAR CONFIG
    for (uint i = 0; i < config["lidar"].size(); i++)
    {
      for (const auto& kv: config["lidar"][i])
      {
        std::string lidar_config_key = kv.first.as<std::string>();

        YAML::Node subconfig = config["lidar"][i][lidar_config_key];

        // LIDAR CONFIG -> DRIVER
        if (lidar_config_key == "driver")
        {

          std::string default_lidar_type = "RS128";
          config["lidar"][i][lidar_config_key]["lidar_type"] = setParameter(nh, subconfig, "lidar/driver/", "lidar_type", default_lidar_type);
          std::string default_frame_id = "/rslidar";
          config["lidar"][i][lidar_config_key]["frame_id"] = setParameter(nh, subconfig, "lidar/driver/", "frame_id", default_frame_id);
          config["lidar"][i][lidar_config_key]["msop_port"] = setParameter(nh, subconfig, "lidar/driver/", "msop_port", 6699);
          config["lidar"][i][lidar_config_key]["difop_port"] = setParameter(nh, subconfig, "lidar/driver/", "difop_port", 7788);
          config["lidar"][i][lidar_config_key]["start_angle"] = setParameter(nh, subconfig, "lidar/driver/", "start_angle", 0);
          config["lidar"][i][lidar_config_key]["end_angle"] = setParameter(nh, subconfig, "lidar/driver/", "end_angle", 360);
          config["lidar"][i][lidar_config_key]["min_distance"] = setParameter(nh, subconfig, "lidar/driver/", "min_distance", 0.2);
          config["lidar"][i][lidar_config_key]["max_distance"] = setParameter(nh, subconfig, "lidar/driver/", "max_distance", 200);
          config["lidar"][i][lidar_config_key]["use_lidar_clock"] = setParameter(nh, subconfig, "lidar/driver/", "use_lidar_clock", false);

        }

        // LIDAR CONFIG -> ROS
        if (lidar_config_key == "ros")
        {
          std::string ros_recv_packet_topic = "/rslidar_packets";
          config["lidar"][i][lidar_config_key]["ros_recv_packet_topic"] = setParameter(nh, subconfig, "lidar/ros/", "ros_recv_packet_topic", ros_recv_packet_topic);
          
          std::string ros_send_packet_topic = "/rslidar_packets";
          config["lidar"][i][lidar_config_key]["ros_send_packet_topic"] = setParameter(nh, subconfig, "lidar/ros/", "ros_send_packet_topic", ros_send_packet_topic);

          std::string ros_send_point_cloud_topic = "/rslidar_points";
          config["lidar"][i][lidar_config_key]["ros_send_point_cloud_topic"] = setParameter(nh, subconfig, "lidar/ros/", "ros_send_point_cloud_topic", ros_send_point_cloud_topic);
        }

        if (lidar_config_key == "proto")
        {
          config["lidar"][i][lidar_config_key]["point_cloud_recv_port"] = setParameter(nh, subconfig, "lidar/proto/", "point_cloud_recv_port", 60021);
          config["lidar"][i][lidar_config_key]["point_cloud_send_port"] = setParameter(nh, subconfig, "lidar/proto/", "point_cloud_send_port", 60021);
          config["lidar"][i][lidar_config_key]["msop_recv_port"] = setParameter(nh, subconfig, "lidar/proto/", "msop_recv_port", 60022);
          config["lidar"][i][lidar_config_key]["msop_send_port"] = setParameter(nh, subconfig, "lidar/proto/", "msop_send_port", 60022);
          config["lidar"][i][lidar_config_key]["difop_recv_port"] = setParameter(nh, subconfig, "lidar/proto/", "difop_recv_port", 60023);
          config["lidar"][i][lidar_config_key]["difop_send_port"] = setParameter(nh, subconfig, "lidar/proto/", "difop_send_port", 60023);
          std::string point_cloud_send_ip = "127.0.0.1";
          config["lidar"][i][lidar_config_key]["point_cloud_send_ip"] = setParameter(nh, subconfig, "lidar/proto/", "point_cloud_send_ip", point_cloud_send_ip);
          std::string packet_send_ip = "127.0.0.1";
          config["lidar"][i][lidar_config_key]["packet_send_ip"] = setParameter(nh, subconfig, "lidar/proto/", "packet_send_ip", packet_send_ip);
        }
 
      }

    }  
  }

#endif

#ifdef ROS2_FOUND  ///< if ROS2 is found, call the rclcpp::init function
  rclcpp::init(argc, argv);
#endif

  demo_ptr->init(config);
  demo_ptr->start();
  RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;

#ifdef ROS_FOUND
  ros::spin();
#else
  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
#endif
  return 0;
}