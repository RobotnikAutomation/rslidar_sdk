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

#ifdef ROS_FOUND  ///< if ROS is found, call the ros::init function
  ros::init(argc, argv, "rslidar_sdk_node", ros::init_options::NoSigintHandler);
#endif

#ifdef ROS2_FOUND  ///< if ROS2 is found, call the rclcpp::init function
  rclcpp::init(argc, argv);
#endif

  YAML::Node config;
  std::string rslidar_config_file = (std::string)PROJECT_PATH + "/config/config.yaml";
  if (ros::param::has("rslidar_config_file"))
    ros::param::get("rslidar_config_file", rslidar_config_file);
  RS_WARNING << "Using configuration file in the directory: " << rslidar_config_file <<"." << RS_REND;
  std::string frame_id = "";
  if (ros::param::has("frame_id"))
    ros::param::get("frame_id", frame_id);

  std::string ros_send_point_cloud_topic = "";
  if (ros::param::has("ros_send_point_cloud_topic"))
    ros::param::get("ros_send_point_cloud_topic", ros_send_point_cloud_topic);

  std::string lidar_type = "";
  if (ros::param::has("lidar_type"))
    ros::param::get("lidar_type", lidar_type);

  int msop_port = -1;
  if (ros::param::has("msop_port"))
    ros::param::get("msop_port", msop_port);

  int difop_port = -1;
  if (ros::param::has("difop_port"))
    ros::param::get("difop_port", difop_port);

  try
  {
    config = YAML::LoadFile(rslidar_config_file);
    for (auto i : config["lidar"])
    {
      if(ros_send_point_cloud_topic != "")
      {
        RS_INFO << "Using ros_send_point_cloud_topic: " << ros_send_point_cloud_topic << RS_REND;
        i["ros"]["ros_send_point_cloud_topic"] = ros_send_point_cloud_topic;
      }
      if(frame_id != "")
      {
        RS_INFO << "Using frame_id: " << frame_id << RS_REND;
        i["driver"]["frame_id"] = frame_id;
      }
      if (lidar_type != "")
      {
        RS_INFO << "Using lidar_type: " << lidar_type << RS_REND;
        i["driver"]["lidar_type"] = lidar_type;
      }
      if (msop_port != -1)
      {
        RS_INFO << "Using msop_port: " << msop_port << RS_REND;
        i["driver"]["msop_port"] = msop_port;
      }
      if (difop_port != -1)
      {
        RS_INFO << "Using difop_port: " << difop_port << RS_REND;
        i["driver"]["difop_port"] = difop_port;
      }
    }
    
  }
  catch (...)
  {
    RS_ERROR << "Config file format wrong! Please check the format(e.g. indentation) " << RS_REND;
    return -1;
  }

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