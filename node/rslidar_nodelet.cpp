
#include "manager/node_manager.hpp"

#include <signal.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace rslidar_sdk{
class rslidarNodelet : public nodelet::Nodelet
{
public:
    rslidarNodelet(){}
    virtual ~rslidarNodelet() = default;
private:
    void onInit() 
    {
        nh_ = getNodeHandle();
        nhp_ = getPrivateNodeHandle();
        std::string config_path;
        if(!nhp_.param("rs_lidar_config_path", config_path, std::string("")))
        {
            NODELET_ERROR("file could not be found");
        }
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(config_path);
        }
        catch (...)
        {
            NODELET_ERROR("The format of config file %s is wrong. Please check (e.g. indentation).", config_path.c_str());
        }

        std::string lidar_type = "";
        nhp_.param("lidar_type", lidar_type, lidar_type);
        if (lidar_type != "")
            config["lidar"][0]["driver"]["lidar_type"] = lidar_type;

        int msop_port = -1;
        nhp_.param("msop_port", msop_port, msop_port);
        if (lidar_type != "")
            config["lidar"][0]["driver"]["msop_port"] = msop_port;

        int difop_port = -1;
        nhp_.param("difop_port", difop_port, difop_port);
        if (lidar_type != "")
            config["lidar"][0]["driver"]["difop_port"] = difop_port;

        std::string ros_frame_id = "";
        nhp_.param("frame_id", ros_frame_id, ros_frame_id);
        if (lidar_type != "")
            config["lidar"][0]["ros"]["ros_frame_id"] = ros_frame_id;

        demo_ptr = std::make_shared<robosense::lidar::NodeManager>();
        demo_ptr->init(config, nhp_);
        demo_ptr->start();
    }
    std::shared_ptr<robosense::lidar::NodeManager> demo_ptr;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
};
}

PLUGINLIB_EXPORT_CLASS(rslidar_sdk::rslidarNodelet, nodelet::Nodelet);