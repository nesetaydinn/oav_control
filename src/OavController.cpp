#include "OavController.h"


OavController::OavController(ros::NodeHandle &nh,ros::NodeHandle &priv_nh):
    nh_(nh),
    priv_nh_(priv_nh)
{   
    /* Params */
    robot_id_ = priv_nh.param<std::string>("robot_id", "rX");
    cmd_vel_topic_ = priv_nh.param<std::string>("cmd_vel", "cmd_vel");
    odom_topic_ = priv_nh.param<std::string>("odom", "odom");
    tf_topic_ = priv_nh.param<std::string>("tf", "/tf");
    published_cmd_topic_ = priv_nh.param<std::string>("publish_cmd", "publish_cmd");
    /* Subscribers */
    
    /* Publishers */

}

OavController::~OavController()
{
}
