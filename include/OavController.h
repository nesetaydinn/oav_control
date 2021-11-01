#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/TwistStamped.h"

/*
Subs:
    cmd_vel (geometry_msgs/Twist)
Publish:
    odom (nav_msgs/Odometry)
    /tf (tf/tfMessage)
    publish_cmd (geometry_msgs/TwistStamped)

*/

class OavController
{
private:
        ros::NodeHandle nh_;
        ros::NodeHandle priv_nh_;
        
        ros::Subscriber cmd_vel_;
        ros::Publisher odom_;
        ros::Publisher tf_;
        ros::Publisher published_cmd_;

        std::string robot_id_;
        std::string cmd_vel_topic_;
        std::string odom_topic_;
        std::string tf_topic_;
        std::string published_cmd_topic_;

protected:

public:
    OavController(ros::NodeHandle &nh,ros::NodeHandle &priv_nh);
    ~OavController();
};
