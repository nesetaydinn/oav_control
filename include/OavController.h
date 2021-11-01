#include "ros/ros.h"
#include "stdint.h"
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

#define DEBUG 1
#define FREQ_TO_PER(x) ((double)(1/x))

#define ODOM_PUB_FREQ_ 50L
#define TF_PUB_FREQ_ 50L
#define PUBLISHED_CMD_PUB_FREQ_ 50L 

typedef struct{
   uint8_t x;
   uint8_t y;
   uint8_t z;
}cmdVelType;


class OavController
{
private:
        ros::NodeHandle nh_;
        ros::NodeHandle priv_nh_;
        
        ros::Subscriber cmd_vel_sub_;

        ros::Publisher odom_pub_;
        ros::Publisher tf_pub_;
        ros::Publisher published_cmd_pub_;

        nav_msgs::Odometry odom_pub_msg_;
        tf::tfMessage tf_pub_msg_;
        geometry_msgs::TwistStamped published_cmd_pub_msg_;

        std::string robot_id_;
        std::string cmd_vel_topic_;
        std::string odom_topic_;
        std::string tf_topic_;
        std::string published_cmd_topic_;

        cmdVelType lineer_cmd_;
        cmdVelType angular_cmd_;

        ros::Timer odom_pub_timer_;
        ros::Timer tf_pub_timer_;
        ros::Timer published_cmd_pub_timer_;



    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    
    void odomPubTimerCallback(const ros::TimerEvent &event);
    void tfPubTimerCallback(const ros::TimerEvent &event);
    void publishedCmdPubTimerCallback(const ros::TimerEvent &event);

    void odomPublisher(void);
    void tfPublisher(void);
    void publishedCmdPublisher(void);

protected:

public:
    OavController(ros::NodeHandle &nh,ros::NodeHandle &priv_nh);
    ~OavController();
};
