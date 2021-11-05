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
    cmd_vel_sub_=nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic_,1,&OavController::cmdVelCallback,this);

    /* Publishers */
    odom_pub_=priv_nh_.advertise<nav_msgs::Odometry>("/" + robot_id_ + "/" + odom_topic_, 1);
    tf_pub_=priv_nh_.advertise<tf::tfMessage>("/" + robot_id_ + "/" + tf_topic_, 1);
    published_cmd_pub_=priv_nh_.advertise<geometry_msgs::TwistStamped>("/" + robot_id_ + "/" + published_cmd_topic_, 1);

    /* Timers */
    odom_pub_timer_= ros::Timer( priv_nh_.createTimer(ros::Duration(FREQ_TO_PER(ODOM_PUB_FREQ_)),&OavController::odomPubTimerCallback,this));
    tf_pub_timer_=  ros::Timer(priv_nh_.createTimer(ros::Duration(FREQ_TO_PER(TF_PUB_FREQ_)),&OavController::tfPubTimerCallback,this));
    published_cmd_pub_timer_=  ros::Timer(priv_nh_.createTimer(ros::Duration(FREQ_TO_PER(PUBLISHED_CMD_PUB_FREQ_)),&OavController::publishedCmdPubTimerCallback,this));
}

/* Subscribers Feedback */
void OavController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg){
    lineer_cmd_.x = msg->linear.x;
    lineer_cmd_.y = msg->linear.y;
    lineer_cmd_.z = msg->linear.z;
    
    angular_cmd_.x = msg->angular.x;
    angular_cmd_.y = msg->angular.y;
    angular_cmd_.z = msg->angular.z;
    
    #if DEBUG
    ROS_INFO("lineer velocity:\n x = %d\ty = %d\tz = %d",lineer_cmd_.x,lineer_cmd_.y,lineer_cmd_.z);
    ROS_INFO("angular velocity:\n x = %d\ty = %d\tz = %d",angular_cmd_.x,angular_cmd_.y,angular_cmd_.z);
    #endif
}

/* Timer Feedback */
void OavController::odomPubTimerCallback(const ros::TimerEvent &event){
  //  OavController::odomPublisher();
}

void OavController::tfPubTimerCallback(const ros::TimerEvent &event){
//    OavController::tfPublisher();
}

void OavController::publishedCmdPubTimerCallback(const ros::TimerEvent &event){
  //  OavController::publishedCmdPublisher();
}

/* Publishers Functions */


void OavController::odomPublisher(void){
    
}

void OavController::tfPublisher(void){

}

void OavController::publishedCmdPublisher(void){

}

/* Deconstructor */
OavController::~OavController()
{
}
