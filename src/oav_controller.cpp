#include "OavController.h"

int main(int argc, char **argv)
{   
    ros::init(argc,argv,"oav_control");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    OavController oavController_(nh, priv_nh);
    ros::spin();
    return 0;
}
