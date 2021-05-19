#include "robot_arm_3Dscan/RTABMAPServer.h"

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "RTABMAP_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //RTABMAP measure service initialisation
    RTABMAPServer measureService;

    ros::waitForShutdown();
    return 0;
}