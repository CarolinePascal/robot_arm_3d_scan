#include "robot_arm_3Dscan/PointCloudServer.h"
#include "pcl_filters/PCLFilters.h"

void pointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    thresholdFilter(pointCloud,0.0,0.4);
    groundRemovalFilter(pointCloud,0.005);
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "ICP_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //RTABMAP measure service initialisation
    PointCloudServer measureService("/camera/depth/color/points");
    measureService.addFilter(pointCloudFilter);

    ros::waitForShutdown();
    return 0;
}

