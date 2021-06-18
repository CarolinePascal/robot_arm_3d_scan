#include "robot_arm_3d_scan/PointCloudServer.h"

void ICPPointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    thresholdFilter(pointCloud,0.0,0.4);
    groundRemovalFilter(pointCloud,0.005);
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "icp_server");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //RTABMAP measure service initialisation
    PointCloudServer ICPServer("/camera/depth/color/points","/icp_filtered_point_cloud","/icp_server",ICPPointCloudFilter);

    ros::waitForShutdown();
    return 0;
}