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

    //Get the point cloud topic name
    ros::NodeHandle n;
    std::string pointCloudTopic;
    if(!n.getParam("pointCloudTopic",pointCloudTopic))
    {
        ROS_ERROR("Unable to retrieve point cloud topic name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    //RTABMAP measure service initialisation
    PointCloudServer ICPServer(pointCloudTopic,"/icp_filtered_point_cloud","/icp_server",ICPPointCloudFilter);

	ros::spin();
    return 0;
}