#include <pcl/io/pcd_io.h>

#include "robot_arm_3d_scan/PointCloudServer.h"

PointCloudServer::PointCloudServer(std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter):MeasurementServer(), m_pointCloudFilter(pointCloudFilter)
{
    //Launch point cloud ROS publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/filtered_point_cloud",1);
}

bool PointCloudServer::measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    //Get last published raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
    pcl::fromROSMsg(*rawPointCloud, *pointCloud);

    m_measurementServerCounter++;

    //Filter point cloud, save it and publish it
    m_pointCloudFilter(pointCloud);
    pcl::io::savePCDFileASCII(std::string(m_measurementServerStorageFolder + "PointCloud_" + std::to_string(m_measurementServerCounter) + ".pcd"), *pointCloud);
    m_pointCloudPublisher.publish(*pointCloud);

    return(true);
}

void SimplePointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    //Mandatory !!
    transformPointCloud(pointCloud,"world");
    try
    {
        //TODO Estimate ground height !!!
        groundRemovalFilter(pointCloud,0.05); 
    }
    catch(const std::exception& e)
    {
        continue;
    }   
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "point_cloud_server");  

    //Point cloud ervice initialisation
    PointCloudServer pointCloudServer(SimplePointCloudFilter);

	ros::spin();
    return 0;
}