#include <pcl/io/pcd_io.h>

#include "robot_arm_3d_scan/PointCloudServer.h"

PointCloudServer::PointCloudServer(std::string rawPointCloudTopic, std::string filteredPointCloudTopic, std::string pointCloudServerName, std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter):MeasurementServer(pointCloudServerName,false,"/tmp/PointCloudMeasurements/"), m_rawPointCloudTopic(rawPointCloudTopic), m_pointCloudFilter(pointCloudFilter), m_pointCloudCounter(0)
{
    //Launch point cloud ROS publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(filteredPointCloudTopic,10);

    //Get the storage folder name
    if(!m_nodeHandle.getParam("storageFolderName",m_storageFolderName))
    {
        ROS_ERROR("Unable to retrieve positions file name !");
        throw std::runtime_error("MISSING PARAMETER");
    }
}

bool PointCloudServer::measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    //Get last published raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(m_rawPointCloudTopic);
    pcl::fromROSMsg(*rawPointCloud, *pointCloud);

    //Iterate...
    m_pointCloudCounter++;

    //Filter point cloud, save it and publish it
    m_pointCloudFilter(pointCloud);
    pcl::io::savePCDFileASCII(std::string(m_storageFolderName + "PointCloud_" + std::to_string(m_pointCloudCounter) + ".pcd"), *pointCloud);
    m_pointCloudPublisher.publish(*pointCloud);

    return(true);
}

void PointCloudServer::setFilter(std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter)
{
    m_pointCloudFilter = pointCloudFilter;
}