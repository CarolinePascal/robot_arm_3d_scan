#include "robot_arm_3d_scan/PointCloudServer.h"

PointCloudServer::PointCloudServer(std::string rawPointCloudTopic, std::string filteredPointCloudTopic, std::string pointCloudServerName, std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter):MeasurementServer(pointCloudServerName), m_rawPointCloudTopic(rawPointCloudTopic), m_pointCloudFilter(pointCloudFilter)
{
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(filteredPointCloudTopic,10);
}

bool PointCloudServer::measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(m_rawPointCloudTopic);
    pcl::fromROSMsg(*rawPointCloud, *pointCloud);

    m_pointCloudFilter(pointCloud);
    m_pointCloudPublisher.publish(*pointCloud);

    return(true);
}

void PointCloudServer::setFilter(std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter)
{
    m_pointCloudFilter = pointCloudFilter;
}