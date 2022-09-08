#include <pcl/io/pcd_io.h>

#include "robot_arm_3d_scan/PointCloudServer.h"

PointCloudServer::PointCloudServer(std::string rawPointCloudTopic, std::string filteredPointCloudTopic, std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter):MeasurementServer(), m_rawPointCloudTopic(rawPointCloudTopic), m_pointCloudFilter(pointCloudFilter), m_pointCloudCounter(0)
{
    //Launch point cloud ROS publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(filteredPointCloudTopic,10);
}

bool PointCloudServer::measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    //Get last published raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(m_rawPointCloudTopic);
    pcl::fromROSMsg(*rawPointCloud, *pointCloud);

    m_pointCloudCounter++;

    //Filter point cloud, save it and publish it
    m_pointCloudFilter(pointCloud);
    pcl::io::savePCDFileASCII(std::string(m_measurementServerStorageFolder + "PointCloud_" + std::to_string(m_pointCloudCounter) + ".pcd"), *pointCloud);
    m_pointCloudPublisher.publish(*pointCloud);

    return(true);
}

void SimplePointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    thresholdFilter(pointCloud,0.0,0.4);
    groundRemovalFilter(pointCloud,0.005);
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "point_cloud_server");  

    //Get the point cloud topic name
    ros::NodeHandle n;
    std::string pointCloudTopic;
    if(!n.getParam("pointCloudTopic",pointCloudTopic))
    {
        ROS_ERROR("Unable to retrieve point cloud topic name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    //Point cloud ervice initialisation
    PointCloudServer pointCloudServer(pointCloudTopic,"/filtered_point_cloud",SimplePointCloudFilter);

	ros::spin();
    return 0;
}