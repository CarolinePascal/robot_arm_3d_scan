/**
 * \file PointCloudServer.h
 * \brief Header file of the PointCloudServer class
 *
 * Header file of the PointCloudServer class - Defines the attributes and methods used to trigger point cloud measurements
 *
 */

#pragma once

#include <functional>
#include "robot_arm_tools/MeasurementServer.h"
#include "pcl_filters/PCLFilters.h"

 /*! \class PointCloudServer
  * \brief Class used to trigger point cloud measurements
  */
class PointCloudServer : public MeasurementServer
{
    public:
        /*!
         *  \brief Constructor
         *  \param rawPointCloudTopic The name of the ROS topic on which raw point clouds are published
         *  \param filteredPointCloudTopic The name of the ROS topic on which to publish the filtered point clouds
         *  \param pointCloudFilter The filter to be applied on the measured point clouds
         */
        PointCloudServer(std::string rawPointCloudTopic, std::string filteredPointCloudTopic = "/filtered_point_cloud", std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter = PointCloudServer::emptyPCLFilter);

        /*!
         *  \brief Destructor
         */
        ~PointCloudServer(){};

        /*!
         *  \brief Triggers a point cloud measurement
         */
        bool measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    private:

        static void emptyPCLFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud){};

        ros::Publisher m_pointCloudPublisher;   /*!< ROS measured & filtered point clouds publisher */
        std::string m_rawPointCloudTopic;   /*!< Name of the ROS topic on which raw point clouds are published */

        std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> m_pointCloudFilter;  /*!< Additionnal PCL filter */

        //TODO GLOBAL !!
        int m_pointCloudCounter;    /*! Point clouds counter */
};