/**
 * \file PointCloudServer.h
 * \brief Header file of the PointCloudServer class
 *
 * Header file of the PointCloudServer class - Defines the attributes and methods used to trigger point cloud measurements
 *
 */

#pragma once

#include <functional>
#include "measurement_tools/MeasurementServer.h"
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
         */
        PointCloudServer(std::string rawPointCloudTopic, std::string pointCloudServerName, std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter = emptyPCLFilter);

        /*!
         *  \brief Destructor
         */
        ~PointCloudServer(){};

        /*!
         *  \brief Triggers a point cloud measurement
         */
        bool measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        /*! 
         * \brief Adds a PCL filter to the point cloud measurement server
         */
        void setFilter(std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> pointCloudFilter);

    private:

        ros::Publisher m_pointCloudPublisher;   /*!< ROS measured & filtered point clouds publisher */
        std::string m_rawPointCloudTopic;   /*!< Name of the ROS topic on which raw point clouds are published */

        std::function<void (pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)> m_pointCloudFilter;  /*!< Additionnal PCL filter */
};