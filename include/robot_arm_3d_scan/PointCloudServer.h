/**
 * \file PointCloudServer.h
 * \brief Header file of the PointCloudServer class
 *
 * Header file of the PointCloudServer class - Defines the attributes and methods used to trigger point cloud measurements
 *
 */

#pragma once

#include <functional>
#include <robot_arm_tools/MeasurementServer.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include "pcl_filters/PCLFilters.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "filters/filter_chain.h"


 /*! \class PointCloudServer
  * \brief Class used to trigger point cloud measurements
  */
class PointCloudServer : public MeasurementServer
{
    public:
        /*!
         *  \brief Constructor
         */
        PointCloudServer();

        /*!
         *  \brief Destructor
         */
        ~PointCloudServer(){};

        /*!
         *  \brief Triggers a point cloud measurement
         */
        bool measure();

        void simplePointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, bool initialisation = false);

    protected:

        ros::NodeHandle m_privateNodeHandle;

    private:

        ros::Publisher m_pointCloudPublisher;   /*!< ROS measured & filtered point clouds publisher */

        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener;
        std::string m_pointCloudFrame;

        RobotVisualTools m_visualTools;
        double m_objectSize;
        geometry_msgs::Pose m_objectPose;

        bool m_groundRemoval;
        //TODO rename ?
        static int m_supportScanCounter;

        filters::FilterChain<sensor_msgs::PointCloud2> m_filterChain;

};