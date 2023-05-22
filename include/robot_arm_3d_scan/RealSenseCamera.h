/**
 * \file RealSenseCamera.h
 * \brief Header file of the RealSenseCamera class
 *
 * Header file of the RealSenseCamera class - Defines the attributes and methods used to configure and publish a RealSense camera depth measurements
 *
 */

#pragma once

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "robot_arm_3d_scan/FloatParameters.h"
#include "robot_arm_3d_scan/StringParameter.h"

 /*! \class RealSenseCamera
  * \brief Class sed to configure and publish a RealSense camera depth measurements
  */
class RealSenseCamera
{
    public:

    /*!
     *  \brief Constructor
     */
    RealSenseCamera();

    /*!
     *  \brief Destructor
     */
    ~RealSenseCamera(){};

    /*!
     *  \brief Publishes a point cloud
     */
    void publishPointcloud();

    /*!
     *  \brief Dynamically changes the disparity shift parameter of the camera
     */
    bool disparityShiftService(robot_arm_3d_scan::FloatParameters::Request& req, robot_arm_3d_scan::FloatParameters::Response& res);

    /*!
     *  \brief Dynamically changes the thresholds filter parameters of the camera
     */
    bool thresholdFilterService(robot_arm_3d_scan::FloatParameters::Request& req, robot_arm_3d_scan::FloatParameters::Response& res);

    private:

    //ROS attributes
    ros::NodeHandle m_nodeHandle;   /*! ROS node handle*/
    ros::Publisher m_pointCloudPublisher;  /*! ROS point cloud publisher*/

    //RealSense camera attributes
    rs2::pipeline m_pipe;
    rs2::frameset m_frames;
    rs2::pointcloud m_pc; 
    rs2::points m_points;
    rs2::frame m_color;
    rs2::frame m_depth;
    rs2::align m_align;

    rs2::device m_device;

    //RealSense -> PCL conversion attribute
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pclPointCloud;

    //Preset
    std::string m_presetPath;

    //Disparity shift
    int m_disparityShift;
    ros::ServiceServer m_disparityShiftService;

    //Depth units
    double m_depthUnits;

    //Filters
    std::vector<std::string> m_filtersNames;
    std::vector<rs2::filter> m_filters;

    //Temporal filter 

    //Threshold filter
    std::vector<double> m_thresholdFilterParameters;
    ros::ServiceServer m_thresholdFilterService;

    bool m_mutex;
};
