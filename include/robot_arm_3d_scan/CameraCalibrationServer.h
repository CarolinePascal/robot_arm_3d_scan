/**
 * \file CameraCalibrationServer.h
 * \brief Header file of the CameraCalibrationServer class
 *
 * Header file of the CameraCalibrationServer class - Defines the attributes and methods used to calibrate the transform between the robot flange frame and camera optical frame
 *
 */

#pragma once

#include <robot_arm_tools/MeasurementServer.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <moveit/handeye_calibration_target/handeye_target_base.h>
#include <moveit/handeye_calibration_solver/handeye_solver_base.h>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

 /*! \class CameraCalibrationServer
  * \brief Class used to calibrate the transform between the robot flange frame and camera optical frame
  */
class CameraCalibrationServer : public MeasurementServer
{
    public:

        /*!
         *  \brief Constructor
         */
        CameraCalibrationServer();

        /*!
         *  \brief Destructor
         */
        ~CameraCalibrationServer(){};

        /*!
         *  \brief Triggers a point cloud measurement
         */
        bool measure();

        /*!
         *  \brief Computes the camera sensor - robot flange transform based on previous measurements.
         *  \return The computed transform.
         */
        geometry_msgs::TransformStamped computeCameraFlangeTransform();

        void publishTransforms()
        {
            m_TargetCameraTransform.header.stamp = ros::Time::now();
            m_tfBroadcaster.sendTransform(m_TargetCameraTransform);

            if(m_CameraFlangeTransform.transform.rotation.x != 0 || m_CameraFlangeTransform.transform.rotation.y != 0 || m_CameraFlangeTransform.transform.rotation.z != 0 || m_CameraFlangeTransform.transform.rotation.w != 0)
            {
                m_CameraFlangeTransform.header.stamp = ros::Time::now();
                m_tfBroadcaster.sendTransform(m_CameraFlangeTransform);
            }
        }

    private:

        std::vector<Eigen::Isometry3d> m_FlangeBaseTransforms, m_TargetCameraTransforms;
        std::string m_flangeTF, m_cameraTF, m_baseTF;

        std::string m_cameraInfoTopic, m_cameraImageTopic;

        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener;
        tf2_ros::TransformBroadcaster m_tfBroadcaster;

        std::string m_targetType;
        int m_markersX, m_markersY;
        int m_markerSize, m_markerSeparation, m_markerBorder;
        double m_markerSizeReal, m_markerSeparationReal;
        int m_targetDict;

        pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase> m_targetLoader;
        boost::shared_ptr<moveit_handeye_calibration::HandEyeTargetBase> m_target;

        ros::Subscriber m_targetSubscriber;
        bool m_measurementDone;

        geometry_msgs::TransformStamped m_TargetCameraTransform, m_CameraFlangeTransform, m_FlangeBaseTransform;

        void targetCallback(const sensor_msgs::Image::ConstPtr& msg)
        {
            cv_bridge::CvImagePtr tmp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
                     
            if(m_target->detectTargetPose(tmp->image))
            {
                m_TargetCameraTransform = m_target->getTransformStamped(m_cameraTF);
                m_measurementDone = true;
            } 
        };

        ros::NodeHandle m_targetNodeHandle;  /*!< The ROS node handle for target measurements  */
        ros::CallbackQueue m_targetQueue ; /*!< The callback queue for target measurements */
        ros::AsyncSpinner m_targetSpinner;    /*!< The async. spinner for target measurements */

        int m_setupType, m_solverType;
        pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase> m_solverLoader;
        boost::shared_ptr<moveit_handeye_calibration::HandEyeSolverBase> m_solver;
};