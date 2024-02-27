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

#include <dynamic_reconfigure/Reconfigure.h>

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
     *  \brief Computes the camera sensor to robot flange transform based on previous measurements.
     *  \return The computed transform.
     */
    geometry_msgs::TransformStamped computeFlangeCameraTransform();

    /*!
     * \brief Publishes the measured (camera - target) and computed (flange - camera) transforms
     */
    void publishTransforms()
    {
        // Publish camera - target transform (if it was computed in the constructor)
        m_CameraTargetTransform.header.stamp = ros::Time::now();
        m_tfBroadcaster.sendTransform(m_CameraTargetTransform);

        // Publish flange - camera transform (if it was computed !)
        if (m_FlangeCameraTransform.transform.rotation.x != 0 || m_FlangeCameraTransform.transform.rotation.y != 0 || m_FlangeCameraTransform.transform.rotation.z != 0 || m_FlangeCameraTransform.transform.rotation.w != 0)
        {
            m_FlangeCameraTransform.header.stamp = ros::Time::now();
            m_tfBroadcaster.sendTransform(m_FlangeCameraTransform);
        }
    }

private:
    geometry_msgs::TransformStamped m_CameraTargetTransform, m_FlangeCameraTransform, m_BaseFlangeTransform, m_FlangeCameraLinkTransform; /*!< The camera - target, flange - camera and base - flange transforms */
    Eigen::Isometry3d m_CameraCameraLinkIsometry;                                                                                         /*!< The camera - camera link isometry/transform */

    std::vector<Eigen::Isometry3d> m_BaseFlangeTransforms, m_CameraTargetTransforms; /*!< The vectors containing the base - flange and camera - target transforms */
    std::string m_flangeTF, m_cameraTF, m_baseTF, m_cameraLinkTF;                    /*!< TF names */

    std::string m_cameraInfoTopic, m_cameraImageTopic; /*!< Camera topics names */

    std::string m_cameraEmitterParameterName, m_cameraEmitterParameterService; /*!< IR emitter parameter name and service */
    ros::ServiceClient m_cameraEmitterClient;                                  /*!< IR emitter service client */
    dynamic_reconfigure::Reconfigure m_cameraEmitterClientSrv;                 /*!< IR emitter dynamic reconfigure service */

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;       /*!< The transform listener */
    tf2_ros::TransformBroadcaster m_tfBroadcaster; /*!< The transform broadcaster */

    std::string m_targetType;                             /*!< The type of the target used for calibration : Aruco, ChAruco, etc. */
    int m_markersX, m_markersY;                           /*!< Number of markers along the X and Y axis */
    int m_markerSize, m_markerSeparation, m_markerBorder; /*!< Theoretical marker size, separation width and border width */
    double m_markerSizeReal, m_markerSeparationReal;      /*!< Real marker size and separation width */
    int m_targetDict;                                     /*!< The dictionary used for the target markers */

    pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeTargetBase> m_targetLoader;
    boost::shared_ptr<moveit_handeye_calibration::HandEyeTargetBase> m_target; /*!< The target used for calibration */

    cv_bridge::CvImagePtr m_currentFrame, m_currentFrameAnnotated; /*!< The raw current frame and the current frame with the detected markers annotations */

    /*!
     * \brief Detects the markers on the camera target, and infer the camera - target transform
     * \param msg The image sensor message
     */
    void targetCallback(const sensor_msgs::Image::ConstPtr &msg)
    {
        m_currentFrame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        m_currentFrameAnnotated = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        if (m_target->detectTargetPose(m_currentFrameAnnotated->image))
        {
            m_CameraTargetTransform = m_target->getTransformStamped(m_cameraTF);
            m_measurementDone = true;
        }
    };

    ros::NodeHandle m_targetNodeHandle; /*!< The node handle for target measurements  */
    ros::Subscriber m_targetSubscriber; /*!< The subscriber for target measurements */
    bool m_measurementDone;             /*!< Flag indicating if a measurement has been done */
    ros::CallbackQueue m_targetQueue;   /*!< The callback queue for target measurements */
    ros::AsyncSpinner m_targetSpinner;  /*!< The async. spinner for target measurements */

    int m_setupType, m_solverType;
    pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase> m_solverLoader;
    boost::shared_ptr<moveit_handeye_calibration::HandEyeSolverBase> m_solver; /*!< The solver used for calibration */
};