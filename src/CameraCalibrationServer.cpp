#include "robot_arm_3d_scan/CameraCalibrationServer.h"

#include <pluginlib/class_loader.h>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include <robot_arm_tools/RobotVisualTools.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#define MIN_TRANSFORM 4

CameraCalibrationServer::CameraCalibrationServer() : MeasurementServer(), m_tfListener(m_tfBuffer), m_targetSpinner(1, &m_targetQueue), m_targetLoader("moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeTargetBase"), m_solverLoader("moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeSolverBase")
{
    // Retrieve TF names
    if (!m_nodeHandle.getParam("flangeLinkName", m_flangeTF))
    {
        ROS_WARN("Unable to retrieve robot flange TF name, defaulting to tool0 !");
        m_baseTF = "tool0";
    }
    if (!m_nodeHandle.getParam("baseLinkName", m_baseTF))
    {
        ROS_WARN("Unable to retrieve robot base TF name, defaulting to world !");
        m_baseTF = "world";
    }
    if (m_nodeHandle.getParam("cameraLinkName", m_cameraLinkTF))
    {
        ROS_WARN("Unable to retrieve camera link TF name, defaulting to camera_link !");
        m_cameraLinkTF = "camera_link";
    }

    // Retrieve camera topics
    ros::NodeHandle privateNodeHandle("~");
    if (!privateNodeHandle.getParam("cameraInfoTopic", m_cameraInfoTopic))
    {
        ROS_WARN("Unable to retrieve camera info topic name !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    if (!privateNodeHandle.getParam("cameraImageTopic", m_cameraImageTopic))
    {
        ROS_WARN("Unable to retrieve camera image topic name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    // Setup camera IR emitter ON/OFF service
    privateNodeHandle.param<std::string>("cameraEmitterParameter", m_cameraEmitterParameterService, "");
    std::size_t tmp = m_cameraEmitterParameterService.find_last_of("/\\");
    m_cameraEmitterParameterName = m_cameraEmitterParameterService.substr(tmp + 1);
    m_cameraEmitterParameterService = m_cameraEmitterParameterService.substr(0, tmp) + "/set_parameters";

    if (m_cameraEmitterParameterName != "")
    {
        // Create service
        m_cameraEmitterClient = m_nodeHandle.serviceClient<dynamic_reconfigure::Reconfigure>(m_cameraEmitterParameterService);
        // ROS_WARN("%s %s",m_cameraEmitterParameterService.c_str(),m_cameraEmitterParameterName.c_str());
        m_cameraEmitterClient.waitForExistence();

        // Configure default request
        dynamic_reconfigure::IntParameter cameraEmitterClientParam;
        dynamic_reconfigure::Config cameraEmitterClientConf;
        cameraEmitterClientParam.name = m_cameraEmitterParameterName;
        cameraEmitterClientConf.ints.push_back(cameraEmitterClientParam);
        m_cameraEmitterClientSrv.request.config = cameraEmitterClientConf;
    }

    // Retrieve camera target parameters
    privateNodeHandle.param<std::string>("targetType", m_targetType, "Aruco");
    privateNodeHandle.param<int>("markersX", m_markersX, 3);
    privateNodeHandle.param<int>("markersY", m_markersY, 4);
    privateNodeHandle.param<int>("markerSize", m_markerSize, 200);
    privateNodeHandle.param<int>("markerSeparation", m_markerSeparation, 20);
    privateNodeHandle.param<int>("markerBorder", m_markerBorder, 1);
    privateNodeHandle.param<double>("markerSizeReal", m_markerSizeReal, 0.2);
    privateNodeHandle.param<double>("markerSeparationReal", m_markerSeparationReal, 0.02);
    privateNodeHandle.param<int>("targetDict", m_targetDict, 1);

    // Create camera target
    m_target = m_targetLoader.createInstance("HandEyeTarget/" + m_targetType);

    // Set camera target parameters
    bool parametersSet;
    parametersSet = m_target->setParameter("markers, X", m_markersX);
    parametersSet = parametersSet && m_target->setParameter("markers, Y", m_markersY);
    parametersSet = parametersSet && m_target->setParameter("marker size (px)", m_markerSize);
    parametersSet = parametersSet && m_target->setParameter("marker separation (px)", m_markerSeparation);
    parametersSet = parametersSet && m_target->setParameter("marker border (bits)", m_markerBorder);
    parametersSet = parametersSet && m_target->setParameter("ArUco dictionary", m_targetDict);
    parametersSet = parametersSet && m_target->setParameter("measured marker size (m)", m_markerSizeReal);
    parametersSet = parametersSet && m_target->setParameter("measured separation (m)", m_markerSeparationReal);

    if (!parametersSet)
    {
        ROS_WARN("Could not set camera target parameters !");
        throw std::runtime_error("CAMERA TARGET PARAMETERS ERROR");
    }

    // Initialize camera target
    if (!m_target->initialize())
    {
        ROS_WARN("Could not initialize camera target !");
        throw std::runtime_error("CAMERA TARGET INITIALIZATION ERROR");
    }

    // Wait for the camera to publish a camera info message
    sensor_msgs::CameraInfoConstPtr cameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(m_cameraInfoTopic, ros::Duration(10));

    m_target->setCameraIntrinsicParams(cameraInfo);
    m_cameraTF = cameraInfo->header.frame_id;

    // Create storage file for camera info message
    boost::filesystem::path pathCameraInfo(m_measurementServerStorageFolder + "CameraInfo.yaml");
    boost::filesystem::create_directories(pathCameraInfo.parent_path());
    boost::filesystem::ofstream{pathCameraInfo};

    YAML::Node cameraInfoStorageFile = YAML::LoadFile(m_measurementServerStorageFolder + "CameraInfo.yaml");

    cameraInfoStorageFile["frame_id"] = cameraInfo->header.frame_id;
    cameraInfoStorageFile["width"] = cameraInfo->width;
    cameraInfoStorageFile["height"] = cameraInfo->height;
    cameraInfoStorageFile["distortion_model"] = cameraInfo->distortion_model;
    for (std::size_t i = 0; i < cameraInfo->D.size(); i++)
    {
        cameraInfoStorageFile["D"].push_back(cameraInfo->D[i]);
    }
    for (std::size_t i = 0; i < cameraInfo->K.size(); i++)
    {
        cameraInfoStorageFile["K"].push_back(cameraInfo->K[i]);
    }
    for (std::size_t i = 0; i < cameraInfo->R.size(); i++)
    {
        cameraInfoStorageFile["R"].push_back(cameraInfo->R[i]);
    }
    for (std::size_t i = 0; i < cameraInfo->P.size(); i++)
    {
        cameraInfoStorageFile["P"].push_back(cameraInfo->P[i]);
    }
    cameraInfoStorageFile["binning_x"] = cameraInfo->binning_x;
    cameraInfoStorageFile["binning_y"] = cameraInfo->binning_y;
    cameraInfoStorageFile["roi"]["x_offset"] = cameraInfo->roi.x_offset;
    cameraInfoStorageFile["roi"]["y_offset"] = cameraInfo->roi.y_offset;
    cameraInfoStorageFile["roi"]["height"] = cameraInfo->roi.height;
    cameraInfoStorageFile["roi"]["width"] = cameraInfo->roi.width;
    cameraInfoStorageFile["roi"]["do_rectify"] = cameraInfo->roi.do_rectify;

    std::ofstream fout(m_measurementServerStorageFolder + "CameraInfo.yaml");
    fout << cameraInfoStorageFile;
    fout.close();

    // Get camera link - camera transform
    try
    {
        geometry_msgs::TransformStamped tmp = m_tfBuffer.lookupTransform(m_cameraTF, m_cameraLinkTF, ros::Time(0), ros::Duration(5.0));
        m_CameraCameraLinkIsometry = tf2::transformToEigen(tmp.transform);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not retrieve camera link - camera transform, assuming identity !");
        m_CameraCameraLinkIsometry = Eigen::Isometry3d::Identity();
    }

    // Setup subscriber for camera target detection and hand-eye calibration
    m_targetNodeHandle.setCallbackQueue(&m_targetQueue);
    m_targetSubscriber = m_targetNodeHandle.subscribe<sensor_msgs::Image>(m_cameraImageTopic, 1, &CameraCalibrationServer::targetCallback, this);

    // Initialize hand-eye calibration solver
    privateNodeHandle.param<int>("solverType", m_solverType, 0);
    privateNodeHandle.param<int>("setupType", m_setupType, 1);

    m_solver = m_solverLoader.createInstance("crigroup");
    m_solver->initialize();

    // Create storage file for flange - camera and camera link - camera transform
    boost::filesystem::path transformPath(m_measurementServerStorageFolder + "FlangeCameraTransform.yaml");
    boost::filesystem::create_directories(transformPath.parent_path());
    boost::filesystem::ofstream{transformPath};

    transformPath = boost::filesystem::path(m_measurementServerStorageFolder + "FlangeCameraLinkTransform.yaml");
    boost::filesystem::ofstream{transformPath};

    // Get initial camera target location
    ROS_WARN("Getting initial camera target location...");

    double startTime = ros::WallTime::now().toSec();
    double interStartTime = ros::WallTime::now().toSec();
    m_measurementDone = false;
    m_targetSpinner.start();

    while (!m_measurementDone && ros::WallTime::now().toSec() - startTime < 60.0)
    {
        if (ros::WallTime::now().toSec() - interStartTime > 5.0)
        {
            interStartTime = ros::WallTime::now().toSec();
            ROS_WARN("Cannot detect target, please move the robot in a viable position !");
        }
        continue;
    }

    m_targetSpinner.stop();

    if (!m_measurementDone)
    {
        ROS_WARN("Could not retrieve target - camera sensor transform !");
        throw std::runtime_error("INVALID TRANSFORM");
    }
}

bool CameraCalibrationServer::measure()
{
    // Stop camera infrared emitter
    if (m_cameraEmitterParameterName != "")
    {
        m_cameraEmitterClientSrv.request.config.ints[0].value = 0;
        if (!m_cameraEmitterClient.call(m_cameraEmitterClientSrv))
        {
            ROS_WARN("Could not stop camera infrared emitter !");
        }
        else
        {
            ros::WallDuration(1.0).sleep();
        }
    }

    // Get new target pose
    double startTime = ros::WallTime::now().toSec();

    m_measurementDone = false;
    m_targetSpinner.start();
    while (!m_measurementDone && ros::WallTime::now().toSec() - startTime < 5.0)
    {
        continue;
    }
    m_targetSpinner.stop();

    // Restart camera infrared emitter
    if (m_cameraEmitterParameterName != "")
    {
        m_cameraEmitterClientSrv.request.config.ints[0].value = 1;
        if (!m_cameraEmitterClient.call(m_cameraEmitterClientSrv))
        {
            ROS_WARN("Could not start camera infrared emitter !");
        }
    }

    if (!m_measurementDone)
    {
        ROS_WARN("Could not retrieve target - camera sensor transform !");
        return (false);
    }

    // Get base - flange transform
    try
    {
        m_BaseFlangeTransform = m_tfBuffer.lookupTransform(m_baseTF, m_flangeTF, ros::Time(0), ros::Duration(5.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not retrieve robot flange - base transform !");
        return (false);
    }

    m_BaseFlangeTransforms.push_back(tf2::transformToEigen(m_BaseFlangeTransform.transform));
    m_CameraTargetTransforms.push_back(tf2::transformToEigen(m_CameraTargetTransform.transform)); // Remark : the camera - target transform was set in the camera target detection callback

    // Compute flange - camera transform (hand-eye calibration)
    try
    {
        m_FlangeCameraTransform = computeFlangeCameraTransform();
    }
    catch (const std::exception &ex)
    {
        ROS_WARN("Robot flange - camera sensor transform computation failed - Returning default value !");
        m_FlangeCameraTransform = m_tfBuffer.lookupTransform(m_flangeTF, m_cameraTF, ros::Time(0), ros::Duration(5.0));
    }

    // Fill storage file for flange - camera transform + reprojection error
    YAML::Node FlangeCameraTransformStorageFile = YAML::LoadFile(m_measurementServerStorageFolder + "FlangeCameraTransform.yaml");
    FlangeCameraTransformStorageFile["FlangeCameraTransform_" + std::to_string(m_measurementServerCounter)]["x"] = m_FlangeCameraTransform.transform.translation.x;
    FlangeCameraTransformStorageFile["FlangeCameraTransform_" + std::to_string(m_measurementServerCounter)]["y"] = m_FlangeCameraTransform.transform.translation.y;
    FlangeCameraTransformStorageFile["FlangeCameraTransform_" + std::to_string(m_measurementServerCounter)]["z"] = m_FlangeCameraTransform.transform.translation.z;

    tf2::Quaternion tmpQuaternion;
    tf2::Matrix3x3 tmpMatrix;
    double roll, pitch, yaw;
    tf2::fromMsg(m_FlangeCameraTransform.transform.rotation, tmpQuaternion);
    tmpMatrix = tf2::Matrix3x3(tmpQuaternion);
    tmpMatrix.getRPY(roll, pitch, yaw);

    FlangeCameraTransformStorageFile["FlangeCameraTransform_" + std::to_string(m_measurementServerCounter)]["rx"] = roll;
    FlangeCameraTransformStorageFile["FlangeCameraTransform_" + std::to_string(m_measurementServerCounter)]["ry"] = pitch;
    FlangeCameraTransformStorageFile["FlangeCameraTransform_" + std::to_string(m_measurementServerCounter)]["rz"] = yaw;

    // Calculate reprojection error
    std::pair<double, double> reprojectionError = m_solver->getReprojectionError(m_BaseFlangeTransforms, m_CameraTargetTransforms, tf2::transformToEigen(m_FlangeCameraTransform), moveit_handeye_calibration::SensorMountType(m_setupType));

    // BUG : https://github.com/ros-planning/moveit_calibration/issues/136
    double positionError = reprojectionError.second;
    double orientationError = reprojectionError.first;

    FlangeCameraTransformStorageFile["reprojectionError_" + std::to_string(m_measurementServerCounter)]["position"] = positionError;
    FlangeCameraTransformStorageFile["reprojectionError_" + std::to_string(m_measurementServerCounter)]["orientation"] = orientationError;

    // Close file
    std::ofstream fout(m_measurementServerStorageFolder + "FlangeCameraTransform.yaml");
    fout << FlangeCameraTransformStorageFile;
    fout.close();

    // Fill storage file for flange - camera link transform
    YAML::Node FlangeCameraLinkTransformStorageFile = YAML::LoadFile(m_measurementServerStorageFolder + "FlangeCameraLinkTransform.yaml");

    // FlangeCameraLinkTransform = FlangeCameraTransform * CameraCameraLinkTransform
    Eigen::Isometry3d tmp;
    tf2::doTransform(m_CameraCameraLinkIsometry, tmp, m_FlangeCameraTransform);
    m_FlangeCameraLinkTransform = tf2::eigenToTransform(tmp);

    FlangeCameraLinkTransformStorageFile["FlangeCameraLinkTransform"]["x"] = m_FlangeCameraLinkTransform.transform.translation.x;
    FlangeCameraLinkTransformStorageFile["FlangeCameraLinkTransform"]["y"] = m_FlangeCameraLinkTransform.transform.translation.y;
    FlangeCameraLinkTransformStorageFile["FlangeCameraLinkTransform"]["z"] = m_FlangeCameraLinkTransform.transform.translation.z;

    tf2::fromMsg(m_FlangeCameraLinkTransform.transform.rotation, tmpQuaternion);
    tmpMatrix = tf2::Matrix3x3(tmpQuaternion);
    tmpMatrix.getRPY(roll, pitch, yaw);

    FlangeCameraLinkTransformStorageFile["FlangeCameraLinkTransform"]["rx"] = roll;
    FlangeCameraLinkTransformStorageFile["FlangeCameraLinkTransform"]["ry"] = pitch;
    FlangeCameraLinkTransformStorageFile["FlangeCameraLinkTransform"]["rz"] = yaw;

    // Close file
    std::ofstream foutFinal(m_measurementServerStorageFolder + "FlangeCameraLinkTransform.yaml");
    foutFinal << FlangeCameraLinkTransformStorageFile;
    foutFinal.close();

    // Save current frame (for post-processing)
    if (!cv::imwrite(cv::String(m_measurementServerStorageFolder + "Frame_" + std::to_string(m_measurementServerCounter) + ".png"), m_currentFrame->image))
    {
        ROS_WARN("Could not save current frame !");
    }
    else
    {
        ROS_INFO("Current frame saved at %s", (m_measurementServerStorageFolder + "Frame_" + std::to_string(m_measurementServerCounter) + ".png").c_str());
    }

    if (m_measurementServerDisplay)
    {
        ROS_INFO("Camera - Flange transform : X: %f, Y: %f, Z: %f, RX: %f, RY: %f, RZ: %f", m_FlangeCameraTransform.transform.translation.x, m_FlangeCameraTransform.transform.translation.y, m_FlangeCameraTransform.transform.translation.z, roll, pitch, yaw);
        ROS_INFO("Reprojection error : %f m, %f rad", positionError, orientationError);

        // Display frame with feature points
        cv::imshow("Current frame", m_currentFrameAnnotated->image);
        cv::waitKey(2000);
        cv::destroyAllWindows();
    }

    return (true);
}

geometry_msgs::TransformStamped CameraCalibrationServer::computeFlangeCameraTransform()
{
    // Check if the number of transforms is valid
    if (m_BaseFlangeTransforms.size() != m_CameraTargetTransforms.size())
    {
        ROS_WARN("Uneven transform number for solver !");
        throw std::runtime_error("INVALID TRANSFORM");
    }
    else if (m_BaseFlangeTransforms.size() < MIN_TRANSFORM)
    {
        ROS_WARN("Too small transform number for solver !");
        throw std::runtime_error("INVALID TRANSFORM");
    }

    // Solve the hand-eye calibration problem
    else
    {
        std::string errorMessage;
        bool output = m_solver->solve(m_BaseFlangeTransforms, m_CameraTargetTransforms, moveit_handeye_calibration::SensorMountType(m_setupType), m_solver->getSolverNames()[m_solverType], &errorMessage);

        if (!output)
        {
            ROS_WARN("Solver failed with error %s !", errorMessage.c_str());
            throw std::runtime_error("INVALID TRANSFORM");
        }

        geometry_msgs::TransformStamped FlangeCameraTransform = tf2::eigenToTransform(m_solver->getCameraRobotPose());
        FlangeCameraTransform.header.frame_id = m_flangeTF;
        FlangeCameraTransform.child_frame_id = "calibrated_camera_sensor_frame";

        return (FlangeCameraTransform);
    }
}

int main(int argc, char *argv[])
{
    // ROS node initialisation
    ros::init(argc, argv, "camera_calibration_server");

    // Point cloud service initialisation
    CameraCalibrationServer cameraCalibrationServer;

    while (ros::ok())
    {
        cameraCalibrationServer.publishTransforms();
        ros::spinOnce();
    }

    return 0;
}