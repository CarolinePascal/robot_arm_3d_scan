#include "robot_arm_3d_scan/CameraCalibrationServer.h"

#include <pluginlib/class_loader.h>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include <robot_arm_tools/RobotVisualTools.h>

#define MIN_TRANSFORM 4

CameraCalibrationServer::CameraCalibrationServer() : MeasurementServer(), m_tfListener(m_tfBuffer), m_targetSpinner(1,&m_targetQueue), m_targetLoader("moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeTargetBase"), m_solverLoader("moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeSolverBase")
{
    //Retrieve TF names
    if(!m_nodeHandle.getParam("flangeLinkName",m_flangeTF))
    {
        ROS_WARN("Unable to retrieve robot flange TF name, defaulting to tool0 !");
        m_baseTF = "tool0";
    }
    if(!m_nodeHandle.getParam("baseLinkName",m_baseTF))
    {
        ROS_WARN("Unable to retrieve robot base TF name, defaulting to world !");
        m_baseTF = "world";
    }

    //Setup camera
    ros::NodeHandle privateNodeHandle("~");
    if(!privateNodeHandle.getParam("cameraInfoTopic",m_cameraInfoTopic))
    {
        ROS_WARN("Unable to retrieve camera info topic name !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    if(!privateNodeHandle.getParam("cameraImageTopic",m_cameraImageTopic))
    {
        ROS_WARN("Unable to retrieve camera image topic name !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    //Setup target
    privateNodeHandle.param<std::string>("targetType", m_targetType, "Aruco");
    privateNodeHandle.param<int>("markersX", m_markersX, 3);
    privateNodeHandle.param<int>("markersY", m_markersY, 4);
    privateNodeHandle.param<int>("markerSize", m_markerSize, 200);
    privateNodeHandle.param<int>("markerSeparation", m_markerSeparation, 20);
    privateNodeHandle.param<int>("markerBorder", m_markerBorder, 1);
    privateNodeHandle.param<double>("markerSizeReal", m_markerSizeReal, 0.2);
    privateNodeHandle.param<double>("markerSeparationReal", m_markerSeparationReal, 0.02);
    privateNodeHandle.param<int>("targetDict", m_targetDict, 1);

    m_target = m_targetLoader.createInstance("HandEyeTarget/" + m_targetType);

    //TODO Check if all set + initialize return true
    m_target->setParameter("markers, X", m_markersX);
    m_target->setParameter("markers, Y", m_markersY);
    m_target->setParameter("marker size (px)", m_markerSize);
    m_target->setParameter("marker separation (px)", m_markerSeparation);
    m_target->setParameter("marker border (bits)", m_markerBorder);
    m_target->setParameter("ArUco dictionary", m_targetDict);
    m_target->setParameter("measured marker size (m)", m_markerSizeReal);
    m_target->setParameter("measured separation (m)", m_markerSeparationReal);

    m_target->initialize();

    //Wait for the camera to publish a camera info message
    sensor_msgs::CameraInfoConstPtr cameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(m_cameraInfoTopic, ros::Duration(10));

    m_target->setCameraIntrinsicParams(cameraInfo);
    m_cameraTF = cameraInfo->header.frame_id;

    m_targetNodeHandle.setCallbackQueue(&m_targetQueue);
    m_targetSubscriber = m_targetNodeHandle.subscribe<sensor_msgs::Image>(m_cameraImageTopic, 1, &CameraCalibrationServer::targetCallback, this);

    privateNodeHandle.param<int>("solverType", m_solverType, 0);
    privateNodeHandle.param<int>("setupType", m_setupType, 1);

    m_solver = m_solverLoader.createInstance("crigroup");
    
    m_solver->initialize();

    //Creating storage file if needed, and emptying it
    YAML::Node CameraFlangeTransformStorageFile;
    try
    {
        CameraFlangeTransformStorageFile = YAML::LoadFile(m_measurementServerStorageFolder + "CameraFlangeTransform.yaml");
    }
    catch(const std::exception& e)
    {
        boost::filesystem::path path(m_measurementServerStorageFolder + "CameraFlangeTransform.yaml");
        boost::filesystem::create_directories(path.parent_path());
        boost::filesystem::ofstream {path};

        CameraFlangeTransformStorageFile = YAML::LoadFile(m_measurementServerStorageFolder + "CameraFlangeTransform.yaml");
    }

    if(CameraFlangeTransformStorageFile["CameraFlangeTransform"]) 
    {
        CameraFlangeTransformStorageFile.remove("CameraFlangeTransform");
        CameraFlangeTransformStorageFile.remove("reprojectionError");
    }

    ROS_WARN("Getting initial target location...");

    double startTime = ros::WallTime::now().toSec();
    double interStartTime = ros::WallTime::now().toSec();
    m_measurementDone = false;
    m_targetSpinner.start();

    while(!m_measurementDone && ros::WallTime::now().toSec() - startTime < 60.0)
    {
        if(ros::WallTime::now().toSec() - interStartTime > 5.0)
        {
            interStartTime = ros::WallTime::now().toSec();
            ROS_WARN("Cannot detect target, please move the robot in a viable position !");
        }
        continue;
    }

    m_targetSpinner.stop();

    if(!m_measurementDone)
    {
        ROS_WARN("Could not retrieve target - camera sensor transform !");
        throw std::runtime_error("INVALID TRANSFORM");  
    }

    /*
    RobotVisualTools robotVisualTools;

    publishTransforms();
    geometry_msgs::TransformStamped tmp = m_tfBuffer.lookupTransform("world", "handeye_target", ros::Time(0), ros::Duration(5.0));

    geometry_msgs::Pose targetPose;
    targetPose.position.x = tmp.transform.translation.x;
    targetPose.position.y = tmp.transform.translation.y;
    targetPose.position.z = tmp.transform.translation.z;
    targetPose.orientation = tmp.transform.rotation;
    robotVisualTools.addBox("target",targetPose,0.5,0.5,0.005,false,false);
    */
}

bool CameraCalibrationServer::measure()
{
    //Get new target pose
    double startTime = ros::WallTime::now().toSec();

    m_measurementDone = false;
    m_targetSpinner.start();
    while(!m_measurementDone && ros::WallTime::now().toSec() - startTime < 5.0)
    {
        continue;
    }
    m_targetSpinner.stop();

    if(!m_measurementDone)
    {
        ROS_WARN("Could not retrieve target - camera sensor transform !");
        return(false);
    }

    try
    {
        m_FlangeBaseTransform = m_tfBuffer.lookupTransform(m_baseTF, m_flangeTF, ros::Time(0), ros::Duration(5.0));
    } 
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("Could not retrieve robot flange - base transform !");
        return(false);
    }

    m_FlangeBaseTransforms.push_back(tf2::transformToEigen(m_FlangeBaseTransform.transform));
    m_TargetCameraTransforms.push_back(tf2::transformToEigen(m_TargetCameraTransform.transform));

    bool success;
    try
    {
        m_CameraFlangeTransform = computeCameraFlangeTransform();
        success = true;
    }

    catch (const std::exception& ex) 
    {
        ROS_WARN("Robot flange - camera sensor transform computation failed - Returning default value !");
        m_CameraFlangeTransform = m_tfBuffer.lookupTransform(m_flangeTF, m_cameraTF, ros::Time(0), ros::Duration(5.0));
        success = false;
    }

    YAML::Node CameraFlangeTransformStorageFile = YAML::LoadFile(m_measurementServerStorageFolder + "CameraFlangeTransform.yaml");       
    CameraFlangeTransformStorageFile["CameraFlangeTransform"]["x"] = m_CameraFlangeTransform.transform.translation.x;
    CameraFlangeTransformStorageFile["CameraFlangeTransform"]["y"] = m_CameraFlangeTransform.transform.translation.y;
    CameraFlangeTransformStorageFile["CameraFlangeTransform"]["z"] = m_CameraFlangeTransform.transform.translation.z;

    tf2::Quaternion tmpQuaternion;
    tf2::Matrix3x3 tmpMatrix;
    double roll, pitch, yaw;
    tf2::fromMsg(m_CameraFlangeTransform.transform.rotation,tmpQuaternion);
    tmpMatrix = tf2::Matrix3x3(tmpQuaternion);
    tmpMatrix.getRPY(roll,pitch,yaw);

    CameraFlangeTransformStorageFile["CameraFlangeTransform"]["rx"] = roll;
    CameraFlangeTransformStorageFile["CameraFlangeTransform"]["ry"] = pitch;
    CameraFlangeTransformStorageFile["CameraFlangeTransform"]["rz"] = yaw;

    // Calculate reprojection error
    std::pair<double, double> reprojectionError  = m_solver->getReprojectionError(m_FlangeBaseTransforms,m_TargetCameraTransforms,tf2::transformToEigen(m_CameraFlangeTransform), moveit_handeye_calibration::SensorMountType(m_setupType));
    
    CameraFlangeTransformStorageFile["reprojectionError"]["position"] = reprojectionError.first;
    CameraFlangeTransformStorageFile["reprojectionError"]["orientation"] = reprojectionError.second;

    //Close file
    std::ofstream fout(m_measurementServerStorageFolder + "CameraFlangeTransform.yaml");   
    fout << CameraFlangeTransformStorageFile;

    if(m_measurementServerDisplay)
    {
        ROS_INFO("Camera - Flange transform : X: %f, Y: %f, Z: %f, RX: %f, RY: %f, RZ: %f", m_CameraFlangeTransform.transform.translation.x, m_CameraFlangeTransform.transform.translation.y, m_CameraFlangeTransform.transform.translation.z, roll, pitch, yaw);
        ROS_INFO("Reprojection error : %f m, %f rad", reprojectionError.first, reprojectionError.second);
    }

    return(success);
}

geometry_msgs::TransformStamped CameraCalibrationServer::computeCameraFlangeTransform()
{
    if(m_FlangeBaseTransforms.size() != m_TargetCameraTransforms.size())
    {
        ROS_WARN("Uneven transform number for solver !");
        throw std::runtime_error("INVALID TRANSFORM");  
    }
    else if(m_FlangeBaseTransforms.size() < MIN_TRANSFORM)
    {
        ROS_WARN("Too small transform number for solver !");
        throw std::runtime_error("INVALID TRANSFORM");  
    }
    else
    {
        std::string errorMessage;
        bool output = m_solver->solve(m_FlangeBaseTransforms,m_TargetCameraTransforms,moveit_handeye_calibration::SensorMountType(m_setupType),m_solver->getSolverNames()[m_solverType],&errorMessage);
        if(!output)
        {
            ROS_WARN("Solver failed with error %s !", errorMessage.c_str());
            throw std::runtime_error("INVALID TRANSFORM");  
        }

        geometry_msgs::TransformStamped CameraFlangeTransform = tf2::eigenToTransform(m_solver->getCameraRobotPose());
        CameraFlangeTransform.header.frame_id = m_flangeTF;
        CameraFlangeTransform.child_frame_id = "calibrated_camera_sensor_frame";
        
        return(CameraFlangeTransform);
    }
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "camera_calibration_server");  

    //Point cloud service initialisation
    CameraCalibrationServer cameraCalibrationServer;

    while(ros::ok())
    {
        cameraCalibrationServer.publishTransforms();
        ros::spinOnce();
    }

    return 0;
}