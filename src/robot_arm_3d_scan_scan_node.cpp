#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <std_srvs/Empty.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <fstream>
#include <cmath>

#include "robot_arm_3d_scan/FloatParameters.h"

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_3d_scan_spheric_scan_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation TODO More generic approach
    Robot robot;
    RobotVisualTools robotVisualTools;

    //Move the robot to its initial configuration
    robot.setAcceleration(0.05);
    robot.setVelocity(0.1);

    //Get the object radius, pose and the trajectory radius
    std::vector<double> objectPoseArray;
    double objectSize;
    double radiusTrajectory;
    int trajectoryStepsNumber;

    ros::NodeHandle n;
    if(!n.getParam("objectPose",objectPoseArray))
    {
        ROS_ERROR("Unable to retrieve measured object pose !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if(!n.getParam("objectSize",objectSize))
    {
        ROS_ERROR("Unable to retrieve measured object radius !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if(!n.getParam("radiusTrajectory",radiusTrajectory))
    {
        ROS_ERROR("Unable to retrieve trajectory radius !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if(!n.getParam("trajectoryStepsNumber",trajectoryStepsNumber))
    {
        ROS_ERROR("Unable to retrieve trajectory steps number !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    geometry_msgs::Pose objectPose;
    objectPose.position.x = objectPoseArray[0];
    objectPose.position.y = objectPoseArray[1];
    objectPose.position.z = objectPoseArray[2];

    //Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;
    
    //Initial measurement

    /*
    if(robot.getToolName() == "cameraD435" || robot.getToolName() == "cameraD405")
    {
        ros::ServiceClient thresholdClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("threshold_filter");
        ros::ServiceClient disparityShiftClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("disparity_shift");
    
        robot_arm_3d_scan::FloatParameters thresholdSrv;
        robot_arm_3d_scan::FloatParameters disparityShiftSrv;

        thresholdSrv.request.parameters = std::vector<double>{0.0,2*radiusTrajectory};
        disparityShiftSrv.request.parameters = std::vector<double>{0.0};

        if(thresholdClient.call(thresholdSrv) && disparityShiftClient.call(disparityShiftSrv))
        {
            ROS_INFO("Initial measurement parameters set up...");
        }    
    }

    geometry_msgs::Pose initialPose;

    for(int i = 0; i < 4; i++)
    {
        initialPose = sphericPose(objectPose,radiusTrajectory*1.5,M_PI*i/2,0);
        if(!robot.isReachable(initialPose))
        {
            initialPose = sphericPose(objectPose,radiusTrajectory*1.5,M_PI*i/2,M_PI);
            if(robot.isReachable(initialPose))
            {
                robot.runMeasurementRoutine({initialPose},false,false,M_PI);
                break;
            }
        }
        else
        {
            robot.runMeasurementRoutine({initialPose},false,false,M_PI);
            break;
        }
    }*/

    if(robot.getToolName() == "cameraD435" || robot.getToolName() == "cameraD405")
    {
        ros::ServiceClient thresholdClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("threshold_filter");
        ros::ServiceClient disparityShiftClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("disparity_shift");
    
        robot_arm_3d_scan::FloatParameters thresholdSrv;
        robot_arm_3d_scan::FloatParameters disparityShiftSrv;

        thresholdSrv.request.parameters = std::vector<double>{0.0,radiusTrajectory};
        disparityShiftSrv.request.parameters = std::vector<double>{5.0};

        if(thresholdClient.call(thresholdSrv) && disparityShiftClient.call(disparityShiftSrv))
        {
            ROS_INFO("Main loop measurements parameters set up...");
        }    
    }

    moveit_msgs::CollisionObject collisionSphere = robotVisualTools.getCollisionObject("collisionSphere");
    
    ROS_INFO("%f,%f,%f",objectPose.position.x,objectPose.position.y,objectPose.position.z);
    objectPose = collisionSphere.primitive_poses[0];
    ROS_INFO("%f,%f,%f",objectPose.position.x,objectPose.position.y,objectPose.position.z);
    
    //Main loop
    waypoints.clear();
     
    //TODO Find a way to custom !!
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI, 0, 2*M_PI, 1, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2 + M_PI/6, 0, 2*M_PI, trajectoryStepsNumber, waypoints); 
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2, 0, 2*M_PI, trajectoryStepsNumber, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2 - M_PI/6, 0, 2*M_PI, trajectoryStepsNumber, waypoints);
    
    //TODO Online trajectory adaptation ?
    robot.runMeasurementRoutine(waypoints,false,true,M_PI);

    //Shut down ROS node 
    ros::shutdown();
    return 0;
}
