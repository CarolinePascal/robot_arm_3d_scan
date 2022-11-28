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
    std::vector<double> rawObjectPose;
    double radiusObject;
    double radiusTrajectory;
    int trajectoryStepsNumber;

    ros::NodeHandle n;
    if(!n.getParam("rawObjectPose",rawObjectPose))
    {
        ROS_ERROR("Unable to retrieve measurements reference pose !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if(!n.getParam("radiusObject",radiusObject))
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

    geometry_msgs::Pose poseObject;
    poseObject.position.x = rawObjectPose[0];
    poseObject.position.y = rawObjectPose[1];
    poseObject.position.z = rawObjectPose[2];
    
    //Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;
    
    //Initial measurement

    if(robot.getToolName() == "cameraD435")
    {
        ros::ServiceClient thresholdClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("threshold_filter");
        ros::ServiceClient disparityShiftClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("disparity_shift");
    
        robot_arm_3d_scan::FloatParameters thresholdSrv;
        robot_arm_3d_scan::FloatParameters disparityShiftSrv;

        thresholdSrv.request.parameters = std::vector<double>{0.0,1.0};
        disparityShiftSrv.request.parameters = std::vector<double>{0.0};

        if(thresholdClient.call(thresholdSrv) && disparityShiftClient.call(disparityShiftSrv))
        {
            ROS_INFO("Initial measurement parameters set up...");
        }    
    }

    sphericInclinationTrajectory(poseObject, radiusTrajectory*2, 0, 0, 2*M_PI, 1, waypoints); 
    robot.runMeasurementRountine(waypoints,false,false,M_PI/2);

    if(robot.getToolName() == "cameraD435")
    {
        ros::ServiceClient thresholdClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("threshold_filter");
        ros::ServiceClient disparityShiftClient = n.serviceClient<robot_arm_3d_scan::FloatParameters>("disparity_shift");
    
        robot_arm_3d_scan::FloatParameters thresholdSrv;
        robot_arm_3d_scan::FloatParameters disparityShiftSrv;

        thresholdSrv.request.parameters = std::vector<double>{0.0,0.5};
        disparityShiftSrv.request.parameters = std::vector<double>{5.0};

        if(thresholdClient.call(thresholdSrv) && disparityShiftClient.call(disparityShiftSrv))
        {
            ROS_INFO("Main loop measurements parameters set up...");
        }    
    }

    moveit_msgs::CollisionObject collisionSphere = robotVisualTools.getCollisionObject("collisionSphere");
    
    ROS_INFO("%f,%f,%f",poseObject.position.x,poseObject.position.y,poseObject.position.z);
    poseObject = collisionSphere.primitive_poses[0];
    ROS_INFO("%f,%f,%f",poseObject.position.x,poseObject.position.y,poseObject.position.z);
    
    //Main loop
    waypoints.clear();
    sphericInclinationTrajectory(poseObject, radiusTrajectory, M_PI/3.5, 0, 2*M_PI, trajectoryStepsNumber, waypoints); 
    //TODO Online trajectory adaptation ?
    robot.runMeasurementRountine(waypoints,false,true,M_PI);

    robot.stop();

    //Shut down ROS node 
    ros::shutdown();
    return 0;
}
