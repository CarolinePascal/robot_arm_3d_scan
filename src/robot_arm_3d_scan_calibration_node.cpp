#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <std_srvs/Empty.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

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

    //Get trajectory parameters
    double radiusTrajectory;
    int trajectoryStepsNumber;
    std::string calibrationTargetTF;

    ros::NodeHandle n;
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
    n.param<std::string>("calibrationTargetTF", calibrationTargetTF, "handeye_target");

    //Wait for the target to show up...
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    while(ros::ok())
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("world", calibrationTargetTF, ros::Time::now(), ros::Duration(3.0));
            break;
        }
        catch(tf2::TransformException &e)
        {
            ros::Duration(1.0).sleep();
            continue;
        }
    }     

    geometry_msgs::Pose objectPose;
    tf2::Quaternion quaternion;
    tf2::fromMsg(transformStamped.transform.rotation, quaternion);
    tf2::Matrix3x3 matrix(quaternion);
    tf2::Vector3 X = matrix.getColumn(0);
    tf2::Vector3 Y = matrix.getColumn(1);
    
    objectPose.position.x = X[0]*0.075 + Y[0]*0.1 + transformStamped.transform.translation.x;
    objectPose.position.y = X[1]*0.075 + Y[1]*0.1 + transformStamped.transform.translation.y;
    objectPose.position.z = X[2]*0.075 + Y[2]*0.1 + transformStamped.transform.translation.z;
    robotVisualTools.addBox("target",objectPose,abs(X[0]*0.15 + Y[0]*0.2),abs(X[1]*0.15 + Y[1]*0.2),0.005,false,false);

    //Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;
     
    //TODO Find a way to custom !!
    sphericInclinationTrajectory(objectPose, radiusTrajectory, 0, 0, 2*M_PI, 1, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/6, 0, 2*M_PI, trajectoryStepsNumber/3, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/4, 0, 2*M_PI, trajectoryStepsNumber/3, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/3, 0, 2*M_PI, trajectoryStepsNumber/3, waypoints);
    
    //TODO Online trajectory adaptation ?
    robot.runMeasurementRoutine(waypoints,false,true,M_PI,false);

    //Shut down ROS node 
    ros::shutdown();
    return 0;
}
