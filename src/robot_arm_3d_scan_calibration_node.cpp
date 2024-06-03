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
    // ROS node initialisation
    ros::init(argc, argv, "robot_arm_3d_scan_spheric_scan_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    // Robot initialisation
    Robot robot;
    RobotVisualTools robotVisualTools;

    // Move the robot to its initial configuration
    robot.setAcceleration(0.05);
    robot.setVelocity(0.1);

    // Get trajectory parameters
    double radiusTrajectory;
    int trajectoryStepsNumber;
    std::string calibrationTargetTF;

    ros::NodeHandle n;
    if (!n.getParam("radiusTrajectory", radiusTrajectory))
    {
        ROS_ERROR("Unable to retrieve trajectory radius !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    if (!n.getParam("trajectoryStepsNumber", trajectoryStepsNumber))
    {
        ROS_ERROR("Unable to retrieve trajectory steps number !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    n.param<std::string>("calibrationTargetTF", calibrationTargetTF, "handeye_target");

    // Wait for the calibration target to show up...
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    while (ros::ok())
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("world", calibrationTargetTF, ros::Time::now(), ros::Duration(3.0));
            break;
        }
        catch (tf2::TransformException &e)
        {
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    // Build the target object
    geometry_msgs::Pose objectPose;
    tf2::Quaternion quaternion;
    tf2::fromMsg(transformStamped.transform.rotation, quaternion);
    tf2::Matrix3x3 matrix(quaternion);
    tf2::Vector3 X = matrix.getColumn(0);
    tf2::Vector3 Y = matrix.getColumn(1);
    tf2::Vector3 Z = matrix.getColumn(2);

    // Target object = A4 paper sheet, 20cm x 30cm
    objectPose.position.x = X[0] * 0.1 + Y[0] * 0.15 + transformStamped.transform.translation.x;
    objectPose.position.y = X[1] * 0.1 + Y[1] * 0.15 + transformStamped.transform.translation.y;
    objectPose.position.z = X[2] * 0.1 + Y[2] * 0.15 + transformStamped.transform.translation.z;

    robotVisualTools.addBox("target_object", objectPose, abs(X[0] * 0.2 + Y[0] * 0.3 + Z[0] * 0.005), abs(X[1] * 0.2 + Y[1] * 0.3 + Z[1] * 0.005), abs(X[2] * 0.2 + Y[2] * 0.3 + Z[2] * 0.005), false, false);

    // Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    // TODO Find a way to custom ?
    sphericInclinationTrajectory(objectPose, radiusTrajectory, 0, 0, 2 * M_PI, 1, waypoints, true, -M_PI / 2);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI / 8, 0, 2 * M_PI, trajectoryStepsNumber, waypoints, true, -M_PI / 2);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI / 6, 0, 2 * M_PI, trajectoryStepsNumber, waypoints, true, -M_PI / 2);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI / 4, 0, 2 * M_PI, trajectoryStepsNumber, waypoints, true, -M_PI / 2);

    sphericAzimuthTrajectory(objectPose, radiusTrajectory, 0, -M_PI / 4, M_PI / 4, trajectoryStepsNumber, waypoints, true, -M_PI / 2);
    sphericAzimuthTrajectory(objectPose, radiusTrajectory, M_PI / 2, -M_PI / 4, M_PI / 4, trajectoryStepsNumber, waypoints, true, -M_PI / 2);

    // Rotate trajectory around center object position
    rotateTrajectory(waypoints, objectPose.position, roll, pitch, yaw);

    std::vector<geometry_msgs::Pose> waypointsUpShifted, waypointsDownShifted;
    objectPose.position.x += Y[0] * 0.075;
    objectPose.position.y += Y[1] * 0.075;
    objectPose.position.z += Y[2] * 0.075;
    sphericAzimuthTrajectory(objectPose, radiusTrajectory, 0, -M_PI / 4, M_PI / 4, trajectoryStepsNumber, waypointsUpShifted, true, -M_PI / 2);
    rotateTrajectory(waypointsUpShifted, objectPose.position, roll, pitch, yaw);

    objectPose.position.x -= Y[0] * 0.15;
    objectPose.position.y -= Y[1] * 0.15;
    objectPose.position.z -= Y[2] * 0.15;
    sphericAzimuthTrajectory(objectPose, radiusTrajectory, 0, -M_PI / 4, M_PI / 4, trajectoryStepsNumber, waypointsDownShifted, true, -M_PI / 2);
    rotateTrajectory(waypointsDownShifted, objectPose.position, roll, pitch, yaw);

    waypoints.insert(waypoints.end(), waypointsUpShifted.begin(), waypointsUpShifted.end());
    waypoints.insert(waypoints.end(), waypointsDownShifted.begin(), waypointsDownShifted.end());

    // TODO Online trajectory adaptation ?
    robot.runMeasurementRoutine(waypoints, false, true, -1, true);

    // Shut down ROS node
    ros::shutdown();
    return 0;
}
