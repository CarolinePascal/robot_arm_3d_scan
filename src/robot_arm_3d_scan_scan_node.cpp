#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <std_srvs/Empty.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <fstream>

#include <cmath>

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_3d_scan_spheric_scan_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation TODO More generic approach
    Robot robot;

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Move the robot to its initial configuration
    robot.init();
    robot.setAcceleration(0.05);
    robot.setVelocity(0.1);

    //Get the object radius, pose and the trajectory radius
    std::vector<double> poseObject;
    double radiusObject;
    double radiusTrajectory;
    int trajectoryStepsNumber;

    ros::NodeHandle n;
    if(!n.getParam("poseObject",poseObject))
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

    geometry_msgs::Pose centerPose;
    centerPose.position.x = poseObject[0];
    centerPose.position.y = poseObject[1];
    centerPose.position.z = poseObject[2];
    
    //Add collisions objects
    if(radiusObject != 0)
    {
        visualTools.addSphere("collisionSphere", centerPose, radiusObject, false);
    }

    //Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;
    
    sphericInclinationTrajectory(centerPose, radiusTrajectory, M_PI/3, 0, 2*M_PI, trajectoryStepsNumber, waypoints); 

    //Main loop
    robot.runMeasurementRountine(waypoints);

    //Shut down ROS node
    robot.init();   
    ros::waitForShutdown();
    return 0;
}
