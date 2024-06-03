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

#include <octomap_msgs/conversions.h>

int main(int argc, char **argv)
{
    // ROS node initialisation
    ros::init(argc, argv, "robot_arm_3d_scan_spheric_scan_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    // Robot initialisation TODO More generic approach
    Robot robot;
    RobotVisualTools robotVisualTools;

    // Move the robot to its initial configuration
    robot.setAcceleration(0.05);
    robot.setVelocity(0.1);

    // Get the object radius, pose and the trajectory radius
    double radiusTrajectory;
    int trajectoryStepsNumber;

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

    // Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;

    while(!n.hasParam("objectPose") && !n.hasParam("objectSize"))
    {
        ROS_WARN("Waiting for object pose and size parameters...");
        ros::WallDuration(5.0).sleep();
    }

    geometry_msgs::Pose objectPose;
    std::vector<double> objectPosition;
    objectPosition = n.param<std::vector<double>>("objectPose", objectPosition);
    objectPose.position.x = objectPosition[0];
    objectPose.position.y = objectPosition[1];
    objectPose.position.z = objectPosition[2];

    double objectSize;
    objectSize = n.param<double>("objectSize", objectSize);

    // Main loop
    // TODO Find a way to custom !!

    //ANECHOIC ROOM
    geometry_msgs::Pose supportPose = objectPose;
    supportPose.position.z += 0.8;
    robotVisualTools.addCylinder("support", supportPose, 0.02, 1.0, false, false);

    //ANECHOIC ROOM
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI, 0, 2*M_PI, 1, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2 + M_PI/3, 0, 2*M_PI, trajectoryStepsNumber, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2 + M_PI/6, 0, 2*M_PI, trajectoryStepsNumber, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2, 0, 2*M_PI, trajectoryStepsNumber, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2 - M_PI/6, 0, 2*M_PI, trajectoryStepsNumber, waypoints);
    sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI/2 - M_PI/4, 0, 2*M_PI, trajectoryStepsNumber, waypoints);

    //OPTITRACK ROOM
    //sphericInclinationTrajectory(objectPose, radiusTrajectory, 0, 0, 2 * M_PI, 1, waypoints);
    //sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI / 6, 0, 2 * M_PI, //trajectoryStepsNumber / 3, waypoints);
    //sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI / 4, 0, 2 * M_PI, trajectoryStepsNumber / 3, waypoints);
    //sphericInclinationTrajectory(objectPose, radiusTrajectory, M_PI / 3, 0, 2 * M_PI, trajectoryStepsNumber / 3, waypoints);

    // TODO Online trajectory adaptation ?
    robot.runMeasurementRoutine(waypoints, true, true, -1, true, false);

    //Save final planning scene if perception was enabled
    bool robotPerception = false;
    if(n.getParam("robotPerception", robotPerception) && robotPerception)
    {
        std::shared_ptr<planning_scene::PlanningScene> planningScene = robot.getPlanningScene();

        octomap_msgs::OctomapWithPose rawOctomap;
        bool output = planningScene->getOctomapMsg(rawOctomap);

        if(!output)
        {
            ROS_WARN("Could not save planning scene octomap data !");
        }
        else
        {
            octomap_msgs::Octomap octomap = rawOctomap.octomap;
            octomap::AbstractOcTree* abstractMap = octomap_msgs::msgToMap(octomap);

            octomap::OcTree* map = (octomap::OcTree*)abstractMap;
            octomap::OcTree tree = *map;

            std::string storageFolder;
            if(!n.getParam("measurementServerStorageFolder",storageFolder) || storageFolder == "")
            {
                ROS_WARN("No measurement server storage folder specified, switching to /tmp/Measurements/");
                storageFolder = "/tmp/Measurements/";
            }
            tree.writeBinary(storageFolder + "Octomap.bt");
        }       
    }

    // Shut down ROS node
    ros::shutdown();
    return 0;
}
