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

    //Command line arguments handling
    if(argc < 3)
    {
        throw std::invalid_argument("MISSING CMD LINE ARGUMENT FOR robot_arm_3d_scan_spheric_scan_node !");
        return(1);
    }

    //Robot initialisation TODO More generic approach
    Robot robot("panda_arm", argv[1], "panda_" + std::string(argv[1]));

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Move the robot to its initial configuration
    robot.init();

    //Load object geometry
    std::vector<double> poseObject;
    double radiusObject;
    double radiusTrajectory;

    ROS_INFO("Getting acquisition parameters");

    ros::NodeHandle n;
    n.getParam("poseObject",poseObject);
    n.getParam("radiusObject",radiusObject);
    n.getParam("radiusTrajectory",radiusTrajectory);

    geometry_msgs::Pose centerPose;
    centerPose.position.x = poseObject[0];
    centerPose.position.y = poseObject[1];
    centerPose.position.z = poseObject[2];
    
    if(radiusObject != 0)
    {
        visualTools.addSphere("collisionSphere", centerPose, radiusObject, false);
    }

    int N=10;   //Waypoints number
    std::vector<geometry_msgs::Pose> waypoints;
    
    sphericInclinationTrajectory(centerPose, radiusTrajectory, M_PI/6, 0, 2*M_PI, N, waypoints); 
    
    robot.runMeasurementRountine(waypoints,argv[2],"/tmp/3d_ScanMeasurements/Positions.csv");

    //Shut down ROS node
    robot.init();   
    ros::waitForShutdown();
    return 0;
}