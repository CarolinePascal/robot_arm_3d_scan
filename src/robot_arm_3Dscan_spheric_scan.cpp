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
    ros::init(argc, argv, "robot_arm_3Dscan_spheric_scan_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Command line arguments handling
    if(argc < 3)
    {
        throw std::invalid_argument("MISSING CMD LINE ARGUMENT FOR robot_arm_3Dscan_spheric_scan_node !");
        return(1);
    }

    //Robot initialisation TODO More generic approach
    Robot robot("panda_arm", argv[1], "panda_" + std::string(argv[1]));

    //Get ROS service measurement server name and create client
    ros::NodeHandle n;
    ros::ServiceClient measurementClient = n.serviceClient<std_srvs::Empty>(argv[2]);

    //Robot visual tools initialisation
    RobotVisualTools visualTools;

    //Move the robot to its initial configuration
    robot.init();

    //Load object geometry
    std::vector<double> poseObject;
    double radiusObject;
    double radiusTrajectory;

    ROS_INFO("Getting acquisition parameters");

    n.getParam("poseObject",poseObject);
    n.getParam("radiusObject",radiusObject);
    n.getParam("radiusTrajectory",radiusTrajectory);

    geometry_msgs::Pose centerPose;
    centerPose.position.x = poseObject[0];
    centerPose.position.y = poseObject[1];
    centerPose.position.z = poseObject[2];

    std::cout<<radiusObject<<std::endl;
    
    if(radiusObject != 0)
    {
        visualTools.addSphere("collisionSphere", centerPose, radiusObject, false);
    }

    int N=10;   //Waypoints number
    std::vector<geometry_msgs::Pose> waypoints;
    
    sphericInclinationTrajectory(centerPose, radiusTrajectory, M_PI/6, 0, 2*M_PI, N, waypoints); 
    
    //Create .csv file for positions recording - Folder was already created in MicroService node
    std::ofstream myfile;
    myfile.open("/tmp/3DScan/Positions.csv");

    std_srvs::Empty request;

    //Acquisition loop
    for(int i = 0; i < waypoints.size(); i++)
    {      
        ROS_INFO("Waypoint %i out of %i", i+1, (int)waypoints.size());
        //std::cout<<waypoints[i]<<std::endl;

        visualTools.addFrame("waypoint",waypoints[i]);
        
        try
        {
            robot.goToTarget(waypoints[i],false);
            visualTools.deleteObject("waypoint");
        }
        catch(const std::runtime_error& e)
        {
            ROS_WARN("Skipping unreachable waypoint");
            visualTools.deleteObject("waypoint");
            continue;
        }

        ros::WallDuration(1.0).sleep();

        if(measurementClient.call(request))
        {
            ROS_INFO("Measurement - done !");
        }

        else
        {
            throw std::runtime_error("ERROR DURING Measurement !");
            ros::waitForShutdown();
            return 1;
        }

        tf2::Quaternion quaternion;
        tf2::fromMsg(waypoints[i].orientation,quaternion);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        myfile << i+1;
        myfile << ",";
        myfile << waypoints[i].position.x;
        myfile << ",";
        myfile << waypoints[i].position.y;
        myfile << ",";
        myfile << waypoints[i].position.z;
        myfile << ",";
        myfile << roll;
        myfile << ",";
        myfile << pitch;
        myfile << ",";
        myfile << yaw;
        myfile << "\n";
    }

    //Close .csv file and shut down ROS node
    myfile.close();
    robot.init();   
    ros::waitForShutdown();
    return 0;
}