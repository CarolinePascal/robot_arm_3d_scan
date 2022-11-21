#include <ros/ros.h>
#include <robot_arm_tools/Robot.h>
#include <robot_arm_tools/RobotTrajectories.h>
#include <robot_arm_tools/RobotVisualTools.h>

#include <std_srvs/Empty.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <fstream>

#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>

class CollisionUpdater
{   
    public:
        CollisionUpdater(double radiusObject, geometry_msgs::Pose objectPose) : m_radiusObject(radiusObject), m_objectPose(objectPose), m_tfListener(m_tfBuffer)
        {
            //Add initial collision object
            if(radiusObject != 0)
            {
                m_visualTools.addSphere("collisionSphere", objectPose, radiusObject, false);
            }

            sensor_msgs::PointCloud2ConstPtr tmpPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
            m_pointCloudFrame = tmpPointCloud->header.frame_id;
        }

        void updateCollisions(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud)
        {
            m_visualTools.deleteObject("collisionSphere");

            //Compute new centroid
            pcl::PointXYZRGB newCentoid;
            computeCentroid(*pointCloud,newCentoid);

            //Compute new bounding sphere
            double newBoundingRadius;
            pcl::PointXYZRGB minPoint, maxPoint;
            pcl::getMinMax3D(*pointCloud, minPoint, maxPoint);

            newBoundingRadius = sqrt((maxPoint.x - minPoint.x)*(maxPoint.x - minPoint.x) + 
                                     (maxPoint.y - minPoint.y)*(maxPoint.y - minPoint.y) +
                                     (maxPoint.z - minPoint.z)*(maxPoint.z - minPoint.z));    

            //Get current point cloud frame position
            tf2::Transform transform;
            try
            {
                tf2::fromMsg(m_tfBuffer.lookupTransform(m_pointCloudFrame, "world", ros::Time(0), ros::Duration(5.0)).transform, transform);
            } 
            catch (tf2::TransformException &ex) 
            {
                throw std::runtime_error("CANNOT RETRIVE SEEKED TRANSFORM !");
            }      

            tf2::Matrix3x3 rotationMatrix = transform.getBasis();
            tf2::Vector3 delta;

            //TODO Some intelligent probabilities for epsilon
            double epsilonX,epsilonY,epsilonZ;
            epsilonX = 1.0;
            epsilonY = 1.0;
            epsilonZ = 0.0;

            delta.setX((newCentoid.x - m_objectPose.position.x));
            delta.setY((newCentoid.y - m_objectPose.position.y));
            delta.setZ((newCentoid.z - m_objectPose.position.z));

            delta = rotationMatrix*delta;

            delta.setX(delta.x()*epsilonX);
            delta.setY(delta.y()*epsilonY);
            delta.setZ(delta.z()*epsilonZ);

            delta = rotationMatrix.inverse()*delta;

            m_objectPose.position.x += delta.x();
            m_objectPose.position.y += delta.y();
            m_objectPose.position.z += delta.z();

            if(newBoundingRadius >= 0.75*m_radiusObject)
            {
                m_radiusObject = newBoundingRadius;
            }

            m_visualTools.addSphere("collisionSphere", m_objectPose, m_radiusObject, false);
        }

    private: 
        RobotVisualTools m_visualTools;
        double m_radiusObject;
        geometry_msgs::Pose m_objectPose;

        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener;

        std::string m_pointCloudFrame;
};

int main(int argc, char **argv)
{
    //ROS node initialisation
    ros::init(argc, argv, "robot_arm_3d_scan_spheric_scan_node");  
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    //Robot initialisation TODO More generic approach
    Robot robot;

    //Move the robot to its initial configuration
    //robot.init();
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

    //Robot visual tools initialisation
    CollisionUpdater collisionUpdater(radiusObject,poseObject);
    ros::Subscriber collisionUpdaterSubscriber = n.subscribe("/filtered_point_cloud", 1, &CollisionUpdater::updateCollisions, &collisionUpdater);
    
    //Create spherical scanning waypoints poses
    std::vector<geometry_msgs::Pose> waypoints;
    
    //Initial measurement
    sphericInclinationTrajectory(poseObject, radiusTrajectory*2, 0, 0, 2*M_PI, 1, waypoints); 
    robot.runMeasurementRountine(waypoints,false,false,M_PI/2);
    
    //Main loop
    waypoints.clear();
    sphericInclinationTrajectory(poseObject, radiusTrajectory, M_PI/3.5, 0, 2*M_PI, trajectoryStepsNumber, waypoints); 
    robot.runMeasurementRountine(waypoints,false,true,M_PI);

    //Shut down ROS node 
    ros::waitForShutdown();
    return 0;
}
