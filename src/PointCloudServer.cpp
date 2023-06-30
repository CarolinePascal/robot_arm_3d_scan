#include <pcl/io/pcd_io.h>

#include "robot_arm_3d_scan/PointCloudServer.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

int PointCloudServer::m_supportScanCounter = 0;

PointCloudServer::PointCloudServer() : MeasurementServer(), m_tfListener(m_tfBuffer), m_groundRemoval(false)
{
    //Launch point cloud ROS publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/filtered_point_cloud",1);

    //Get point cloud frame id
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
    m_pointCloudFrame = rawPointCloud->header.frame_id;

    ros::NodeHandle n("~");
    n.getParam("ground_removal",m_groundRemoval);

    ROS_WARN("SERVER SETUP OK");

    try
    {
        std::vector<double> objectPoseArray;
        if(!m_nodeHandle.getParam("objectPose",objectPoseArray))
        {
            ROS_ERROR("Unable to retrieve measurements reference pose !");
            throw std::runtime_error("MISSING PARAMETER");
        }
        m_objectPose.position.x = objectPoseArray[0];
        m_objectPose.position.y = objectPoseArray[1];
        m_objectPose.position.z = objectPoseArray[2];

        if(!m_nodeHandle.getParam("objectSize",m_objectSize))
        {
            ROS_ERROR("Unable to retrieve measured object size !");
            throw std::runtime_error("MISSING PARAMETER");
        }
    }
    catch(const std::exception& e)
    {
        //Get last published raw point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
        pcl::fromROSMsg(*rawPointCloud, *pointCloud);

        //Filter obvious outliers
        confidenceIntervalFilter(pointCloud,0.95);

        //Retrive point cloud data
        pcl::PointXYZ centroid;
        double radius;

        boundingSphereFilter(pointCloud,centroid,radius);   

        m_objectPose.position.x = centroid.x;
        m_objectPose.position.y = centroid.y;
        m_objectPose.position.z = centroid.z;
        m_objectSize = 2*radius; 
    }  

    m_visualTools.addSphere("collisionSphere", m_objectPose, m_objectSize/2, false);    
}

bool PointCloudServer::measure()
{
    //Get last published raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
    pcl::fromROSMsg(*rawPointCloud, *pointCloud);

    m_measurementServerCounter++;

    //Filter point cloud, save it and publish it
    simplePointCloudFilter(pointCloud);
    if(m_measurementServerStorageFolder != "")
    {
        pcl::io::savePCDFileASCII(std::string(m_measurementServerStorageFolder + "PointCloud_" + std::to_string(m_measurementServerCounter) + ".pcd"), *pointCloud);
    }
    m_pointCloudPublisher.publish(*pointCloud);

    return(true);
}

void PointCloudServer::simplePointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    transformPointCloud(pointCloud,"world");
    
    //Optional ground removal filter !
    if(m_groundRemoval)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        //Filter gound and retrieve ground point cloud 
        groundRemovalFilter(pointCloud,groundPointCloud,0.01); 

        //When plane detection is over and a plane was detected 
        if(groundPointCloud->points.size() > 0)
        {
            //Retrieve ground point cloud data
            pcl::PointXYZ center;
            double sizeX,sizeY,sizeZ;
            boundingBoxFilter(groundPointCloud,center,sizeX,sizeY,sizeZ);

            //Update new scan support collision object
            m_supportScanCounter++;

            geometry_msgs::Pose newSupportPose;
            newSupportPose.position.x = center.x;
            newSupportPose.position.y = center.y;
            newSupportPose.position.z = center.z;

            m_visualTools.addBox("supportScan" + std::to_string(m_supportScanCounter),newSupportPose,sizeX,sizeY,sizeZ,false);
        }
    }

    //Update new scanned object collision object
    m_visualTools.deleteObject("collisionSphere");

    //Filter obvious outliers
    confidenceIntervalFilter(pointCloud,0.99);

    //Retrive point cloud data
    pcl::PointXYZ centroid;
    double radius;

    boundingSphereFilter(pointCloud,centroid,radius);   

    //TODO Some intelligent probabilities for epsilon
    double epsilonX,epsilonY,epsilonZ;
    epsilonX = 1.0;
    epsilonY = 1.0;
    epsilonZ = 0.0;

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
    delta.setX(centroid.x  - m_objectPose.position.x);
    delta.setY(centroid.y  - m_objectPose.position.y);
    delta.setZ(centroid.z  - m_objectPose.position.z);
    
    delta = rotationMatrix*delta;

    delta.setX(delta.x()*epsilonX);
    delta.setY(delta.y()*epsilonY);
    delta.setZ(delta.z()*epsilonZ);

    delta = rotationMatrix.inverse()*delta;

    m_objectPose.position.x += delta.x();
    m_objectPose.position.y += delta.y();
    m_objectPose.position.z += delta.z();

    if(2*radius >= 0.75*m_objectSize)
    {
        m_objectSize = 2*radius;
    }

    //TODO Add collisison volumes rather than sphere !
    m_visualTools.addSphere("collisionSphere", m_objectPose, m_objectSize/2, false);
}

int main(int argc, char *argv[])
{
    //ROS node initialisation
    ros::init(argc, argv, "point_cloud_server");  

    //Point cloud service initialisation
    PointCloudServer pointCloudServer;

	ros::spin();
    return 0;
}