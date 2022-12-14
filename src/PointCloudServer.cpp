#include <pcl/io/pcd_io.h>

#include "robot_arm_3d_scan/PointCloudServer.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

int PointCloudServer::m_supportScanCounter = 0;

PointCloudServer::PointCloudServer() : MeasurementServer(), m_tfListener(m_tfBuffer)
{
    //Launch point cloud ROS publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/filtered_point_cloud",1);

    //Get point cloud frame id
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
    m_pointCloudFrame = rawPointCloud->header.frame_id;

    ROS_WARN("SERVER SETUP OK");

    std::vector<double> rawObjectPose;
    if(!m_nodeHandle.getParam("rawObjectPose",rawObjectPose))
    {
        ROS_ERROR("Unable to retrieve measurements reference pose !");
        throw std::runtime_error("MISSING PARAMETER");
    }
    m_objectPose.position.x = rawObjectPose[0];
    m_objectPose.position.y = rawObjectPose[1];
    m_objectPose.position.z = rawObjectPose[2];

    if(!m_nodeHandle.getParam("radiusObject",m_radiusObject))
    {
        ROS_ERROR("Unable to retrieve measured object radius !");
        throw std::runtime_error("MISSING PARAMETER");
    }

    m_visualTools.addSphere("collisionSphere", m_objectPose, m_radiusObject, false);    
}

bool PointCloudServer::measure(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
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
    //Remove scan support from point cloud
    transformPointCloud(pointCloud,"world");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    try
    {
        //Filter gound and retrieve ground point cloud 
        groundRemovalFilter(pointCloud,groundPointCloud,0.01); 
    }
    catch(const std::exception& e)
    {
        //continue;
    }   

    //Retrieve ground point cloud data
    pcl::PointXYZRGB centroidPoint;
    computeCentroid(*groundPointCloud,centroidPoint);
    double sizeX,sizeY,sizeZ;
    boundingBoxFilter(groundPointCloud,sizeX,sizeY,sizeZ);

    //Update new scan support collision object
    m_supportScanCounter++;

    geometry_msgs::Pose newSupportPose;
    newSupportPose.position.x = centroidPoint.x;
    newSupportPose.position.y = centroidPoint.y;
    newSupportPose.position.z = centroidPoint.z;

    m_visualTools.addBox("supportScan" + std::to_string(m_supportScanCounter),newSupportPose,sizeX,sizeY,sizeZ,false);

    //Update new scanned object collision object
    m_visualTools.deleteObject("collisionSphere");

    //Filter obvious outliers
    confidenceIntervalFilter(pointCloud,0.95);

    //Retrive point cloud data
    computeCentroid(*pointCloud,centroidPoint);

    double boundingRadius;
    boundingBoxFilter(pointCloud,sizeX,sizeY,sizeZ);
    boundingRadius = sqrt(sizeX*sizeX + sizeY*sizeY + sizeZ*sizeZ)/2;   
    ROS_WARN("%f %f %f %f",sizeX,sizeY,sizeZ,boundingRadius); 

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
    delta.setX(centroidPoint.x  - m_objectPose.position.x);
    delta.setY(centroidPoint.y  - m_objectPose.position.y);
    delta.setZ(centroidPoint.z  - m_objectPose.position.z);
    
    delta = rotationMatrix*delta;

    delta.setX(delta.x()*epsilonX);
    delta.setY(delta.y()*epsilonY);
    delta.setZ(delta.z()*epsilonZ);

    delta = rotationMatrix.inverse()*delta;

    m_objectPose.position.x += delta.x();
    m_objectPose.position.y += delta.y();
    m_objectPose.position.z += delta.z();

    if(boundingRadius >= 0.75*m_radiusObject)
    {
        m_radiusObject = boundingRadius;
    }

    m_visualTools.addSphere("collisionSphere", m_objectPose, m_radiusObject, false);
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