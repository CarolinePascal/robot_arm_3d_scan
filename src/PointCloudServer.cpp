#include <pcl/io/pcd_io.h>

#include "robot_arm_3d_scan/PointCloudServer.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

int PointCloudServer::m_supportScanCounter = 0;

PointCloudServer::PointCloudServer() : MeasurementServer(), m_tfListener(m_tfBuffer), m_groundRemoval(false), m_filterChain("sensor_msgs::PointCloud2"), m_privateNodeHandle("~")
{
    //Configure robot body filter
    m_filterChain.configure("cloud_filter_chain",m_privateNodeHandle);

    //Launch point cloud ROS publisher
    m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/filtered_point_cloud",1);

    //Get point cloud frame id
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");
    m_pointCloudFrame = rawPointCloud->header.frame_id;

    //Get ground removal parameter
    m_privateNodeHandle.getParam("ground_removal",m_groundRemoval);

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
        pcl::fromROSMsg(*rawPointCloud, *pointCloud);

        simplePointCloudFilter(pointCloud, true);
    }    

}

bool PointCloudServer::measure()
{
    //Get last published raw point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr rawPointCloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/point_cloud");

    //Save raw point cloud
    if(m_measurementServerStorageFolder != "")
    {
    pcl::fromROSMsg(*rawPointCloud, *pointCloud);
        pcl::io::savePCDFileASCII(std::string(m_measurementServerStorageFolder + "RawPointCloud_default_" + std::to_string(m_measurementServerCounter) + ".pcd"), *pointCloud);
    }

    //Filter out robot body (before PCL conversion !)
    sensor_msgs::PointCloud2 cleanedPointCloud;
    m_filterChain.update(*rawPointCloud,cleanedPointCloud);  
    pcl::fromROSMsg(cleanedPointCloud, *pointCloud); 

    //Filter point cloud, save it and publish it
    simplePointCloudFilter(pointCloud);
    if(m_measurementServerStorageFolder != "")
    {
        pcl::io::savePCDFileASCII(std::string(m_measurementServerStorageFolder + "PointCloud_default_" + std::to_string(m_measurementServerCounter) + ".pcd"), *pointCloud);
    }
    m_pointCloudPublisher.publish(*pointCloud);

    return(true);
}

void PointCloudServer::simplePointCloudFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, bool initialisation)
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

    //Downsample point cloud
    voxelGridFilter(pointCloud,0.001);

    //Filter outliers
    radiusOutliersFilter(pointCloud,0.01,100);
    //radiusOutliersFilter(pointCloud);

    //Retrive point cloud data
    pcl::PointXYZ centroid;
    double radius;

    boundingSphereFilter(pointCloud,centroid,radius);   

    /*if(!initialisation)
    {
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
    }
    */
    if(initialisation)
    {
        m_objectPose.position.x = centroid.x;
        m_objectPose.position.y = centroid.y;
        m_objectPose.position.z = centroid.z;
        m_objectSize = 2*radius;
        ROS_INFO("Object position : %f,%f,%f", m_objectPose.position.x, m_objectPose.position.y, m_objectPose.position.z);
        ROS_INFO("Object size : %f", m_objectSize);
    }

    m_nodeHandle.setParam("objectPose", std::vector<double>{m_objectPose.position.x,m_objectPose.position.y,m_objectPose.position.z,0.0,0.0,0.0});
    m_nodeHandle.setParam("objectSize", m_objectSize);
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