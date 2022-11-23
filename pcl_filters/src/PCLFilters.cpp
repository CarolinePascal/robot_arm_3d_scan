#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common.h>

#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>

#include "pcl_filters/PCLFilters.h"

void thresholdFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minimumDistance, double maximumDistance)
{
    //Depth threshold filter => Remove points located outside a given depth range
    pcl::PassThrough<pcl::PointXYZRGB> thresholdFilter;
    thresholdFilter.setInputCloud(pointCloud);
    thresholdFilter.setFilterFieldName("z");
    thresholdFilter.setFilterLimits(minimumDistance,maximumDistance);
    thresholdFilter.filter(*pointCloud);
}

void groundRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundPointCloud, double distanceThreshold)
{
    //Plane model segmentation => Split the points regarding to the plane thay belong to 

    //Define the segementation filter parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(0.1);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudErrors (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTmp (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for(int i = 0; i < RANSAC_MAXIMUM_ITERATIONS; i++)
    {    
        //DEBUG
        //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("DEBUG"));
        //viewer->setBackgroundColor (0, 0, 0);

        //Perform plane segmentation in the point cloud
        seg.setInputCloud(pointCloud);
        seg.segment(*inliers, *coefficients);

        //Case where no plane is detected
        if (inliers->indices.size () == 0)
        {
            throw std::invalid_argument("CANNOT ESTIMATE PLANAR MODEL FOR THE GIVEN DATA SET !"); 
        }

        else
        {
            //DEBUG
            /*std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                            << coefficients->values[1] << " "
                            << coefficients->values[2] << " " 
                            << coefficients->values[3] << std::endl;*/

            //Extract the detected plane
            extract.setInputCloud(pointCloud);
            extract.setIndices(inliers);
            extract.setNegative(false); 
            extract.filter(*pointCloudTmp);

            //DEBUG
            //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(pointCloudTmp, 255, 0, 0);
            //viewer->addPointCloud(pointCloudTmp,color,"Outliers");

            //Filter the detected plane
            extract.setNegative(true); 
            extract.filter(*pointCloud);

            //DEBUG
            //viewer->addPointCloud(pointCloud,"Inliers");

            //Case where a plane not perpendicular to the z-axis is detected (i.e. not ground plane)
            pcl::PointXYZRGB centroid; pcl::computeCentroid(*pointCloud,centroid);
            if((coefficients->values[2] < 0.8 && coefficients->values[2] > -0.8) || (coefficients->values[3] > centroid.z))
            {
                //Store the corresponding points in a "false positive" temporary point cloud
                *pointCloudErrors += *pointCloudTmp;
            }

            else
            {
                //Store the extracted points for ground data recovery
                *groundPointCloud += *pointCloudTmp;
            }

            //DEBUG
            /*viewer->addCoordinateSystem (1.0);
            viewer->initCameraParameters ();
            while(!viewer->wasStopped())
            {
                viewer->spinOnce();
            }*/
        }
    }

    //Add back the eventual "false positive" detected ground planes
    *pointCloud += *pointCloudErrors;
}

void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string targetTF)
{
    //Create a tf2 transform listenner
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::Transform transform;

    try
    {
        transform = tfBuffer.lookupTransform(targetTF, pointCloud->header.frame_id, ros::Time(0), ros::Duration(5.0)).transform;
    } 
    catch (tf2::TransformException &ex) 
    {
        throw std::runtime_error("CANNOT RETRIVE SEEKED TRANSFORM !");
    }

    //Apply the seeked transform to the point cloud
    pcl_ros::transformPointCloud(*pointCloud,*pointCloud,transform);
    pointCloud->header.frame_id = targetTF;
}

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::string targetTF)
{
    //Create a tf2 transform listenner
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::Transform transform;

    try
    {
        transform = tfBuffer.lookupTransform(targetTF, pointCloud->header.frame_id, ros::Time(0), ros::Duration(5.0)).transform;
    } 
    catch (tf2::TransformException &ex) 
    {
        throw std::runtime_error("CANNOT RETRIVE SEEKED TRANSFORM !");
    }

    //Apply the seeked transform to the point cloud
    pcl_ros::transformPointCloud(*pointCloud,*pointCloud,transform);
    pointCloud->header.frame_id = targetTF;
}

void statisticalOutliersFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    //Statistical outliers removal => Remove points located outside the assumed gaussian repartition of the distance to its neighbours
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statisticalFilter;
    statisticalFilter.setInputCloud(pointCloud);
    statisticalFilter.setMeanK(50);
    statisticalFilter.setStddevMulThresh(1.0);
    statisticalFilter.filter(*pointCloud);
}

void radiusOutliersFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
    //Radius outliers removal => Remove points which do not have at least some number of neighbours within a certain range
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radiusFilter;
    radiusFilter.setInputCloud(pointCloud);
    radiusFilter.setRadiusSearch(0.01);
    radiusFilter.setMinNeighborsInRadius(10);
    radiusFilter.setKeepOrganized(true);
    radiusFilter.filter(*pointCloud);
}

void RGBFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minR, double minG, double minB, double maxR, double maxG, double maxB)
{
    //Conditionnal RGB filter => Segments and removes points of a given color (Optionnal - HL Programming)
    pcl::ConditionalRemoval<pcl::PointXYZRGB> RGBfilter;

    //Define comparaisons between red, green and blue values
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr redConditionLT(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r",pcl::ComparisonOps::LT,maxR));
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr greenConditionLT(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g",pcl::ComparisonOps::LT,maxG));
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blueConditionLT(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b",pcl::ComparisonOps::LT,maxB));

    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr redConditionGT(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r",pcl::ComparisonOps::GT,minR));
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr greenConditionGT(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g",pcl::ComparisonOps::GT,minG));
    pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr blueConditionGT(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b",pcl::ComparisonOps::GT,minB));

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr colorCondition (new pcl::ConditionAnd<pcl::PointXYZRGB>());
    colorCondition->addComparison(redConditionLT);
    colorCondition->addComparison(greenConditionLT);
    colorCondition->addComparison(blueConditionLT);
    colorCondition->addComparison(redConditionGT);
    colorCondition->addComparison(greenConditionGT);
    colorCondition->addComparison(blueConditionGT);

    //Apply RGB filter
    RGBfilter.setInputCloud(pointCloud);
    RGBfilter.setCondition(colorCondition);
    RGBfilter.filter(*pointCloud);
}

void BoundingBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double& sizeX, double& sizeY, double& sizeZ)
{
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*pointCloud, minPoint, maxPoint);

    sizeX = sqrt((maxPoint.x - minPoint.x)*(maxPoint.x - minPoint.x));
    sizeY = sqrt((maxPoint.y - minPoint.y)*(maxPoint.y - minPoint.y));
    sizeZ = sqrt((maxPoint.z - minPoint.z)*(maxPoint.z - minPoint.z));
}