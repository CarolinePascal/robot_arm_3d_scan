#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>

#include <boost/math/distributions/normal.hpp>

#include <pcl/filters/crop_box.h>

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
        if (inliers->indices.size() == 0)
        {
            break;
        }

        else
        {
            //DEBUG
            ROS_WARN("DEBUG GROUND REMOVAL FILTER - PLANE COEFFICIENTS");
            std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                            << coefficients->values[1] << " "
                            << coefficients->values[2] << " " 
                            << coefficients->values[3] << std::endl;

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
            pcl::PointXYZRGB centroid; 
            pcl::computeCentroid(*pointCloud,centroid);
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

void statisticalOutliersFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, int meanK, double stdDevMulThresh)
{
    //Statistical outliers removal => Remove points located outside the assumed gaussian repartition of the distance to its neighbours
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statisticalFilter;
    statisticalFilter.setInputCloud(pointCloud);
    statisticalFilter.setMeanK(meanK);
    statisticalFilter.setStddevMulThresh(stdDevMulThresh);
    statisticalFilter.filter(*pointCloud);
}

void radiusOutliersFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double radius, int minNeighbors)
{
    //Radius outliers removal => Remove points which do not have at least some number of neighbours within a certain range
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radiusFilter;
    radiusFilter.setInputCloud(pointCloud);
    radiusFilter.setRadiusSearch(radius);
    radiusFilter.setMinNeighborsInRadius(minNeighbors);
    radiusFilter.setKeepOrganized(true);
    radiusFilter.filter(*pointCloud);
}

void downsampleFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double leafSizeX, double leafSizeY, double leafSizeZ)
{
    //Downsampling filter => Reduce the number of points in the point cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
    voxelGrid.setInputCloud(pointCloud);
    voxelGrid.setLeafSize(leafSizeX,leafSizeY,leafSizeZ);
    voxelGrid.filter(*pointCloud);
}

void RGBFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minR, double maxR, double minG, double maxG, double minB, double maxB)
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

void boundingBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointXYZ& center, double& sizeX, double& sizeY, double& sizeZ)
{
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*pointCloud, minPoint, maxPoint);

    center.x = (maxPoint.x + minPoint.x)/2;
    center.y = (maxPoint.y + minPoint.y)/2;
    center.z = (maxPoint.z + minPoint.z)/2;

    sizeX = sqrt((maxPoint.x - minPoint.x)*(maxPoint.x - minPoint.x));
    sizeY = sqrt((maxPoint.y - minPoint.y)*(maxPoint.y - minPoint.y));
    sizeZ = sqrt((maxPoint.z - minPoint.z)*(maxPoint.z - minPoint.z));
}

void boundingSphereFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointXYZ& center, double& radius)
{
    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(*pointCloud,centroid);
    center.x = centroid.x;
    center.y = centroid.y;
    center.z = centroid.z;

    double tmp;
    radius = 0;
    for(int i = 0; i < pointCloud->points.size(); i++) 
    {   
        tmp = euclideanDistance(pointCloud->points[i],centroid);
        if(tmp > radius){radius = tmp;}
    }
}

void cropFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
{
    pcl::CropBox<pcl::PointXYZRGB> box;
    box.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    box.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));

    box.setInputCloud(pointCloud);
    box.filter(*pointCloud);
}

void confidenceIntervalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double confidenceRate)
{
    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(*pointCloud,centroid);

    std::vector<float> distances;
    for(int i = 0; i < pointCloud->points.size(); i++) 
    {
        distances.push_back(euclideanDistance(pointCloud->points[i],centroid));
    }

    double mean,stdd;
    pcl::getMeanStd(distances,mean,stdd);

    boost::math::normal dist(0.0, 1.0);
    double q = quantile(dist, confidenceRate);

    pcl::PointIndices::Ptr outliers (new pcl::PointIndices);
    for(int i = 0; i < pointCloud->points.size(); i++) 
    {
        if(distances[i] > mean + q*stdd)
        {
            outliers->indices.push_back(i);
        }
    }

    //DEBUG
    ROS_WARN("DEBUG CONFIDENCE INTERVAL FILTER - OUTLIERS NUMBER : %d",outliers->indices.size());
    //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("DEBUG"));
    //viewer->setBackgroundColor (0, 0, 0);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(pointCloud);
    extract.setIndices(outliers);

    //DEBUG
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //extract.setNegative(true); 
    //extract.filter(*tmpPointCloud);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tmpColor(tmpPointCloud, 255, 0, 0);
    //viewer->addPointCloud(tmpPointCloud,tmpColor,"Outliers");

    extract.setNegative(false); 
    extract.filter(*pointCloud);

    //DEBUG
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(pointCloud, 0, 255, 0);
    //viewer->addPointCloud(pointCloud,color,"Inliers");
    //viewer->addCoordinateSystem (1.0);
    //viewer->initCameraParameters ();
    //while(!viewer->wasStopped())
    //{
    //    viewer->spinOnce();
    //}
}