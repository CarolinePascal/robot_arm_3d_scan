/**
 * \file PCLFilters.h
 * \brief Header file
 *
 * Defines the functions used to filter point clouds recieved from LIDARs and depth cameras
 *
 */

#pragma once

#define RANSAC_MAXIMUM_ITERATIONS 3

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

/*!
 * \brief Applies a depth threshold filter on a XYZRGB point cloud.
 * \param pointCloud Pointer on the point cloud to filter.
 * \param lowerThreshold Lower threshold filtering value.
 * \param upperThreshold Upper threshold filtering value.
*/
void thresholdFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double lowerThreshold, double upperThreshold);

/*!
 * \brief Applies a RANSAC based ground removal filter on a XYZRGB point cloud.
 * \param pointCloud Pointer on the point cloud to filter.
 * \param groundPointCloud Pointer on the filtered ground point cloud.
 * \param distanceThreshold Distance threshold of the plane detection.
*/
void groundRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr groundPointCloud, double distanceThreshold);

/*!
 * \brief Transform a XYZRGB point cloud into a given frame.
* \param pointCloud Pointer on the point cloud to transform.
 * \param targetFrame Target frame.
*/
void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, std::string targetFrame);

/*!
 * \brief Transform a XYZ point cloud into a given frame.
* \param pointCloud Pointer on the point cloud to transform.
 * \param targetFrame Target frame.
*/
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::string targetFrame);

/*!
 * \brief Applies a statistical outilers filter on a XYZRGB point cloud.
 * \param pointCloud Pointer on the point cloud to filter.
*/
void statisticalOutliersFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

/*!
 * \brief Applies a radius outliers filter on a XYZRGB point cloud.
 * \param pointCloud Pointer on the point cloud to filter.
*/
void radiusOutliersFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

/*!
 * \brief Applies a color threshold filter on a XYZRGB point cloud.
 * \param pointCloud Pointer on the point cloud to filter.
 * \param minR Minimum red value.
 * \param minG Minimum green value.
 * \param minB Minimum blue value.
 * \param maxR Maximum red value.
 * \param maxG Maximum green value.
 * \param maxB Maximum blue value.
*/
void RGBFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minR, double minG, double minB, double maxR, double maxG, double maxB);

/*!
 * \brief Returns the bounding box of a point cloud.
 * \param pointCloud Pointer on the point cloud to filter. 
 * \param sizeX Size of the bouding box along the X axis.
 * \param sizeY Size of the bouding box along the Y axis.
 * \param sizeZ Size of the bouding box along the Z axis.
*/
void BoundingBoxFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double& sizeX, double& sizeY, double& sizeZ);