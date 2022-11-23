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
 * \param distanceThreshold Distance threshold of the plane detection.
*/
void groundRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double distanceThreshold);

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
 * \param maxR Maximum red value.
 * \param minG Minimum green value.
 * \param maxG Maximum green value.
 * \param minB Minimum blue value.
 * \param maxB Maximum blue value.
*/
void RGBFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minR, double maxR, double minG, double maxG, double minB, double maxB);

/*!
 * \brief Applies a cropping filter on a XYZRGB point cloud.
 * \param pointCloud Pointer on the point cloud to filter.
 * \param minX Minimum distance along the X axis.
 * \param maxX Maximum distance along the X axis. 
 * \param minY Minimum distance along the Y axis.
 * \param maxY Maximum distance along the Y axis. 
 * \param minZ Minimum distance along the Z axis.
 * \param maxZ Maximum distance along the Z axis. 
*/
void cropFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, double minX, double maxX, double minY, double maxY, double minZ, double maxZ);