/**
 * \file PointCloudToPrimitives.h
 * \brief Header file
 *
 * Defines the function used to convert a point cloud into a set of geometric primitives (plane, sphere, cyclinder & box)
 *
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>

std::vector<visualization_msgs::Marker> pointCloudToPrimitives(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);