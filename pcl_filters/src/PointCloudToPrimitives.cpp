#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include "pcl_filters/PCLFilters.h"
#include "pcl_filters/PointCloudToPrimitives.h"

std::vector<visualization_msgs::Marker> pointCloudToPrimitives(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  pcl::io::savePCDFileASCII("/tmp/PointCloudTest.pcd", *pointCloud);
  ///DEBUG
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer.reset(new pcl::visualization::PCLVisualizer("Primitives Viewer"));
  viewer->setBackgroundColor(48 / 255.0, 48 / 255.0, 48 / 255.0);
  viewer->initCameraParameters();

  viewer->removeAllShapes();
  viewer->removeAllPointClouds();

  // Downsample input to speed up the computations
  downsampleFilter(pointCloud,0.001,0.001,0.001);

  // Setup supervoxel clustering
  pcl::SupervoxelClustering<pcl::PointXYZRGB> supervoxelClustering(0.002,0.05);
  supervoxelClustering.setInputCloud(pointCloud);
  supervoxelClustering.setUseSingleCameraTransform(false);
  supervoxelClustering.setSpatialImportance(1.0);
  supervoxelClustering.setColorImportance(0.25);

  // Estimate the normals and use them during clustering
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(pointCloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  normalEstimator.setSearchMethod(tree);
  normalEstimator.setRadiusSearch(0.002);
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>());
  normalEstimator.compute(*cloudNormals);

  supervoxelClustering.setNormalCloud(cloudNormals);
  supervoxelClustering.setNormalImportance(5.0);

  // Segment and extract the supervoxel clusters
  std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> supervoxelClusters;
  supervoxelClustering.extract(supervoxelClusters);

  // Refine supervoxels
  supervoxelClustering.refineSupervoxels(5, supervoxelClusters);

  // Get supervoxel adjacency
  std::multimap<std::uint32_t, std::uint32_t> supervoxelAdjacency;
  supervoxelClustering.getSupervoxelAdjacency(supervoxelAdjacency);

  //
  // Segmentation
  //
  pcl::LCCPSegmentation<pcl::PointXYZRGB> segmentation;

  segmentation.setInputSupervoxels(supervoxelClusters, supervoxelAdjacency);
  segmentation.setConcavityToleranceThreshold(20);
  segmentation.setSmoothnessCheck(true, 0.01, 0.05, 0.25);
  segmentation.setMinSegmentSize(1);
  segmentation.setKFactor(1);
  segmentation.setSanityCheck(true);

  // Segment
  segmentation.segment();

  // Determine labels for the segmented supervoxels
  std::map<std::uint32_t, std::set<std::uint32_t>> segmentationMap;
  segmentation.getSegmentToSupervoxelMap(segmentationMap);

  // Create separate point clouds for each segment
  std::map<std::uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentedPointClouds;
  for (const auto &[segment_id, voxel_ids] : segmentationMap)
  {
    // Fill each point cloud with all corresponding supervoxels
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &voxel_id : voxel_ids)
    {
      *segmentedPointCloud += *supervoxelClusters.at(voxel_id)->voxels_;
    }
    segmentedPointClouds.insert(std::pair<std::uint32_t, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(segment_id, segmentedPointCloud));
  }

  //
  // Sample Consensus
  //

  // Create individual groups for the supported primitives
  //std::vector<Plane> planes;
  //std::vector<Sphere> spheres;
  //std::vector<Cylinder> cylinders;

  srand((unsigned) time(NULL));
  std::vector<visualization_msgs::Marker> outputPrimitives;

  // Iterate over all segmented point clouds
  int segmentPointCloudCounter = 0;
  for (const auto &[segment_id, segment_cloud] : segmentedPointClouds)
  {
    segmentPointCloudCounter++;
    pcl::PointCloud<pcl::PointXYZ> rawPointCloud;
    pcl::copyPointCloud(*segment_cloud,rawPointCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rawPointCloudPtr = rawPointCloud.makeShared();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(rawPointCloudPtr, rand() % 256, rand() % 256, rand() % 256);
    viewer->addPointCloud<pcl::PointXYZ> (rawPointCloudPtr, singleColor, "segment_" + std::to_string(segmentPointCloudCounter));
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segment_" + std::to_string(segmentPointCloudCounter));

    // Make sure the point cloud has enough points
    if (segment_cloud->size() < pointCloud->size()*0.01)
    {
      ROS_WARN("Segment too small, skipping");
      continue;
    }

    // Create nullptr of sample consensus implementation
    pcl::RandomSampleConsensus<pcl::PointXYZRGB>::Ptr sacPlane, sacSphere, sacCylinder;

    // Create nullptr of sample consensus models
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr sacModelPlane;
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr sacModelSphere;
    pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal>::Ptr sacModelCylinder;

    // Create new consideration of the geometric primitives
    //Plane plane;
    //Sphere sphere;
    //Cylinder cylinder;

    // Setup and compute sample consensus for plane model
    sacModelPlane.reset(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(segment_cloud));

    sacPlane.reset(new pcl::RandomSampleConsensus<pcl::PointXYZRGB>(sacModelPlane));
    sacPlane->setDistanceThreshold(0.001);
    sacPlane->setProbability(0.99);
    sacPlane->setMaxIterations(1000);

    sacPlane->computeModel();
    //sac->getModelCoefficients(plane._raw_coefficients);
    //sac->getInliers(plane.inliers);
    //plane.validity.inlier_proportion = (float)plane.inliers.size() / (float)segment_cloud->size();

    // Setup and compute sample consensus for sphere model
    sacModelSphere.reset(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>(segment_cloud));
    sacModelSphere->setRadiusLimits(0.01, 0.1);

    sacSphere.reset(new pcl::RandomSampleConsensus<pcl::PointXYZRGB>(sacModelSphere));
    sacSphere->setDistanceThreshold(0.001);
    sacSphere->setProbability(0.99);
    sacSphere->setMaxIterations(1000);

    sacSphere->computeModel();
    //sac->getModelCoefficients(sphere._raw_coefficients);
    //sac->getInliers(sphere.inliers);
    //sphere.validity.inlier_proportion = (float)sphere.inliers.size() / (float)segment_cloud->size();

    // Setup and compute sample consensus for cylinder model

    // Estimate normals for the segmented point cloud - required for cylinder sac
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(segment_cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setRadiusSearch(0.002);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>());
    normalEstimator.compute(*cloudNormals);

    sacModelCylinder.reset(new pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal>(segment_cloud));
    sacModelCylinder->setInputNormals(cloudNormals);
    sacModelCylinder->setRadiusLimits(0.01, 0.1);

    sacCylinder.reset(new pcl::RandomSampleConsensus<pcl::PointXYZRGB>(sacModelCylinder));
    sacCylinder->setDistanceThreshold(0.001);
    sacCylinder->setProbability(0.99);
    sacCylinder->setMaxIterations(1000);

    sacCylinder->computeModel();
    //sac->getModelCoefficients(cylinder._raw_coefficients);
    //sac->getInliers(cylinder.inliers);
    //cylinder.validity.inlier_proportion = (float)cylinder.inliers.size() / (float)segment_cloud->size();

    // Create a local copy of enable flags that get overwritten if number of inliers/variance is not satisfactory
    //bool consider_plane = parameters_.plane.enable;
    //bool consider_sphere = parameters_.sphere.enable;
    //bool consider_cylinder = parameters_.cylinder.enable;

    // Try to select the model with the lowest variance that has satisfactory number of inliers and variance

    std::vector<int> planeInliers, sphereInliers, cylinderInliers;
    sacPlane->getInliers(planeInliers);
    sacSphere->getInliers(sphereInliers);
    sacCylinder->getInliers(cylinderInliers);

    bool isPlane = planeInliers.size() > sphereInliers.size() && planeInliers.size() > cylinderInliers.size();
    bool isSphere = sphereInliers.size() > planeInliers.size() && sphereInliers.size() > cylinderInliers.size();
    bool isCylinder = cylinderInliers.size() > planeInliers.size() && cylinderInliers.size() > sphereInliers.size();

    Eigen::VectorXf rawCoefficients;

    if(isPlane)
    {
      ROS_INFO("Plane detected");

      sacPlane->getModelCoefficients(rawCoefficients);
      Eigen::Vector3f normal = rawCoefficients.head<3>();
      double offset = rawCoefficients[3];

      Eigen::Quaternion<float> orientation;
      orientation.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal);

      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      inliers->indices = planeInliers;

      extract.setInputCloud(segment_cloud);
      extract.setIndices(inliers);
      extract.filter(*segment_cloud);

      pcl::PointXYZRGB centroid; 
      pcl::computeCentroid(*segment_cloud,centroid);
      Eigen::Vector3f position(centroid.x, centroid.y, centroid.z);

      Eigen::Affine3f aff = Eigen::Affine3f::Identity();
      aff.translation() = position;
      aff.linear() = orientation.toRotationMatrix();
      aff = aff.inverse();

      pcl::transformPointCloud (*segment_cloud, *segment_cloud, aff);

      double sizeX, sizeY, sizeZ;
      pcl::PointXYZ dummy;
      boundingBoxFilter(segment_cloud, dummy, sizeX, sizeY, sizeZ);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.type = visualization_msgs::Marker::CUBE;

      marker.pose.position.x = position[0];
      marker.pose.position.y = position[1];
      marker.pose.position.z = position[2];
      marker.pose.orientation.x = orientation.x();
      marker.pose.orientation.y = orientation.y();
      marker.pose.orientation.z = orientation.z();
      marker.pose.orientation.w = orientation.w();
      marker.scale.x = sizeX;
      marker.scale.y = sizeY;
      marker.scale.z = sizeZ;

      outputPrimitives.push_back(marker);

      pcl::ModelCoefficients coefficients;
      coefficients.values = std::vector<float>(rawCoefficients.data(), rawCoefficients.data() + rawCoefficients.size());
      //viewer->addPlane(coefficients, (double)position[0], (double)position[1], (double)position[2], "plane_" + std::to_string(segmentPointCloudCounter));
    }
    if(isSphere)
    {
      ROS_INFO("Sphere detected");

      sacSphere->getModelCoefficients(rawCoefficients);
      Eigen::Vector3f position = rawCoefficients.head<3>();
      double radius = rawCoefficients[3];

      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.type = visualization_msgs::Marker::SPHERE;

      marker.pose.position.x = position[0];
      marker.pose.position.y = position[1];
      marker.pose.position.z = position[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2*radius;
      marker.scale.y = 2*radius;
      marker.scale.z = 2*radius;

      outputPrimitives.push_back(marker);

      pcl::ModelCoefficients coefficients;
      coefficients.values = std::vector<float>(rawCoefficients.data(), rawCoefficients.data() + rawCoefficients.size());;
      //viewer->addSphere(coefficients, "sphere_" + std::to_string(segmentPointCloudCounter));
    }
    if(isCylinder)
    {
      ROS_INFO("Cylinder detected");

      // Extract cylinder limits from the associated pointcloud inliers
      sacCylinder->getModelCoefficients(rawCoefficients);

      double radius = rawCoefficients[6];
      Eigen::ParametrizedLine<float, 3> axis = Eigen::ParametrizedLine<float, 3>(rawCoefficients.head<3>(), rawCoefficients.head<6>().tail<3>());

      // First create a hyperplane that intersects the cylinder, while being parallel with its flat surfaces
      Eigen::Hyperplane<float, 3> cylinderPlane(axis.direction(), axis.origin());

      // Iterate over all point cloud inliers and find the limits
      float min = 0.0, max = 0.0;
      for (auto &inlier_index : cylinderInliers)
      {
        // Get signed distance to the point from hyperplane
        pcl::PointXYZRGB point = segment_cloud->at(inlier_index);
        Eigen::Vector3f pointPosition(point.x, point.y, point.z);
        float signedDistance = cylinderPlane.signedDistance(pointPosition);

        // Overwrite the limits if new are found
        if (signedDistance < min)
        {
          min = signedDistance;
        }
        else if (signedDistance > max)
        {
          max = signedDistance;
        }
      }

      // Determine height of the cylinder
      double height = max - min;

      // Get centre of the cylinder and define it as the position
      Eigen::Vector3f position = (axis.pointAt(min) + axis.pointAt(max)) / 2.0;

      // Determne the orientation
      Eigen::Quaternion<float> orientation;
      orientation.setFromTwoVectors(Eigen::Vector3f::UnitZ(), axis.direction().cast<float>());

      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.type = visualization_msgs::Marker::CYLINDER;

      marker.pose.position.x = position[0];
      marker.pose.position.y = position[1];
      marker.pose.position.z = position[2];

      marker.pose.orientation.x = orientation.x();
      marker.pose.orientation.y = orientation.y();
      marker.pose.orientation.z = orientation.z();
      marker.pose.orientation.w = orientation.w();

      marker.scale.x = 2*radius;
      marker.scale.y = 2*radius;
      marker.scale.z = height;

      outputPrimitives.push_back(marker);

      pcl::ModelCoefficients coefficients;
      coefficients.values = std::vector<float>(rawCoefficients.data(), rawCoefficients.data() + rawCoefficients.size());
      //viewer->addCylinder(coefficients, "cylinder_" + std::to_string(segmentPointCloudCounter));
    }
  }

  //DEBUG
  pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = supervoxelClustering.getLabeledCloud();
  segmentation.relabelCloud(*cpc_labeled_cloud);

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZL> point_cloud_color_handler(cpc_labeled_cloud, "label");
  //viewer->addPointCloud<pcl::PointXYZL>(cpc_labeled_cloud, point_cloud_color_handler, "segments");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segments");

  while (!viewer->wasStopped()) 
  {
    viewer->spinOnce();
  }
  viewer->close();

  return(outputPrimitives);
}