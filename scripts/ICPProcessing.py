#!/usr/bin/env python3

from copy import deepcopy, copy
import glob
import os

import numpy as np

import open3d as o3d
from scipy.spatial.transform import Rotation as R

import cloup

## Function drawing the registration result (target and source point clouds with transformation applied on source point cloud)
# @param source Source point cloud
# @param target Target point cloud
# @param transformation Registration transformation matrix to fit the source point cloud to the target point cloud
def draw_registration_result(source, target, transformation):
    source_temp = deepcopy(source)
    target_temp = deepcopy(target)
    source_temp.paint_uniform_color([1, 0, 0])
    target_temp.paint_uniform_color([0, 0, 1])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    
estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.000001, relative_rmse=0.000001, max_iteration=100)

@cloup.command()
@cloup.option("--input_point_clouds_prefix", type=str, default="PointCloud_VP(4)_DS(0)", help="Input point clouds prefix, such that point clouds are named as <prefix>_<index>.pcd")
@cloup.option("--max_correspondence_distance", type=float, default=0.005, help="Max correspondence distance for ICP")
@cloup.option("--show", is_flag=True, help="Shows intermediate point clouds registrations")
def main(input_point_clouds_prefix,max_correspondence_distance,show):

    init_source_to_target = np.eye(4)

    #Get all point clouds
    print(glob.glob(input_point_clouds_prefix+"_*.pcd"))
    files = sorted(glob.glob(input_point_clouds_prefix+"_*.pcd"), key = lambda file : int(os.path.basename(file).split(".")[0].split("_")[-1]))

    #Set first point cloud as first reference
    final_point_cloud = o3d.io.read_point_cloud(files[0])

    #Clean up (downsample, remove outliers, estimate normals, orient normals)
    final_point_cloud = final_point_cloud.voxel_down_sample(voxel_size=max_correspondence_distance/2)  
    final_point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    final_point_cloud.remove_radius_outlier(nb_points=100, radius=max_correspondence_distance*10)
    final_point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
    final_point_cloud.orient_normals_consistent_tangent_plane(100)

    for i in range(len(files)-1):

        #Get next point cloud
        target = deepcopy(final_point_cloud)
        source = o3d.io.read_point_cloud(files[i+1])

        #Clean up (downsample, remove outliers, estimate normals, orient normals)
        source = source.voxel_down_sample(voxel_size=max_correspondence_distance/2)
        source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        source.remove_radius_outlier(nb_points=100, radius=max_correspondence_distance*10)
        source.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
        source.orient_normals_consistent_tangent_plane(100)
        
        print("----------")
        print("Target: " + str(files[i+1]))

        if(show):
            draw_registration_result(source,target,init_source_to_target)
    
        #Perform ICP registration
        registration_icp = o3d.pipelines.registration.registration_icp(source, target, max_correspondence_distance, init_source_to_target, estimation, criteria)
        final_source_to_target = copy(registration_icp.transformation)

        if(show):
            draw_registration_result(source,target,final_source_to_target)
    
        print("--- ICP results ---")
        print("Transformation : " + str(final_source_to_target))
        print("Translation norm : " + str(np.linalg.norm(final_source_to_target[:3,-1])))
        print("Rotation angle : " + str(np.linalg.norm(R.from_matrix(final_source_to_target[:3,:3]).as_rotvec())))
        print("---")
        # fitness, which measures the overlapping area (# of inlier correspondences / # of points in target). The higher the better.
        # inlier_rmse, which measures the RMSE of all inlier correspondences. The lower the better.
        print("ICP fitness : " + str(registration_icp.fitness))
        print("ICP inliers RMSE : " + str(registration_icp.inlier_rmse))
        print("Number of correspondances : " + str(len(registration_icp.correspondence_set)))
        print("---")

        #Check if ICP is OK (inliers RMSE for correspondances quality, translation norm and rotation angle for transformation quality)
        if(registration_icp.inlier_rmse < 0.005 and np.linalg.norm(final_source_to_target[:3,-1]) < 0.1 and np.linalg.norm(R.from_matrix(final_source_to_target[:3,:3]).as_rotvec()) < 0.1):
            print("ICP OK")
            source.transform(final_source_to_target)
            final_point_cloud += source
        else:
            print("ICP NOK")
            final_point_cloud += source #Remove maybe ?
        print("----------")
            
        #Clean up (downsample, estimate normals, orient normals)
        final_point_cloud = final_point_cloud.voxel_down_sample(voxel_size=max_correspondence_distance/2)
        final_point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
        final_point_cloud.orient_normals_consistent_tangent_plane(100)

    o3d.visualization.draw_geometries([final_point_cloud])

    #Remove small clusters from final point cloud (noise !)
    labels = np.array(final_point_cloud.cluster_dbscan(eps=max_correspondence_distance*2, min_points=20))
    final_point_cloud = final_point_cloud.select_by_index(np.where(labels == 0)[0])
    final_point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    final_point_cloud.orient_normals_consistent_tangent_plane(50) 

    #Orient normals outwards the object (assuming the object is convex)
    centroid = final_point_cloud.get_center()
    for i,(point,normal) in enumerate(zip(final_point_cloud.points,final_point_cloud.normals)):
        delta = point-centroid
        delta = delta/np.linalg.norm(delta)
        if(np.dot(normal,delta) < -0.5):
            final_point_cloud.normals[i] = -normal
    
    #Save final point cloud
    o3d.visualization.draw_geometries([final_point_cloud])
    o3d.io.write_point_cloud("PointCloudICP.pcd", final_point_cloud)
    o3d.io.write_point_cloud("PointCloudICP.xyzn", final_point_cloud)

if __name__ == "__main__":
    main()