#!/usr/bin/env python3

import glob
import numpy as np
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation as R

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
    
estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.000001, relative_rmse=0.000001, max_iteration=100)

max_correspondence_distance = 0.005
init_source_to_target = np.eye(4)

if __name__ == "__main__":

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()

    files = sorted(glob.glob("PointCloud_VP(4)_DS(0)_*.pcd"), key = lambda file : int(file.split(".")[0].split("_")[-1]))[1:]

    final_point_cloud = o3d.io.read_point_cloud(files[0])
    final_point_cloud = final_point_cloud.voxel_down_sample(voxel_size=0.0025)  

    final_point_cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    final_point_cloud.remove_radius_outlier(nb_points=100, radius=0.05)
    
    final_point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
    final_point_cloud.orient_normals_consistent_tangent_plane(100)
    # vis.add_geometry(final_point_cloud)

    for i in range(len(files)-1):
        target = copy.deepcopy(final_point_cloud)
        source = o3d.io.read_point_cloud(files[i+1])

        source = source.voxel_down_sample(voxel_size=0.0025)

        source.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        source.remove_radius_outlier(nb_points=100, radius=0.05)

        source.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
        source.orient_normals_consistent_tangent_plane(100)
        
        print("Target: " + str(files[i+1]))

        #draw_registration_result(source,target,init_source_to_target)
    
        registration_icp = o3d.pipelines.registration.registration_icp(source, target, max_correspondence_distance, init_source_to_target, estimation, criteria)

        final_source_to_target = copy.copy(registration_icp.transformation)
        #draw_registration_result(source,target,final_source_to_target)
    
        print(final_source_to_target)
        print(np.linalg.norm(final_source_to_target[:3,-1]))
        print(np.linalg.norm(R.from_matrix(final_source_to_target[:3,:3]).as_rotvec()))

        print(registration_icp.fitness)
        print(registration_icp.inlier_rmse)
        print(len(registration_icp.correspondence_set))

        #registration_icp.fitness > 0.7
        if(registration_icp.inlier_rmse < 0.005 and np.linalg.norm(final_source_to_target[:3,-1]) < 0.1 and np.linalg.norm(R.from_matrix(final_source_to_target[:3,:3]).as_rotvec()) < 0.1):
            print("ICP OK")
            source.transform(final_source_to_target)
            final_point_cloud += source
        else:
            print("ICP NOK")
            final_point_cloud += source
            
        final_point_cloud = final_point_cloud.voxel_down_sample(voxel_size=0.0025)
        final_point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
        final_point_cloud.orient_normals_consistent_tangent_plane(100)
            
        # vis.update_geometry(final_point_cloud)
        # vis.poll_events()
        # vis.update_renderer()

    o3d.visualization.draw_geometries([final_point_cloud])

    labels = np.array(final_point_cloud.cluster_dbscan(eps=0.01, min_points=20))
    
    # import matplotlib.pyplot as plt
    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # final_point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([final_point_cloud])

    final_point_cloud = final_point_cloud.select_by_index(np.where(labels == 0)[0])

    # final_point_cloud = final_point_cloud.voxel_down_sample(voxel_size=0.005)
    # final_point_cloud.remove_statistical_outlier(nb_neighbors=40, std_ratio=2.0)
    # final_point_cloud.remove_radius_outlier(nb_points=200, radius=0.05)

    final_point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    final_point_cloud.orient_normals_consistent_tangent_plane(50) 

    centroid = final_point_cloud.get_center()
    for i,(point,normal) in enumerate(zip(final_point_cloud.points,final_point_cloud.normals)):
        delta = point-centroid
        delta = delta/np.linalg.norm(delta)
        if(np.dot(normal,delta) < -0.5):
            final_point_cloud.normals[i] = -normal
    
    o3d.visualization.draw_geometries([final_point_cloud])
    o3d.io.write_point_cloud("PointCloudICP.pcd", final_point_cloud)
    o3d.io.write_point_cloud("PointCloudICP.xyzn", final_point_cloud)

