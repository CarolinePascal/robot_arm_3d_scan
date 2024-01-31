import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import copy

if __name__ == "__main__":
    point_cloud = o3d.io.read_point_cloud("PointCloudICP.pcd")
    point_cloud = point_cloud.voxel_down_sample(voxel_size=0.005)
    point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
    point_cloud.orient_normals_consistent_tangent_plane(50)   
    
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=5)
    
    #mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, 0.04)
    #mesh.compute_vertex_normals()
    
    #radii = [0.005, 0.01, 0.02, 0.04, 0.08]
    #mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(point_cloud, o3d.utility.DoubleVector(radii))
    
    o3d.visualization.draw_geometries([mesh])