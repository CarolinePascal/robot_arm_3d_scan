import open3d as o3d
import sys
import os

if __name__ == "__main__":
    point_cloud = o3d.io.read_point_cloud(sys.argv[1])
    point_cloud = point_cloud.voxel_down_sample(voxel_size=0.005)
    point_cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=30))
    point_cloud.orient_normals_consistent_tangent_plane(50)   
    
    o3d.io.write_point_cloud(os.path.basename(sys.argv[1]).split(".")[0] + ".ply", point_cloud)