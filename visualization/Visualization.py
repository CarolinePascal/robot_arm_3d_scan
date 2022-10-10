import open3d as o3d
import numpy as np
import sys
import glob
import os

try:
    Files = [sys.argv[1]]
except:
    Files = glob.glob("*.pcd")
    Files = sorted(Files, key=lambda file:int(os.path.basename(file).split(".")[0].split("_")[-1]))

axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])

for i,file in enumerate(Files):
    pointCloud = o3d.io.read_point_cloud(file)
    #o3d.visualization.draw_geometries([pointCloud])

    if(i==0):
        finalPointCloudPoints = np.asarray(pointCloud.points)
        finalPointCloudColors = np.asarray(pointCloud.colors)
    else:
        finalPointCloudPoints = np.concatenate((np.asarray(pointCloud.points),finalPointCloudPoints),axis=0)
        finalPointCloudColors = np.concatenate((np.asarray(pointCloud.colors),finalPointCloudColors),axis=0)

    finalPointCloud = o3d.geometry.PointCloud()
    finalPointCloud.points = o3d.utility.Vector3dVector(finalPointCloudPoints)
    finalPointCloud.colors = o3d.utility.Vector3dVector(finalPointCloudColors)

finalPointCloud = finalPointCloud.voxel_down_sample(voxel_size=0.001)

o3d.visualization.draw_geometries([finalPointCloud,axis])

finalPointCloudPoints = np.asarray(finalPointCloud.points)
centroid = np.mean(finalPointCloudPoints,axis=0)
distances = np.linalg.norm(finalPointCloudPoints - centroid, axis=1)
finalPointCloud.points = o3d.utility.Vector3dVector(finalPointCloudPoints[distances <= np.mean(distances) + np.std(distances)])

obb = finalPointCloud.get_axis_aligned_bounding_box()
print("BOUNDING BOX : ")
print("Center : ")
print(obb.get_center())
print("Dimensions :")
print(obb.get_extent())

o3d.visualization.draw_geometries([finalPointCloud,axis,obb])
