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

    #finalPointCloud = finalPointCloud.voxel_down_sample(voxel_size=0.001)
    o3d.visualization.draw_geometries([finalPointCloud])