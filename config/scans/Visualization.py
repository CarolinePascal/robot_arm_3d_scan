import open3d as o3d
import numpy as np
import time
import glob
import os

Files = glob.glob("*.pcd")

if(len(Files) == 0):
    path = os.path.dirname(os.path.realpath(__file__))
    directoryIndex = int(input("Scan directory ? " + str(glob.glob("*/"))))
    directory = glob.glob("*/")[directoryIndex-1]
    Files = glob.glob(path + "/" + directory + "*.pcd")

Files = sorted(Files, key=lambda file:int(os.path.basename(file).split(".")[0].split("_")[-1]))

axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(axis)

for i,file in enumerate(Files[1:]):
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

    vis.add_geometry(finalPointCloud)
    vis.update_geometry(finalPointCloud)
    vis.poll_events()
    vis.update_renderer()
    
    time.sleep(0.5)

finalPointCloud = finalPointCloud.voxel_down_sample(voxel_size=0.001)

finalPointCloudPoints = np.asarray(finalPointCloud.points)
centroid = np.mean(finalPointCloudPoints,axis=0)
distances = np.linalg.norm(finalPointCloudPoints - centroid, axis=1)
finalPointCloud.points = o3d.utility.Vector3dVector(finalPointCloudPoints[distances <= np.mean(distances) + 3*np.std(distances)])

obb = finalPointCloud.get_axis_aligned_bounding_box()
print("BOUNDING BOX : ")
print("Center : ")
print(obb.get_center())
print("Dimensions :")
print(obb.get_extent())

o3d.visualization.draw_geometries([axis,finalPointCloud,axis,obb])
