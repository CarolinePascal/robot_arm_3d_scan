#System packages
import sys

#Utility packages
import matplotlib.pyplot as plt 
import numpy as np

#Mesh and 3D modeling packages
import open3d as o3d

if __name__ == "__main__":

    point_cloud = o3d.io.read_point_cloud(sys.argv[1])

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            point_cloud.cluster_dbscan(eps=0.01, min_points=50, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    point_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([point_cloud])

