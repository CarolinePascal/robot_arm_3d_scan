from ctypes.util import find_library
import open3d as o3d
from copy import deepcopy
import numpy as np

maxRotation = 30*np.pi/180
maxTranslation = 0.5

N = 57
estimatedGlobalTransform = np.eye(4)
previousPointCloud = o3d.io.read_point_cloud("/home/caroline/Bureau/PointClouds/#3/PointCloud1.pcd")

vol = o3d.visualization.read_selection_polygon_volume("crop.json")

previousPointCloud = vol.crop_point_cloud(previousPointCloud)

finalPointCloud = deepcopy(previousPointCloud)

def ICP(currentPointCloud,previousPointCloud):

    #Create copies...
    sourcePointCloud = deepcopy(currentPointCloud)
    targetPointCloud = deepcopy(previousPointCloud)

    #Simplify point clouds
    sourcePointCloud = sourcePointCloud.voxel_down_sample(0.004)
    targetPointCloud = targetPointCloud.voxel_down_sample(0.004)

    #Compute local normals
    sourcePointCloud.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
    targetPointCloud.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))

    #Perform a two stages ICP on the local normals
    ICPTransformation = np.identity(4)
    resultICP = o3d.pipelines.registration.registration_icp(sourcePointCloud,targetPointCloud,0.02,ICPTransformation,o3d.pipelines.registration.TransformationEstimationPointToPlane())
    ICPTransformation = resultICP.transformation
    resultICP = o3d.pipelines.registration.registration_icp(sourcePointCloud,targetPointCloud,0.01,ICPTransformation,o3d.pipelines.registration.TransformationEstimationPointToPlane())
    ICPTransformation = resultICP.transformation

    #Extract translation et rotation
    rotationAngle = np.arccos((np.trace(ICPTransformation[:3,:3]) - 1)/2)
    translationMagnitude = np.linalg.norm(ICPTransformation[:3,3])

    #print(translationMagnitude)
    #print(rotationAngle)

    #Detect an eventual invalid transform
    if(translationMagnitude > maxTranslation or np.abs(rotationAngle) > maxRotation):
        raise ValueError  
    else:
        return(ICPTransformation)

for i in range(1,N):
    currentPointCloud = o3d.io.read_point_cloud("/home/caroline/Bureau/PointClouds/#3/PointCloud"+str(i+1)+".pcd")
    currentPointCloud = vol.crop_point_cloud(currentPointCloud)

    try:
        T = np.eye(4)
        #T = ICP(currentPointCloud,previousPointCloud)
    except ValueError:
        T = np.eye(4)

    estimatedGlobalTransform = np.dot(estimatedGlobalTransform, T)

    transformedPointCloud = deepcopy(currentPointCloud)
    transformedPointCloud.transform(estimatedGlobalTransform)

    finalPointCloud = finalPointCloud + transformedPointCloud
    finalPointCloud.voxel_down_sample(0.001) 

    previousPointCloud = deepcopy(currentPointCloud)

vol2 = o3d.visualization.read_selection_polygon_volume("crop2.json")
finalPointCloud = vol2.crop_point_cloud(finalPointCloud)

o3d.visualization.draw_geometries([finalPointCloud])


