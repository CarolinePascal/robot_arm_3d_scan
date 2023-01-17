#System packages
import os
import time

#Utility packages
import numpy as np
import glob
import yaml

#Mesh and 3D modeling packages
import open3d as o3d
import trimesh as tr 

#Rotation package
from scipy.spatial.transform import Rotation as R

def writeSphere(yamlFile,objectName,pose,radius):
    sphere = {}
    sphere[objectName] = {}
    sphere[objectName]["type"] = "sphere"
    sphere[objectName]["pose"] = {}
    sphere[objectName]["pose"]["x"] = float(pose[0])
    sphere[objectName]["pose"]["y"] = float(pose[1])
    sphere[objectName]["pose"]["z"] = float(pose[2])
    sphere[objectName]["pose"]["rx"] = float(pose[3])
    sphere[objectName]["pose"]["ry"] = float(pose[4])
    sphere[objectName]["pose"]["rz"] = float(pose[5])
    sphere[objectName]["size"] = {}
    sphere[objectName]["size"]["radius"] = float(radius)
    sphere[objectName]["collisions"] = "true"
    sphere[objectName]["robot_base_collisions"] = "true"
    
    with open(yamlFile, "a+") as file:
        yaml.dump(sphere, file)

def writeCylinder(yamlFile,objectName,pose,radius,height):
    cylinder = {}
    cylinder[objectName] = {}
    cylinder[objectName]["type"] = "cylinder"
    cylinder[objectName]["pose"] = {}
    cylinder[objectName]["pose"]["x"] = float(pose[0])
    cylinder[objectName]["pose"]["y"] = float(pose[1])
    cylinder[objectName]["pose"]["z"] = float(pose[2])
    cylinder[objectName]["pose"]["rx"] = float(pose[3])
    cylinder[objectName]["pose"]["ry"] = float(pose[4])
    cylinder[objectName]["pose"]["rz"] = float(pose[5])
    cylinder[objectName]["size"] = {}
    cylinder[objectName]["size"]["radius"] = float(radius)
    cylinder[objectName]["size"]["height"] = float(height)
    cylinder[objectName]["collisions"] = "true"
    cylinder[objectName]["robot_base_collisions"] = "true"

    with open(yamlFile, "a+") as file:
        yaml.dump(cylinder, file)

def writeBox(yamlFile,objectName,pose,dx,dy,dz):
    box = {}
    box[objectName] = {}
    box[objectName]["type"] = "box"
    box[objectName]["pose"] = {}
    box[objectName]["pose"]["x"] = float(pose[0])
    box[objectName]["pose"]["y"] = float(pose[1])
    box[objectName]["pose"]["z"] = float(pose[2])
    box[objectName]["pose"]["rx"] = float(pose[3])
    box[objectName]["pose"]["ry"] = float(pose[4])
    box[objectName]["pose"]["rz"] = float(pose[5])
    box[objectName]["size"] = {}
    box[objectName]["size"]["dx"] = float(dx)
    box[objectName]["size"]["dy"] = float(dy)
    box[objectName]["size"]["dz"] = float(dz)
    box[objectName]["collisions"] = "true"
    box[objectName]["robot_base_collisions"] = "true"

    print(box)

    with open(yamlFile, "a+") as file:
        yaml.dump(box, file)
    
if __name__ == "__main__":

    ### Get point clouds
    directory = os.getcwd() + "/"
    Files = glob.glob(directory + "*.pcd")

    if(len(Files) == 0):
        Directories = glob.glob(os.path.dirname(os.path.realpath(__file__)) + "/*/")
        try:
            directoryIndex = int(input("Scan directory ? (default to first directory) " + str(list(zip(np.arange(1,len(Directories)+1),[os.path.basename(directory[:-1]) for directory in Directories])))))
            directory = Directories[directoryIndex-1]
        except:
            directory = Directories[0]
        Files = glob.glob(directory + "*.pcd")

    Files = sorted(Files, key=lambda file:int(os.path.basename(file).split(".")[0].split("_")[-1]))

    ### Get robot configurations
    import csv

    configurationsFile = glob.glob(directory + "*.csv")[0]
    Configurations = []
    with open(configurationsFile) as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            Configurations.append(np.array(row[7:],dtype=float))
    Configurations = np.array(Configurations)

    ### Setting up the building loop

    if(len(Configurations) < len(Files)):
        Files = Files[1:]

    for i,file in enumerate(Files):
        pointCloud = o3d.io.read_point_cloud(file)

        #DEBUG
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

    ### Filter point cloud

    #Optionnal
    finalPointCloud = finalPointCloud.voxel_down_sample(voxel_size=0.0001)

    finalPointCloudPoints = np.asarray(finalPointCloud.points)
    finalPointCloudColors = np.asarray(finalPointCloud.colors)

    centroid = np.mean(finalPointCloudPoints,axis=0)
    distances = np.linalg.norm(finalPointCloudPoints - centroid, axis=1)
    finalPointCloud.points = o3d.utility.Vector3dVector(finalPointCloudPoints[distances <= np.mean(distances) + 3*np.std(distances)])
    finalPointCloud.colors = o3d.utility.Vector3dVector(finalPointCloudColors[distances <= np.mean(distances) + 3*np.std(distances)])

    ### Create collision volumes : points clustering and volume shape optimization
    Nmax = 5
    deltaVolumes = 10e9 #Arbitrarly high value
    collisionVolumes = []

    voxelGrid = o3d.geometry.VoxelGrid.create_from_point_cloud(finalPointCloud,voxel_size=0.0001)
    voxelGridVolume = voxelGrid.voxel_size*len(voxelGrid.get_voxels())

    print("Computing object collision volumes - Maximum number of volumes : " + str(Nmax))

    finalPointCloudPoints = np.asarray(finalPointCloud.points)

    for n in range(Nmax):
        #Cluster point cloud using the k means method
        _,labels = tr.points.k_means(finalPointCloudPoints,n+1)
        volumesSum = 0

        for i in range(n+1):
            #Get cluster and closest primitive (sphere, cylinder, box)
            pointCloud = tr.PointCloud(finalPointCloudPoints[np.where(labels==i)])
            boundingVolume = pointCloud.bounding_primitive
            
            #Update collision volumes
            collisionVolumes.append(boundingVolume)
            volumesSum += boundingVolume.volume

        #If the volumic approximation of the point cloud is increased : keep the lastly added collision volumes
        if(volumesSum - voxelGridVolume < deltaVolumes):
            deltaVolumes = volumesSum - voxelGridVolume
            collisionVolumes = collisionVolumes[-(n+1):]
        #Else : keep the previously added collision volumes
        else:
            collisionVolumes = collisionVolumes[:len(collisionVolumes) - (n+1)]

    collisionVolumes = np.array(collisionVolumes)

    print("Best solution found with " + str(len(collisionVolumes)) + " collision volumes")

    ### Write and save collision volumes description
    
    yamlFile = directory + "CollisionVolumes.yaml"
    if(os.path.isfile(yamlFile)):
        print("[WARNING] " + yamlFile + " already exists, its contents will be overwritten !")
        os.remove(yamlFile)
        
    for i,volume in enumerate(collisionVolumes):

        r  = R.from_matrix(volume.primitive.transform[:3,:3])
        eulerAngles = r.as_euler('xyz')
        pose = [np.round(volume.primitive.transform[0,-1],4),
                np.round(volume.primitive.transform[1,-1],4),
                np.round(volume.primitive.transform[2,-1],4),
                np.round(eulerAngles[0],4),
                np.round(eulerAngles[1],4),
                np.round(eulerAngles[2],4)]

        if(type(volume) is tr.primitives.Sphere):
            writeSphere(yamlFile,"object"+str(i+1),pose,np.round(volume.primitive.radius,4))       
        elif(type(volume) is tr.primitives.Cylinder):
            writeCylinder(yamlFile,"object"+str(i+1),pose,np.round(volume.primitive.radius,4),np.round(volume.primitive.height,4))  
        elif(type(volume) is tr.primitives.Box):
            writeBox(yamlFile,"object"+str(i+1),pose,np.round(volume.primitive.extents[0],4),np.round(volume.primitive.extents[1],4),np.round(volume.primitive.extents[2],4))

        print("Collision volume " + str(i+1) + "/" + str(len(collisionVolumes)))
        print("\t" + str(volume.to_dict()))

    #DEBUG
    #scene = tr.Scene()
    #displayPointCloud = tr.points.PointCloud(finalPointCloudPoints)
    #scene.add_geometry(displayPointCloud)
    #for i,volume in enumerate(collisionVolumes):
    #    scene.add_geometry(volume.as_outline())
    #scene.show()
