#System packages
import os
import time

#Utility packages
import numpy as np
import glob

#Mesh and 3D modeling packages
import open3d as o3d
import trimesh

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

    ### Add robot (optionnal) 

    addRobot = input("Add robot ? y/n")

    if(addRobot == "y"):

        # Get robot URDF
        import rospkg
        import subprocess

        robotName = input("Robot name ?")
        toolName = input("Tool name ?")
        
        xacro = "echo $(xacro $(rospack find robot_arm_tools)/config/tools/" + toolName + "/" + robotName + "/" + robotName + "_" + toolName + ".urdf.xacro)"
        xacroProcess = subprocess.run(xacro,shell=True,stdout=subprocess.PIPE)
        xacroOutput = str(xacroProcess.stdout)[2:-3]

        import re
        packages = np.unique(np.array([xacroOutput[int(m.start()):int(m.end())] for m in re.finditer('package://[a-zA-Z_]+/', xacroOutput)]))

        for package in packages:
            rosFind = "rospack find " + package.split("/")[-2]
            rosFindProcess = subprocess.run(rosFind,shell=True,stdout=subprocess.PIPE)
            rosFindOutput = str(rosFindProcess.stdout)[2:-3] + "/" 
            xacroOutput = xacroOutput.replace(package,rosFindOutput)

        xacroOutput = xacroOutput.replace("file://","")

        URDFfile = open("/tmp/tmp.urdf", "w")
        URDFfile.write(xacroOutput)
        URDFfile.close()

        # Build meshes
        from urdfpy import URDF
        cfg = {}
        RobotMeshes = []

        for i,configuration in enumerate(Configurations):

            print("Generating robot mesh " + str(i+1) + " on " + str(len(Configurations)))

            robot = URDF.load("/tmp/tmp.urdf")

            for i,joint in enumerate(robot.actuated_joints):
                cfg[joint.name] = configuration[i]

            meshes = robot.visual_trimesh_fk(cfg=cfg)
            mesh = trimesh.Trimesh()

            for submesh in meshes:
                pose = meshes[submesh]
                submesh.apply_transform(pose)
                mesh = trimesh.util.concatenate([submesh,mesh])
                
            RobotMeshes.append(mesh.as_open3d)


    ### Setting up the visualization loop

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    #Add robot base frame
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
    vis.add_geometry(axis)

    if(len(Configurations) < len(Files)):
        Files = Files[1:]

    for i,file in enumerate(Files):
        pointCloud = o3d.io.read_point_cloud(file)

        #DEBUG
        #o3d.visualization.draw_geometries([pointCloud])

        if(i==0):
            finalPointCloudPoints = np.asarray(pointCloud.points)
            finalPointCloudColors = np.asarray(pointCloud.colors)

            # Set up the visualization view point depending on the scanned geometry
            maxPoint = np.max(finalPointCloudPoints,axis=0)
            mainAxis = np.argmax(maxPoint)
            if(mainAxis == 2):
                mainAxis = np.argsort(maxPoint)[1]

        else:
            vis.remove_geometry(finalPointCloud,False)
            finalPointCloudPoints = np.concatenate((np.asarray(pointCloud.points),finalPointCloudPoints),axis=0)
            finalPointCloudColors = np.concatenate((np.asarray(pointCloud.colors),finalPointCloudColors),axis=0)

            if(addRobot == "y"):
                vis.remove_geometry(robotMesh,False)

        finalPointCloud = o3d.geometry.PointCloud()
        finalPointCloud.points = o3d.utility.Vector3dVector(finalPointCloudPoints)
        finalPointCloud.colors = o3d.utility.Vector3dVector(finalPointCloudColors)

        if(i==0):
            vis.add_geometry(finalPointCloud)
        else:
            vis.add_geometry(finalPointCloud,False)

        if(addRobot == "y"):
            robotMesh = RobotMeshes[i]
            if(i==0):
                vis.add_geometry(robotMesh)
            else:
                vis.add_geometry(robotMesh,False)

        view_ctl = vis.get_view_control()
        view_ctl.set_up([0, 0, 1])
        if(mainAxis == 0):
            view_ctl.set_front([0, 1/np.sqrt(2), 1/np.sqrt(2)])
        elif(mainAxis == 1):
            view_ctl.set_front([1/np.sqrt(2), 0, 1/np.sqrt(2)])
        vis.update_renderer()
        vis.poll_events()
        
        time.sleep(0.25)

    vis.destroy_window()

    #Optionnal
    finalPointCloud = finalPointCloud.voxel_down_sample(voxel_size=0.0001)

    ### Compute bounding box
    finalPointCloudPoints = np.asarray(finalPointCloud.points)
    finalPointCloudColors = np.asarray(finalPointCloud.colors)

    centroid = np.mean(finalPointCloudPoints,axis=0)
    distances = np.linalg.norm(finalPointCloudPoints - centroid, axis=1)
    finalPointCloud.points = o3d.utility.Vector3dVector(finalPointCloudPoints[distances <= np.mean(distances) + 3*np.std(distances)])
    finalPointCloud.colors = o3d.utility.Vector3dVector(finalPointCloudColors[distances <= np.mean(distances) + 3*np.std(distances)])

    obb = finalPointCloud.get_axis_aligned_bounding_box()
    obb.color = [1,0,0]
    print("BOUNDING BOX : ")
    print("Center : ")
    print(obb.get_center())
    print("Dimensions :")
    print(obb.get_extent())

    ### Display final point cloud
    finalVis = o3d.visualization.Visualizer()
    finalVis.create_window()
    finalVis.add_geometry(axis)
    finalVis.add_geometry(finalPointCloud)
    finalVis.add_geometry(obb)
    finalVis.run()
    finalVis.destroy_window()

    #mainAxis = np.argmax(np.array(obb.get_center()))
    #if(mainAxis == 2):
    #    mainAxis = np.argsort(np.array(obb.get_center()))[1]

    #finalVis1 = o3d.visualization.Visualizer()
    #finalVis1.create_window()
    #finalVis1.add_geometry(axis)
    #finalVis1.add_geometry(finalPointCloud)
    #finalVis1.add_geometry(obb)
    #view_ctl = finalVis1.get_view_control()
    #view_ctl.set_up([0, 0, 1])
    #if(mainAxis == 0):
    #    view_ctl.set_front([0, 1/np.sqrt(2), 1/np.sqrt(2)])
    #elif(mainAxis == 1):
    #    view_ctl.set_front([1/np.sqrt(2), 0, 1/np.sqrt(2)])
    #view_ctl.set_lookat(obb.get_center())
    #finalVis1.run()
    #finalVis1.destroy_window()

    #finalVis2 = o3d.visualization.Visualizer()
    #finalVis2.create_window()
    #finalVis2.add_geometry(axis)
    #finalVis2.add_geometry(finalPointCloud)
    #finalVis2.add_geometry(obb)
    #view_ctl = finalVis2.get_view_control()
    #view_ctl.set_up([0, 0, 1])
    #if(mainAxis == 0):
    #    view_ctl.set_front([0, -1/np.sqrt(2), 1/np.sqrt(2)])
    #elif(mainAxis == 1):
    #    view_ctl.set_front([-1/np.sqrt(2), 0, 1/np.sqrt(2)])
    #view_ctl.set_lookat(obb.get_center())
    #finalVis2.run()
    #finalVis2.destroy_window()

    from functools import partial

    def rotate_view(vis):
        global theta
        view_ctl = vis.get_view_control()
        view_ctl.set_front([np.cos(theta)/np.sqrt(2), np.sin(theta)/np.sqrt(2), 1/np.sqrt(2)])
        theta += np.pi*ratio
        return False

    def speedUp(vis):
        global ratio
        ratio += 0.001
        return False

    def speedDown(vis):
        global ratio
        ratio -= 0.001
        return False

    animatedVis = o3d.visualization.VisualizerWithKeyCallback()
    animatedVis.create_window()
    animatedVis.add_geometry(finalPointCloud)
    animatedVis.add_geometry(obb)

    theta = 0
    ratio = 0.005
    view_ctl = animatedVis.get_view_control()
    view_ctl.set_up([0,0,1])
    view_ctl.set_front([np.cos(theta)/np.sqrt(2), np.sin(theta)/np.sqrt(2), 1/np.sqrt(2)])
    view_ctl.set_lookat(obb.get_center())

    animatedVis.register_animation_callback(rotate_view)
    animatedVis.register_key_callback(263, partial(speedDown))
    animatedVis.register_key_callback(262, partial(speedUp))

    animatedVis.run()
    animatedVis.destroy_window()