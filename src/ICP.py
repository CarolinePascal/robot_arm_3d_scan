#!/usr/bin/env python

#Imports
import rospy
import numpy as np
import copy
import sys

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import pcl as pcl

from std_msgs.msg import Int64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform

class ICPProcessing:
    """! ICPProcessing class
    Defines the class used to perform ICP over successive point clouds.
    """

    class thresholdExeededError(Exception):
        """! thresholdExeededError exception class
        Defines the exception class triggered when the ICP leads to an invalid transform.
        """
        def __init__(self):
            """! thresholdExeededError class initializer.
            """
            super(ICPProcessing.thresholdExeededError, self).__init__("INVALID TRANSFORM - INCREASE WAYPOINTS NUMBER")

    def __init__(self):
        """! The ICPProcessing class constructor.
        """

        ##Point cloud subscriber
        self.pointCloudSubscriber = rospy.Subscriber("/filtered_point_cloud",PointCloud2,self.callback,queue_size=100)

        ##Counter
        self.pointCloudCounter = 0

        ##Transform thresholds
        self.maxRotation = 30*np.pi/180
        self.maxTranslation = 0.5

        ##Estimated tranformation
        self.estimatedLocalTransform = np.identity(4)
        self.estimatedGlobalTransform = np.identity(4)

        ##Point clouds storage folders
        self.storageFolder = "/tmp/PointClouds/"
        
        try:
            os.mkdir(self.storageFolder)
            rospy.loginfo("The folder " + self.storageFolder + "was created")
        except OSError:
            rospy.logwarn("The folder " + self.storageFolder + "already exists : its contents will be ereased !")
            pass 

    def callback(self, pointCloud):
        """! Callback function for the filtered point cloud topic
        @param pointCloud The recieved point cloud
        """
        self.pointCloudCounter += 1

        #Store recieved point cloud
        o3dPointCloud = orh.rospc_to_o3dpc(pointCloud)
        o3d.io.write_point_cloud(self.storageFolder+"PointCloud"+str(self.pointCloudCounter)+".pcd", o3dPointCloud ,write_ascii = False)

        #First point cloud => No ICP !
        if self.pointCloudCounter == 1:     
            self.currentPointCloud = o3dPointCloud
            self.finalPointCloud = copy.deepcopy(self.currentPointCloud)
            #o3d.visualization.draw_geometries([self.currentPointCloud])

        else :
            #Update point clouds
            self.previousPointCloud = copy.deepcopy(self.currentPointCloud)
            self.currentPointCloud = o3dPointCloud
            #o3d.visualization.draw_geometries([self.currentPointCloud])

            #Estimate the transform between the two last recieved point clouds using the ICP method
            try:
                self.estimatedLocalTransform = self.ICP()

            except ICPProcessing.thresholdExeededError:
                raise ICPProcessing.thresholdExeededError()

            #Compute the global transform between the first point cloud and the current one
            self.estimatedGlobalTransform = np.dot(self.estimatedGlobalTransform, self.estimatedLocalTransform)

            #Transform the current point cloud to fit the first point cloud
            transformedPointCloud = copy.deepcopy(self.currentPointCloud)
            transformedPointCloud.transform(self.estimatedGlobalTransform)

            #Add the current point cloud to the first point cloud
            self.finalPointCloud = self.finalPointCloud + transformedPointCloud

            #Update the 3D scan
            self.finalPointCloud.voxel_down_sample(0.001)
            o3d.io.write_point_cloud(self.storageFolder+"ICPMap.pcd", self.finalPointCloud ,write_ascii = False)
    
        return()

    def ICP(self):
        """! Estimates the transform between the two last recieved point clouds using the IPC method
        """

        #Create copies...
        sourcePointCloud = copy.deepcopy(self.currentPointCloud)
        targetPointCloud = copy.deepcopy(self.previousPointCloud)

        #Simplify point clouds
        sourcePointCloud = sourcePointCloud.voxel_down_sample(0.004)
        targetPointCloud = targetPointCloud.voxel_down_sample(0.004)

        #Compute local normals
        sourcePointCloud.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))
        targetPointCloud.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))

        #Perform a two stages ICP on the local normals
        ICPTransformation = np.identity(4)
        resultICP = o3d.registration.registration_icp(sourcePointCloud,targetPointCloud,0.02,ICPTransformation,o3d.registration.TransformationEstimationPointToPlane())
        ICPTransformation = resultICP.transformation
        resultICP = o3d.registration.registration_icp(sourcePointCloud,targetPointCloud,0.01,ICPTransformation,o3d.registration.TransformationEstimationPointToPlane())
        ICPTransformation = resultICP.transformation

        #Extract translation et rotation
        rotationAngle = np.arccos((np.trace(ICPTransformation[:3,:3]) - 1)/2)
        translationMagnitude = np.linalg.norm(ICPTransformation[:3,3])

        #print(translationMagnitude)
        #print(rotationAngle)

        #Detect an eventual invalid transform
        if(translationMagnitude > self.maxTranslation or np.abs(rotationAngle) > self.maxRotation):
            raise ICPProcessing.thresholdExeededError()
        else:
            return(ICPTransformation)

def main():
    """! Main function - ROS node
    """

    #Initialise node
    rospy.init_node('ICP_Node')

    #Create an ICPProcessing object
    ICPProcessing()

    #Check for keyboard interrupt or invalid transforms
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down ROS ICP_Node")
            break
        except ICPProcessing.thresholdExeededError:
            print("Shutting down ROS ICP_Node")
            break

if __name__ == "__main__":
    main()