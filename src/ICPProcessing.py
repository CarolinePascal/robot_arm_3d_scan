#!/usr/bin/env python3

## Definition file of the ICPProcessing class 
#
# Defines the attributes and methods used to perform ICP over successive point clouds


import rospy
import numpy as np
import copy
import sys

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
import pcl as pcl

import os

from std_msgs.msg import Int64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform

## ICPProcessing
#
# Defines the attributes and methods used to perform ICP over successive point clouds.
class ICPProcessing:

    ## thresholdExceptionError
    #
    # Defines the exception class triggered when the ICP leads to an invalid transform.
    class thresholdExeededError(Exception):

        ## Constructor
        def __init__(self):
            super(ICPProcessing.thresholdExeededError, self).__init__("INVALID TRANSFORM - INCREASE WAYPOINTS NUMBER")

    ## Constructor
    def __init__(self):

        ## Point clouds ROS subscriber
        self.pointCloudSubscriber = rospy.Subscriber("/icp_filtered_point_cloud",PointCloud2,self.callback,queue_size=100)

        ## Point cloud counter
        self.pointCloudCounter = 0

        ## Transform thresholds
        self.maxRotation = 30*np.pi/180
        self.maxTranslation = 0.5

        ## Estimated tranformations
        self.estimatedLocalTransform = np.identity(4)
        self.estimatedGlobalTransform = np.identity(4)

        ## Storage folder name
        self.storageFolder = rospy.get_param("storageFolderName")
        try:
            os.mkdir(self.storageFolder)
            rospy.loginfo("Creating " + self.storageFolder + " ...")
        except OSError:
            rospy.logwarn(self.storageFolder + "already exists : its contents will be overwritten !")
            pass 

    ## Callback method performing ICP on the recieved point clouds
    #  @param pointCloud The last received point cloud
    def callback(self, pointCloud):

        self.pointCloudCounter += 1

        #Store recieved point cloud
        o3dPointCloud = orh.rospc_to_o3dpc(pointCloud)

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

    ## Method estimating the transformation between the two last received point clouds using the ICP method
    def ICP(self):

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
        if(translationMagnitude > self.maxTranslation or np.abs(rotationAngle) > self.maxRotation):
            raise ICPProcessing.thresholdExeededError()
        else:
            return(ICPTransformation)

def main():

    #Launch ROS node
    rospy.init_node('icp_processing')

    #Launch ROS subscriber
    ICPProcessing()

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down ROS ICP processing")
            break
        except ICPProcessing.thresholdExeededError:
            print("Shutting down ROS ICP processing")
            break

if __name__ == "__main__":
    main()