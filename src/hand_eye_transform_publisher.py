#!/usr/bin/env python3
import rospy

#Utility packages
import yaml
import sys

# Transform packages
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from tf.transformations import *

if __name__ == '__main__':
    rospy.init_node('hand_eye_transform_publisher')

    try:
        file_path = sys.argv[1]
    except IndexError:
        print("Usage: hand_eye_transform_publisher.py hand_eye_calibration_file")
        sys.exit(-1)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()

    with open(file_path, 'r') as file:

        data = yaml.load(file, Loader=yaml.FullLoader)

        from_tf = "world"
        try:
            from_tf = data["FlangeTF"]
        except IndexError:
            rospy.logwarn("Flange TF not found in the hand-eye calibration file, defaulting to " + from_tf)
            
        to_tf = "camera_link"
        try:
            to_tf = data["CameraLinkTF"]
        except IndexError:
            rospy.logwarn("CameraLink TF not found in the hand-eye calibration file, defaulting to " + to_tf)
    
        static_transformStamped.header.frame_id = from_tf
        static_transformStamped.child_frame_id = to_tf

        transform = {}
        try:
            transform = data["FlangeCameraLinkTransform"]
        except:
            rospy.logwarn("Transform from flange to camera not found in the hand-eye calibration file, defaulting to identity")
            transform = {"x":0,"y":0,"z":0,"rx":0,"ry":0,"rz":0}

        print(transform)

        static_transformStamped.transform.translation.x = transform["x"]
        static_transformStamped.transform.translation.y = transform["y"]
        static_transformStamped.transform.translation.z = transform["z"]

        quaternion = quaternion_from_euler(transform["rx"],
                                           transform["ry"],
                                           transform["rz"])

        static_transformStamped.transform.rotation.x = quaternion[0]
        static_transformStamped.transform.rotation.y = quaternion[1]
        static_transformStamped.transform.rotation.z = quaternion[2]
        static_transformStamped.transform.rotation.w = quaternion[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()


    
        



    
