#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf import transformations as t
from geometry_msgs.msg import TransformStamped, PoseStamped
 
class ArucoTransformNode:
    def __init__(self, topic):
        rospy.init_node('tf2_aruco_broadcaster')
        rospy.Subscriber(topic, TransformStamped, self.transform_callback)
        self.broadcaster = tf2_ros.TransformBroadcaster()        
        self.buffer = tf2_ros.Buffer()

    def transform_callback(self, transform: TransformStamped):        
        old_translation = (transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z)
        old_quaternion = (transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z,
                      transform.transform.rotation.w)

        # Create the transformation matrix
        transformation_matrix = t.concatenate_matrices(
            t.translation_matrix(old_translation),
            t.quaternion_matrix(old_quaternion)
        )
        inverse_transform = t.inverse_matrix(transformation_matrix)
        translation = t.translation_from_matrix(inverse_transform)
        quaternion = t.quaternion_from_matrix(inverse_transform)
        transform_stamped = TransformStamped()

        # Assuming you set the headers as needed
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = transform.child_frame_id
        transform_stamped.child_frame_id = transform.header.frame_id

        # Set the translation
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]

        # Set the rotation
        # q_rot = [-0.5, 0.5, -0.5, 0.5]
        # q_new = t.quaternion_multiply(q_rot, quaternion)
        
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(transform_stamped)
        print("yuh")
      

if __name__ == '__main__':
    node = ArucoTransformNode('/aruco_single/transform')
    rospy.spin()