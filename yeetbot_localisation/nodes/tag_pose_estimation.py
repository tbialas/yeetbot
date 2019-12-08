#!/usr/bin/env python

import rospy
import numpy

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf


rospy.init_node('yeetbot_absolute_pose_estimation')
tf_listener = tf.TransformListener()


def tag_detection_cb(tag_array):
    global pub
    # Transform from the map frame to the camera frame
    try:
        (cam_trans, cam_rot) = tf_listener.lookupTransform(
            'map', tag_array.header.frame_id, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
        rospy.logwarn("Failed to transform frame '" + tag_array.header.frame_id + "'... Discarding whole message!")
        return

    T = tf.transformations.translation_matrix(cam_trans)
    R = tf.transformations.quaternion_matrix(cam_rot)

    # From map to camera
    m_T_c = tf.transformations.concatenate_matrices(T, R)

    for det in tag_array.detections:
        tag_frame = 'tag' + str(det.id[0])
        # Get the transform from the tag frame to the base_link frame
        try:
            (trans, rot) = tf_listener.lookupTransform(
                tag_frame, 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException) as e:
            rospy.logwarn("Failed to transform frame '" + tag_frame + "'... Discarding data.")
            rospy.logerr(e)
            continue

        T = tf.transformations.translation_matrix(trans)
        R = tf.transformations.quaternion_matrix(rot)

        # From tag to base link
        t_T_b = tf.transformations.concatenate_matrices(T, R)

        trans = [det.pose.pose.pose.position.x, 
                 det.pose.pose.pose.position.y, 
                 det.pose.pose.pose.position.z]
        rot = [det.pose.pose.pose.orientation.x,
               det.pose.pose.pose.orientation.y,
               det.pose.pose.pose.orientation.z,
               det.pose.pose.pose.orientation.w]

        T = tf.transformations.translation_matrix(trans)
        R = tf.transformations.quaternion_matrix(rot)

        c_T_t = tf.transformations.concatenate_matrices(T, R)

        # We now have the transform from the tag frame to the base link 
        # frame, the transform from the camera frame to the map frame, and
        # the pose of the tag in the camera frame
        
        # Multiplying these together should give the correct transform
        c_T_b = numpy.dot(c_T_t, t_T_b)
        m_T_b = numpy.dot(m_T_c, c_T_b)

        trans = tf.transformations.translation_from_matrix(m_T_b)
        rot = tf.transformations.quaternion_from_matrix(m_T_b)

        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = trans[0]
        pose.pose.pose.position.y = trans[1]
        pose.pose.pose.position.z = trans[2]
        pose.pose.pose.orientation.x = rot[0]
        pose.pose.pose.orientation.y = rot[1]
        pose.pose.pose.orientation.z = rot[2]
        pose.pose.pose.orientation.w = rot[3]

        pose.pose.covariance = [3e-2,    0,    0,    0,    0,    0,
                                0   , 3e-2,    0,    0,    0,    0,
                                0   ,    0, 3e-2,    0,    0,    0,
                                0   ,    0,    0,    0,    0,    0,
                                0   ,    0,    0,    0,    0,    0,
                                0   ,    0,    0,    0,    0, 1e-1]

        pose.header.stamp = tag_array.header.stamp
        pose.header.frame_id = 'map'

        pub.publish(pose)


def main():
    global pub
    pub = rospy.Publisher(
        "yeetbot_pose_estimate", PoseWithCovarianceStamped, queue_size=20)
    rospy.Subscriber(
        'tag_detections', AprilTagDetectionArray, tag_detection_cb)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
