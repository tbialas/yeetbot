#!/usr/bin/env python

import rospy

from apriltag_ros.msg import AprilTagDetectionArray

import tf


rospy.init_node('yeetbot_absolute_pose_estimation')
tf_listener = tf.TransformListener()


def tag_detection_cb(tag_array):
    # Transform the camera frame into the map frame
    try:
        (cam_trans, cam_rot) = tf_listener.lookupTransform(
            tag_array.header.frame_id, 'map', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
        print "Failed to transform frame '" + tag_array.header.frame_id + "'... Discarding whole message!"
        return

    cam_trans_mat = tf.transformations.translation_matrix(cam_trans)
    cam_rot_mat = tf.transformations.quaternion_matrix(cam_rot)

    for det in tag_array.detections:
        # Get the pose of the base link in the camera frame
        try:
            pose = tf_listener.transformPose('base_link', det.pose.pose)
        except (tf.LookupException, tf.ConnectivityException, 
                tf.ExtrapolationException):
            print "Failed to transform frame '" + det.pose.header.frame_id + "'... Discarding data."
            continue
        
        print "received pose:"
        print det.pose
        print "transformed pose:"
        print pose


def main():
    rospy.Subscriber(
        'tag_detections', AprilTagDetectionArray, tag_detection_cb)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
