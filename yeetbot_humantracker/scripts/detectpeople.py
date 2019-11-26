#!/usr/bin/env python3

"""
@Tobias Fischer (t.fischer@imperial.ac.uk)
Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
"""

from __future__ import print_function, division, absolute_import
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

import sys
print(sys.path)
if ros_path in sys.path:
    sys.path.remove(ros_path)

import cv2

from cv_bridge import CvBridge
sys.path.append(ros_path)


import os
import rospy
import rospkg
import message_filters

from sensor_msgs.msg import Image


import numpy as np
import tensorflow as tf


class ROSTensorFlow(object):
    def __init__(self):
        # Setup tensorflow (v1.14 / v2.0 compatible)
        #tf.compat.v1.disable_eager_execution()

        self.cv_bridge1 = CvBridge()

        self.sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("/humandetect_image", Image, queue_size=1)
        path_add = os.path.join(rospkg.RosPack().get_path("yeetbot_humantracker"), "models/research")
        sys.path.append(path_add)
        os.path.join(path_add, "slim")
        sys.path.append(path_add)
        objectdetect_path = os.path.join(rospkg.RosPack().get_path("yeetbot_humantracker"), "models/research/object_detection")
        sys.path.append(objectdetect_path)
        print(sys.path)
        from utils import label_map_util
        from utils import visualization_utils as viz_utils

        self.label_map_util1 = label_map_util
        self.viz_utils1 = viz_utils

        MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

        CWD_PATH = os.getcwd()

        PATH_TO_CKPT = os.path.join(objectdetect_path,MODEL_NAME,'frozen_inference_graph.pb')

        PATH_TO_LABELS = os.path.join(objectdetect_path, 'data', 'mscoco_label_map.pbtxt')

        NUM_CLASSES = 90

        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)

        self.sess = None
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.sess = tf.Session(graph=detection_graph)
        
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        self.frame_rate_calc = 1
        self.freq = cv2.getTickFrequency()
        self.font = cv2.FONT_HERSHEY_SIMPLEX






        print('Init done')

    def callback(self, msg):
        t1 = cv2.getTickCount()
        print(t1)

        frame = self.cv_bridge1.imgmsg_to_cv2(msg, desired_encoding="passthrough").copy()
        #cv2.imshow('Test', frame)
        # probably you will need to do some pre-processing like image resizing etc.
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_expanded = np.expand_dims(frame_rgb, axis=0)

        with self.sess.graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
        
                feed_dict={self.image_tensor: frame_expanded})

            self.viz_utils1.visualize_boxes_and_labels_on_image_array(
                frame,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                line_thickness=8,
                min_score_thresh=0.70)
            print("FPS: {0:.2f}".format(self.frame_rate_calc))
            cv2.imshow('Object detector', frame)

            t2 = cv2.getTickCount()
            time1 = (t2-t1)/self.freq
            frame_rate_calc = 1/time1

        out_message = self.cv_bridge1.cv2_to_imgmsg(frame)
        out_message.header.stamp = msg.header.stamp

        try:
            self.pub.publish(out_message)
        except rospy.ROSException as e:
            raise e


        

        ## This is a trick specific to ROS
        ## As in ROS, the callback runs in a separate thread
        ## Which is not typically the case in Python scripts!
        #with self.graph.as_default():
        #    tf.compat.v1.keras.backend.set_session(self.sess)
        #    prediction = self.model.predict(sample_X, verbose=0).argmax(1)
        #    # Now, do whatever you want with the prediction ..
        #    print('Prediction object class: {:10s}'.format(self.labels[prediction[0]]), end='\r', flush=True)


if __name__ == "__main__":
    try:
        rospy.init_node("yeetbot_humandetect")
        ros_tf = ROSTensorFlow()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("See ya")
    except rospy.ROSException as e:
        if str(e) == "publish() to a closed topic":
            print("See ya")
        else:
            raise e
    except KeyboardInterrupt:
        print("Shutting down")
    print()  # print new line
