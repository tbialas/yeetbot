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

from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PoseArray


import numpy as np
import tensorflow
import tf


class ROSTensorFlow(object):
    def __init__(self):
        # Setup tensorflow (v1.14 / v2.0 compatible)
        #tf.compat.v1.disable_eager_execution()
        #np.set_printoptions(threshold=sys.maxsize)
        # Init CV bridge
        self.cv_bridge1 = CvBridge()


        # Setup raytracing and transform
        cam_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=None)
        self.img_proc = PinholeCameraModel()
        self.img_proc.fromCameraInfo(cam_info)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.camera_frame = 'camera_rgb_optical_frame'

        # Subscribe to RGB and D data topics
        self.sub_rgb = message_filters.Subscriber("/camera/rgb/image_color", Image)
        self.sub_d = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)

        
        # Setup publishing topics
        self.pub_rgb = rospy.Publisher("/humandetect_imagergb", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("/humandetect_imagedepth", Image, queue_size=1)
        self.pub_pose = rospy.Publisher("/humandetect_poses", PoseArray, queue_size=1)

        # Synchronisation
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_rgb, self.sub_d], 10, 0.1, allow_headerless=False)


        # Setup Tensorflow object detection
        # Tensorflow path setup
        path_add = os.path.join(rospkg.RosPack().get_path("yeetbot_humantracker"), "models/research")
        sys.path.append(path_add)
        os.path.join(path_add, "slim")
        sys.path.append(path_add)
        objectdetect_path = os.path.join(rospkg.RosPack().get_path("yeetbot_humantracker"), "models/research/object_detection")
        sys.path.append(objectdetect_path)
        print(sys.path)
        
        # Import object detection API utils
        from utils import label_map_util
        from utils import visualization_utils as viz_utils

        self.label_map_util1 = label_map_util
        self.viz_utils1 = viz_utils

        # Tensorflow model setup
        MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

        CWD_PATH = os.getcwd()

        PATH_TO_CKPT = os.path.join(objectdetect_path,MODEL_NAME,'frozen_inference_graph.pb')

        PATH_TO_LABELS = os.path.join(objectdetect_path, 'data', 'mscoco_label_map.pbtxt')

        NUM_CLASSES = 90

        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)
        print(self.category_index)

        # Tensorflow session setup
        self.sess = None
        detection_graph = tensorflow.Graph()
        with detection_graph.as_default():
            od_graph_def = tensorflow.GraphDef()
            with tensorflow.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tensorflow.import_graph_def(od_graph_def, name='')
            self.sess = tensorflow.Session(graph=detection_graph)
        
        # Tensorflow detection output setup
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        self.frame_rate_calc = 1
        self.freq = cv2.getTickFrequency()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Callback register
        self.ts.registerCallback(self.callback)
        print('Init done')

    def callback(self, color_msg, depth_msg):
        t1 = cv2.getTickCount()
        print(t1)

        # Receive camera data
        frame_color = self.cv_bridge1.imgmsg_to_cv2(color_msg, desired_encoding="passthrough").copy()
        frame_depth = self.cv_bridge1.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").copy()
     
        # Convert to format
        frame_rgb = cv2.cvtColor(frame_color, cv2.COLOR_BGR2RGB)
        frame_expanded = np.expand_dims(frame_rgb, axis=0)

        # Get image geometry
        rgb_height, rgb_width, rgb_channels = frame_rgb.shape
        depth_height, depth_width = frame_depth.shape
        print("RGB INFO")
        print("{0} {1} {2}".format(rgb_height, rgb_width, rgb_channels))
        print("DEPTH INFO")
        print("{0} {1}".format(depth_height, depth_width))

        # Find people on RGB image with Tensorflow

        with self.sess.graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
        
                feed_dict={self.image_tensor: frame_expanded})
            #print("CLASSES")
            #print(classes)
            #print("NUM")
            #print(int(num[0]))
            #print("BOXES")


            # Find people on image from Tensorflow results
            boxes_coords = list()
            center_points = list()
            depth_coords = list()
            depth_frames = list()
            depths = list()
            for i in range(int(num[0])):
                if (int(classes[0][i]) == 1 and scores[0][i] >= 0.70): # if human detected
                    box = [None]*2
                    box_depth = [None]*2
                    min_y = int(boxes[0][i][0]*rgb_height) 
                    min_x = int(boxes[0][i][1]*rgb_width)
                    max_y = int(boxes[0][i][2]*rgb_height)
                    max_x = int(boxes[0][i][3]*rgb_width)
                    
                    box[0] = (min_x, min_y)
                    box[1] = (max_x, max_y)

                    center_x = int((min_x + max_x)/2)
                    center_y = int((min_y + max_y)/2)
                    center = (center_x, center_y)

                    box_w = max_x - min_x
                    box_h = max_y - min_y

                    depth_min_y = int(center_y - box_h/4)
                    depth_max_y = int(center_y + box_h/4)
                    depth_min_x = int(center_x - box_w/4)
                    depth_max_x = int(center_x + box_w/4)

                    box_depth[0] = (depth_min_x, depth_min_y)
                    box_depth[1] = (depth_max_x, depth_max_y)

                    boxes_coords.append(box)
                    depth_coords.append(box_depth)
                    center_points.append(center)

                    depth_frame = frame_depth[depth_min_y:depth_max_y, depth_min_x:depth_max_x]
                    np.nan_to_num(depth_frame, 0)
                    depth_median = np.median(depth_frame)
                    #print("TYPE")
                    #print(type(depth_frame))
                    #print(depth_frame)
                    depth_frames.append(depth_frame)
                    depths.append(depth_median)
                    #print("OBJECT {0} HAS DEPTH {1}".format(i, depth_median))

            #print(depth_frame[0])
            #print("BOXES COORDS")
            #print(boxes_coords)

            #self.viz_utils1.visualize_boxes_and_labels_on_image_array(
            #    frame_color,
            #    np.squeeze(boxes),
            #    np.squeeze(classes).astype(np.int32),
            #    np.squeeze(scores),
            #    self.category_index,
            #    use_normalized_coordinates=True,
            #    line_thickness=8,
            #    min_score_thresh=0.70)

            #cv2.imshow('Object detector', frame_color)

            t2 = cv2.getTickCount()
            time1 = (t2-t1)/self.freq
            frame_rate_calc = 1/time1

            print("FPS: {0:.2f}".format(frame_rate_calc))



        #print(boxes_coords)
        #print(depth_coords)
        for box in boxes_coords:
            cv2.rectangle(frame_color, box[0], box[1], (255, 0, 0), 2)
        
        for depth_box in depth_coords:
            cv2.rectangle(frame_color, depth_box[0], depth_box[1], (0, 255, 0), 2)
        
        for point in center_points:
            print(point)
            cv2.circle(frame_color, point, 5, (0, 0, 255), 2)
        #boxes_coords = list()
        #center_points = list()
        #depth_coords = list()
        #depth_frames = list()
        #depths = list()
        
        out_pose = PoseArray()
        out_pose.header.stamp = color_msg.header.stamp
        out_pose.header.frame_id = self.camera_frame
        poses1 = list()
        for i in range(len(center_points)):
            print("OBJECT {0} AT {1} DEPTH {2}".format(i, center_points[i], depths[i]))
            x, y, _ = self.img_proc.projectPixelTo3dRay(center_points[i])
            print(x)
            print(y)
            
            z = depths[i]
            print(z)
            self.tf_broadcaster.sendTransform([x, y, z], tf.transformations.quaternion_from_euler(0,0,0), color_msg.header.stamp, '/humanpos_{0}'.format(i), self.camera_frame)
            
            quat = tf.transformations.quaternion_from_euler(0,1.5,0)

            pose1 = Pose()
            pose1.position.x = x
            pose1.position.y = y
            pose1.position.z = z
            pose1.orientation.x = quat[0]
            pose1.orientation.y = quat[1]
            pose1.orientation.z = quat[2]
            pose1.orientation.w = quat[3]
            poses1.append(pose1)
        
        out_pose.poses = poses1
        self.pub_pose.publish(out_pose)



        out_message = self.cv_bridge1.cv2_to_imgmsg(frame_color)
        #print("TYPE2")
        #print(type(depth_frame[0]))
        #print("TYPE3")
        #print(type(frame_color))
        #out_message2 = self.cv_bridge1.cv2_to_imgmsg(depth_frames[0])
        out_message.header.stamp = color_msg.header.stamp
        #out_message2.header.stamp = depth_msg.header.stamp

        try:
            self.pub_rgb.publish(out_message)
            #self.pub_depth.publish(out_message2)
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
