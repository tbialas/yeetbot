#!/usr/bin/env python

from threading import Lock
from math import sqrt

import rospy
import tf

from geometry_msgs.msg import Pose, PoseStamped
from yeetbot_msgs.msg import YEETBotHumanPoseArray
from visualization_msgs.msg import Marker


class Human:
    def __init__(self, unique_id):
        # Calculated pose of the human in the map frame
        self.pose = Pose()
        # Timestamp that the human was last updated
        self.stamp = rospy.Time(0)
        # Unique ID of the human
        self.id = unique_id
        
        # IDs and poses used to calculate the pose of the human last
        # all poses are in the frame of the key used
        self.ids = {}
        self.poses = {}
        self.pose_stamps = {}
        self.updated_stamp = rospy.Time(0)
    def calculate_pose(self):
        dur = self.stamp - self.updated_stamp
        if dur < rospy.Duration(0.02):
            return
        self.updated_stamp = self.stamp
        pose = Pose()
        div = 0
        for key in self.poses:
            p = self.poses[key]
            dur = self.stamp - self.pose_stamps[key]
            d = (1 + dur.secs + dur.nsecs * 1e-9)
            pose.position.x += p.position.x / d
            pose.position.y += p.position.y / d
            div += 1/d
        pose.position.x /= div
        pose.position.y /= div
        
        pose.position.x *= 0.8
        pose.position.x += 0.2 * self.pose.position.x
        pose.position.y *= 0.8
        pose.position.y += 0.2 * self.pose.position.y

        pose.orientation.w = 1
        self.pose = pose

class HumanStitcher:
    def __init__(self):
        rospy.init_node('human_stitcher')

        self.lock = Lock()

        # Maximum distance between two points to be considered as possibly
        # the same human
        self.MAX_DELTA = 0.5
        # Maximum movement from human between frames to be considered as
        # the same human
        self.MAX_HUMAN_DELTA = 0.5

        self.HUMAN_LIFETIME = rospy.Duration(2)

        self.humans = []
        self.id_counter = 0

        # Time of the most recent data from a given source
        self.newest_frame = dict()
        # Array of poses from a given source the key is the frame the data
        # came from, but the data is stored relative to the map frame
        self.pose_arrays = dict()
        # Array of ids from a given source
        self.id_arrays = dict()
        self.last_updated = rospy.Time.now()
        # Maximum time between poses from different sources to be able to
        # attempt to stitch
        self.STITCH_MAX_DELAY = rospy.Duration(0.5)

        self.listener = tf.TransformListener()

        rospy.Subscriber(
            '/humandetect_door_poses', YEETBotHumanPoseArray,
            self.human_pose_cb)
        rospy.Subscriber(
            '/humandetect_window_poses', YEETBotHumanPoseArray,
            self.human_pose_cb)
        self.pub = rospy.Publisher(
            '/stitched_human_poses', YEETBotHumanPoseArray, queue_size=2)
        self.rviz_pub = rospy.Publisher(
            '/human_pose_visualization', Marker, queue_size=1)


    def human_pose_cb(self, pose_array):
        with self.lock:
            new_arr = []
            for pose in pose_array.human_poses.poses:
                ps = PoseStamped()
                ps.pose = pose
                ps.header = pose_array.header
                ps = self.listener.transformPose('map', ps)
                new_arr.append(ps.pose)
            pose_array.human_poses.poses = new_arr
            pose_array.human_poses.header.frame_id = 'map'
            self.pose_arrays[pose_array.header.frame_id] = pose_array.human_poses
            self.id_arrays[pose_array.header.frame_id] = pose_array.ids
            self.newest_frame[pose_array.header.frame_id] = rospy.Time.now()

    def run(self):
        while not rospy.is_shutdown() and len(self.pose_arrays) < 2:
            msg = "Waiting for pose arrays... Currently have " 
            for key in self.pose_arrays:
                msg += key + " " 
            rospy.sleep(1)

        # Run at 10 Hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # It's important that new data isn't updated mid-processing
            with self.lock:
                keys = list(self.id_arrays)
                most_recent_update = self.newest_frame[keys[0]]
                for key in keys:
                    t = self.newest_frame[key]
                    if t > most_recent_update:
                        most_recent_update = t
                
                update_keys = []
                for key in keys:
                    dur = most_recent_update - self.newest_frame[key]
                    if dur < self.STITCH_MAX_DELAY:
                        update_keys.append(key)

                # Stitch together info and estimate poses of humans
                self.stitch_frames(update_keys)
                # Delete humans that haven't been updated for a while
                self.kill_humans()
                # Publish the current list of humans
                self.publish_humans()

            rate.sleep()

    def stitch_frames(self, keys):
        stamp = rospy.Time.now()
        # Loop through all the keys, grab the poses, compare each pose to 
        # the previous set of poses, optimise to find the minimum error in
        # all deltas and assume that's correct

        # Loop through all humans and grab the 3 closest poses for each key
        human_closest_poses = dict()
        human_closest_ids = dict()
        # We assume that the same id is always the same person so reserve 
        # these
        reserved_ids = dict()
        for human in self.humans:
            human_closest_poses[human] = dict()
            human_closest_ids[human] = dict()
            for key in keys:
                human_closest_poses[human][key] = []
                human_closest_ids[human][key] = []
                for pose, id_ in zip(self.pose_arrays[key].poses,
                                     self.id_arrays[key]):
                    if len(human_closest_poses[human][key]) < 3:
                        human_closest_poses[human][key].append(pose)
                        human_closest_ids[human][key].append(id_)
                    else:
                        dists = [self.dist(
                            human_closest_poses[human][key][i],
                            human.pose) for i in range(3)]

                        d = self.dist(pose, human.pose)
                        max_dist = dists[0]
                        it = 0
                        for i in range(3):
                            if dists[i] > max_dist:
                                max_dist = dists[i]
                                it = i
                        if d < max_dist:
                            human_closest_poses[human][key][it] = pose
                            human_closest_ids[human][key][it] = id_

            # Also consider the poses used to calculate the pose last time
            for key in keys:
                if key not in human.ids:
                    continue
                id_ = human.ids[key]
                reserved_ids[key] = []
                for (i, p) in zip(self.id_arrays[key], self.pose_arrays[key].poses):
                    if i == id_:
                        reserved_ids[key].append(id_)
                        if id_ not in human_closest_ids[human][key]:
                            human_closest_ids[human][key].append(id_)
                            human_closest_poses[human][key].append(p)
                        break

        # We now have 2 dicts of dicts of arrays which contain all the 
        # closest matching poses and any poses from the previous round for
        # each human and each key

        # The goal now is to match a pose and id from each key, and
        # align it with an existing human. Once this has been done we can
        # take any remaining poses and ids, match them and create new humans

        # Bear in mind that it's possible (due to occlusion) that a human
        # might not be visible from every key
        used_ids = dict()
        unused_ids = dict()
        for key in keys:
            used_ids[key] = []
            unused_ids[key] = []
            for id_ in self.id_arrays[key]:
                unused_ids[key].append(id_)

        for human in self.humans:
            # Pick the closest point that hasn't been used for each key
            poses = dict()
            ids = dict()
            for key in keys:
                poses[key] = None
                ids[key] = None
                max_dist = 9e9
                for (pose, id_) in zip(human_closest_poses[human][key],
                                       human_closest_ids[human][key]):
                    # If the ID is reserved and equal to an existing ID
                    # then that ID will always be this human
                    if key in human.ids:
                        if id_ == human.ids[key] and id_ in reserved_ids[key] and id_ not in used_ids[key]:
                            poses[key] = pose
                            ids[key] = id_
                            break

                    dist = self.dist(pose, human.pose)
                    if dist < max_dist and dist < self.MAX_HUMAN_DELTA:
                        if id_ not in used_ids[key]:
                            if key not in reserved_ids:
                                max_dist = dist
                                poses[key] = pose
                                ids[key] = id_
                            elif id_ not in reserved_ids[key]:
                                max_dist = dist
                                poses[key] = pose
                                ids[key] = id_
                if ids[key] == None:
                    # This human has no matching points in this key
                    pass
                else:
                    unused_ids[key].remove(ids[key])
                    used_ids[key].append(ids[key])
                    # Update the pose of the human for this key
                    human.poses[key] = poses[key]
                    human.ids[key] = ids[key]
                    human.stamp = stamp
                    human.pose_stamps[key] = stamp

        # Now try and match the remaining points and create new humans
        # dict of [key]: dict[id]
        matching_pairs = dict()
        for key in keys:
            matching_pairs[key] = dict()
            # dict of [id]: ((point, point2, id2, key2), dist)
            for key2 in keys:
                if key == key2:
                    continue
                for (p, i) in zip(self.pose_arrays[key].poses,
                                  self.id_arrays[key]):
                    if i in used_ids[key]:
                        continue
                    for (p2, i2) in zip(self.pose_arrays[key2].poses,
                                        self.id_arrays[key2]):
                        if i2 in used_ids[key2]:
                            continue
                        dist = self.dist(p, p2)
                        if (i not in matching_pairs[key] and dist < self.MAX_DELTA):
                            matching_pairs[key][i] = []
                            matching_pairs[key][i].append((p, p2, i2, key2))
                            matching_pairs[key][i].append(dist)
                        elif i in matching_pairs[key]:
                            if dist < matching_pairs[key][i][1]:
                                matching_pairs[key][i][0] = (p, p2, i2, key2)
                                matching_pairs[key][i][1] = dist
            # Sort the dict such that the closest pairs are together
            try:
                def sort_func(id_):
                    return matching_pairs[key][id_][1]
                res = sorted(
                    matching_pairs[key], key=sort_func)
                new_dict = dict()
                for id_ in res:
                    new_dict[id_] = matching_pairs[key][id_]
                matching_pairs[key] = new_dict
            except KeyError:
                matching_pairs[key] = {}

        # Now create a list containing [(key, id, point), ...]
        sets_of_points = []
        # We take the closest pair of points in the matching pairs dict,
        # Check if one of the ids matches an existing id and add the points
        # to that set if it does. If both match an existing id then we
        # discard the points. If none match an existing id then we create a
        # new set.

        ids_to_del = []
        for key in matching_pairs:
            for id_ in matching_pairs[key]:
                if len(matching_pairs[key][id_]) == 0:
                    ids_to_del.append((key, id_))
        for (key, id_) in ids_to_del:
            del matching_pairs[key][id_]
        
        keys_to_del = []
        for key in matching_pairs:
            if len(matching_pairs[key]) == 0:
                keys_to_del.append(key)
        for key in keys_to_del:
            del matching_pairs[key]

        while len(matching_pairs) != 0:
            # Will be ((point, point2, id2, key2), dist)
            best_pair = None
            best_id = None
            best_key = None
            for key in matching_pairs:
                for id_ in matching_pairs[key]:
                    best_pair = matching_pairs[key][id_]
                    best_id = id_
                    best_key = key
                    break
            
            for key in matching_pairs:
                for id_ in matching_pairs[key]:
                    pair = matching_pairs[key][id_][0]
                    if pair[1] < best_pair[1]:
                        best_pair = pair
                        best_key = key
                        best_id = id_

            match_id1 = False
            match_id2 = False
            for group in sets_of_points:
                for tup in group:
                    if tup[1] == best_id:
                        match_id1 = True
                    if tup[1] == best_pair[0][2]:
                        match_id2 = True

            if match_id1 and match_id2:
                del matching_pairs[best_key][best_id]
            elif match_id1:
                for point in sets_of_points:
                    if len([t for t in point if t[1] == best_pair[0][2]]) != 0:
                        point.append(
                            (best_pair[0][3], best_pair[0][2], 
                             best_pair[0][1]))
                        used_ids[best_pair[0][3]].append(best_pair[0][2])
                        unused_ids[best_pair[0][3]].remove(best_pair[0][2])
                        break
            elif match_id2:
                for point in sets_of_points:
                    if len([t for t in point if t[1] == best_id]) != 0:
                        point.append(
                            (best_key, best_id, best_pair[0][0]))
                        used_ids[best_key].append(best_id)
                        unused_ids[best_key].remove(best_id)
                        break
            else:
                sets_of_points.append(
                    [
                        (best_key,        best_id,         best_pair[0][0]),
                        (best_pair[0][3], best_pair[0][2], best_pair[0][1])
                    ])
                used_ids[best_key].append(best_id)
                unused_ids[best_key].remove(best_id)
                used_ids[best_pair[0][3]].append(best_pair[0][2])
                unused_ids[best_pair[0][3]].remove(best_pair[0][2])

            ids_to_del = []
            for key in matching_pairs:
                for id_ in matching_pairs[key]:
                    if len(matching_pairs[key][id_]) == 0:
                        ids_to_del.append((key, id_))
            for (key, id_) in ids_to_del:
                del matching_pairs[key][id_]
            
            keys_to_del = []
            for key in matching_pairs:
                if len(matching_pairs[key]) == 0:
                    keys_to_del.append(key)
            for key in keys_to_del:
                del matching_pairs[key]

        # Now we have a set of points which have been grouped together - 
        # these are our new humans.

        for point in sets_of_points:
            h = Human(self.id_counter)
            self.id_counter += 1
            h.stamp = stamp
            for i in range(len(point)):
                h.ids[point[i][0]] = point[i][1]
                h.poses[point[i][0]] = point[i][2]
                h.pose_stamps[point[i][0]] = stamp
            self.humans.append(h)

        # The points not being used are visible only on one camera _and_ 
        # have not yet been matched with an existing human. If there are no
        # humans near them then we can make a new human out of them

        for key in keys:
            for id_ in unused_ids[key]:
                unique = True
                it = 0
                for id2 in self.id_arrays[key]:
                    if id2 == id_:
                        break
                    it += 1

                pose = self.pose_arrays[key].poses[it]
                for human in self.humans:
                    human.calculate_pose()
                    if self.dist(human.pose, pose) < self.MAX_DELTA:
                        unique = False

                # If there weren't any humans near this point then we can 
                # count it as a human
                if unique:
                    h = Human(self.id_counter)
                    self.id_counter += 1
                    h.stamp = stamp
                    h.ids[key] = id_
                    h.poses[key] = pose
                    h.pose_stamps[key] = stamp
                    self.humans.append(h)
                used_ids[key].append(id_)
                unused_ids[key].remove(id_)

    def dist(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return sqrt(dx*dx + dy*dy)

    def kill_humans(self):
        for h in self.humans:
            if rospy.Time.now() - h.stamp > self.HUMAN_LIFETIME:
                self.humans.remove(h)

    def publish_humans(self):
        msg = YEETBotHumanPoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.human_poses.header = msg.header
        for h in self.humans:
            h.calculate_pose()
            msg.human_poses.poses.append(h.pose)
            msg.ids.append(h.id)
        rospy.logwarn(msg)
        self.visualise(msg)
        self.pub.publish(msg)

    def visualise(self, pose_array):
        marker = Marker()
        marker.header = pose_array.header
        marker.ns = 'humans'
        
        marker.type = Marker.ARROW

        marker.pose.orientation.y = -0.7071
        marker.pose.orientation.w = 0.7071

        marker.scale.x = 2
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.6

        marker.lifetime = rospy.Duration(3)

        for (pose, id_) in zip(pose_array.human_poses.poses, 
                               pose_array.ids):
            marker.id = id_
            marker.pose.position = pose.position
            self.rviz_pub.publish(marker)

def main():
    try:
        stitcher = HumanStitcher()
        stitcher.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
