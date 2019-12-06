#!/usr/bin/env python

from threading import Lock
from math import sqrt

import rospy
import tf

from geometry_msgs.msg import Pose
from yeetbot_msgs.msg import YEETBotHumanPoseArray


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
            '/door_tracker', YEETBotHumanPoseArray, self.human_pose_cb)
        rospy.Subscriber(
            '/windo_tracker', YEETBotHumanPoseArray, self.human_pose_cb)
        self.pub = rospy.Publisher(
            '/stitched_human_poses', YEETBotHumanPoseArray, queue_size=2)


    def human_pose_cb(self, pose_array):
        with self.lock:
            new_arr = []
            for pose in human_poses.poses:
                new_arr.append(self.listener.transformPose('map', pose))
            pose_array.human_poses.poses = new_arr
            pose_array.human_poses.header.frame_id = 'map'
            self.pose_arrays[pose_array.header.frame_id] = pose_array.human_poses
            self.id_arrays[pose_array.header.frame_id] = pose_array.ids
            self.newest_frame[pose_array.header.frame_id] = rospy.Time.now()

    def run(self):
        while not rospy.is_shutdown() and len(self.pose_arrays) < 2:
            rospy.sleep(0.1)

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
                stitch_frames(update_keys)
                # Delete humans that haven't been updated for a while
                kill_humans()
                # Publish the current list of humans
                publish_humans()

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
                for pose, id_ in zip(self.poses[key].poses, self.ids[key]:
                    if len(human_closest_poses[human][key] < 3):
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
                for (i, p) in zip(self.ids[key], self.poses[key]):
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
            for id_ in self.ids[key]:
                unused_ids[key].append(id_)

        for human in self.humans:
            # Pick the closest point that hasn't been used for each key
            poses = dict()
            ids = dict()
            for key in key:
                poses[key] = None
                ids[key] = None
                max_dist = 9e9
                for (pose, id_) in zip(human_closest_poses[human][key],
                                       human_closest_ids[human][key]):
                    # If the ID is reserved and equal to an existing ID
                    # then that ID will always be this human
                    if id_ == human.ids[key] and id_ in reserved_ids[key] and id_ not in used_ids[key]:
                        poses[key] = pose
                        ids[key] = id_
                        break

                    dist = self.dist(pose, human.pose)
                    if dist < max_dist and id_ not in used_ids[key] and id_ not in reserved_ids[key] and dist < self.MAX_HUMAN_DELTA:
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

        # Now try and match the remaining points and create new humans
        # dict of [key]: dict[id]
        matching_pairs = dict()
        for key in keys:
            # dict of [id]: ((point, point2, id2, key2), dist)
            for key2 in keys:
                if key == key2:
                    continue
                for (p, i) in zip(human_closest_poses[human][key],
                                  human_closest_ids[human][key]):
                    if i in used_ids:
                        continue
                    matching_pairs[key][i] = ((Pose(), Pose(), -1, key2), 
                                              self.MAX_DELTA)
                    for (p2, i2) in zip(human_closest_poses[human][key2],
                                        human_closest_ids[human][key2]):
                        if i2 in used_ids:
                            continue
                        dist = self.dist(p, p2)
                        if dist < min_dist[key][i][1]:
                            matching_pairs[key][i][0] = (p, p2, i2, key2)
                            matching_pairs[key][i][1] = dist
            # Sort the dict such that the closest pairs are together
            matching_pairs[key] = sorted(
                matching_pairs[key], key=lambda x: x[1])

        # Now create a list containing [(key, id, point), ...]
        sets_of_points = []
        # We take the closest pair of points in the matching pairs dict,
        # Check if one of the ids matches an existing id and add the points
        # to that set if it does. If both match an existing id then we
        # discard the points. If none match an existing id then we create a
        # new set.
        for key in keys:
            if len(matching_pairs[key]) == 0:
                del matching_pairs[key]

        while len(matching_pairs) != 0:
            # Will be ((point, point2, id2, key2), dist)
            best_pair = None
            best_id = None
            best_key = None
            for key in matching_pairs:
                for id_ in matching_pairs[key]
                    best_pair = matching_pairs[key][id_]
                    break
                break
            
            for key in matching_pairs:
                for id_ in matching_pairs[id_]:
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
                    if len([t for t in point if t[1] == best_id) != 0:
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

            for key in matching_pairs:
                for id_ in matching_pairs[key]:
                    if len(matching_pairs[key][id_]) == 0:
                        del matching_pairs[key][id_]
                if len(matching_pairs[key]) == 0:
                    del matching_pairs[key]

        # Now we have a set of points which have been grouped together - 
        # these are our new humans.

        for point in set_of_points:
            h = Human(self.id_counter)
            id_counter++
            h.stamp = stamp
            for i in range(len(point)):
                h.ids[point[i][0]] = point[i][1]
                h.poses[point[i][0]] = point[i][2]
                h.pose.position.x += point[i][2].position.x
                h.pose.position.y += point[i][2].position.y
            h.pose.position.x /= len(point)
            h.pose.position.y /= len(point)

            # We don't care what orientation humans are at
            h.pose.orientation.w = 1
            self.humans.append(h)

        # The points not being used are visible only on one camera _and_ 
        # have not yet been matched with an existing human. As such there is
        # no way to know if they are noise or really humans.
        print used_ids
        print unused_ids

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
            msg.human_poses.poses.append(h.pose)
            msg.ids.append(h.id)
        self.pub.publish(msg)

def main():
    try:
        stitcher = HumanStitcher()
        stitcher.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
