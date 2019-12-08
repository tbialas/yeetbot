# Time and relative frame of the pose array
std_msgs/Header header

# Array of poses of detected humans
# Not sorted.
geometry_msgs/PoseArray human_poses

# Matching array of unique IDs of human_poses (e.g. ID 0 is a unique person)
# Not sorted.
int64[] ids

# e.g.:
# human_poses: [A, B, C, D, E, ...]
# ids:         [0, 5, 2, 6, 1, ...]
# A has id 0, B has id 5, C has id 2, D has id 6, E has id 1, ...
