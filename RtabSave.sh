#!/bin/sh

# save Rtabmap
cp ~/.ros/rtabmap.db ~/rtabmap.db

# save .ot map
rosrun octomap_server octomap_saver -f temp_otfile.ot octomap_full:=/rtabmap/octomap_full

