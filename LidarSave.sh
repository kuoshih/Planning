#!/bin/sh
# save grid map (Lidar SLAM)
gnome-terminal -- rosrun map_server map_saver -f ~/map

# copy map files to subgoal folder
sleep 5 && gnome-terminal -- cp ~/map.pgm ~/subgoal_map/map.pgm && cp ~/map.yaml ~/subgoal_map/map.yaml
