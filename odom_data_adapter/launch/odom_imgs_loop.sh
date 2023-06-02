#!/bin/bash
#roscore must be launched previously to this script
echo "launch node 2 times"
source /opt/ros/noetic/setup.bash
source /home/fer/catkin_ws/devel/setup.bash
scenes=("alma" "pare" "rx2")
for i in {0..2}
do
	for j in {1..5}
	do
		roslaunch odom_data_adapter odom_imgs.launch scenario:=${scenes[i]} file_num:=$j
	done 
done

	
