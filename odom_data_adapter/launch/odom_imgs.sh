#!/bin/bash
echo "Taking rviz screenshot and killing nodes"
sleep 255
my_date=$(date +"%Y-%m-%d_%H:%M:%S")
import -window root odom_path_${my_date}.png
#import -window root odom$2_path_$1_${my_date}.png
rosnode kill -a 
#convert alma_test.png -crop 1120x750+850+130 alma_test_crop.png
