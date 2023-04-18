# Map Files

There are four folders corresponding to each one of the environments in the Robo@Home dataset with the map files for each: alma, pare, rx2 and sarmis. 
Inside each folder there are 5 files to launch the files either using mrpt_map node or the map_server from the ROS's navigation stack.

**map_server**
1. pgm file: encoded data of the occupancy grid map as a grayscale image
2. yaml file: metadata and pgm image name 

**mrpt_map**

3. map.simplemap file 
4. environment_name.simpe	file is a copy of the previous file 
5. map.ini map metadata file 

## ROS's Navigation Stack map_server 

 This node reads a map saved in the disk and provides it to any other node that requests it via a ROS service.
	
	rosrun  map_server  map_server  mapfile.yaml 

Apart from this service there are two latched topics where you can get a message with the map. The topics where the node writes the map data are: 

	map_metadata (nav_msgs/MapMetaData) 
	map (nav_msgs/OccupancyGrid) 

## MRPT_map node use and installation guide 

The instructions to install and run the mrpt tools based on the   [MRPT Navigation Installing Tutorial](http://wiki.ros.org/mrpt_navigation/Tutorials/Installing#Get_from_apt_packages).

Install the ROS mrpt navigation metapackage:

	sudo apt-get install ros-noetic-mrpt-navigation

Install latests MRPT version from PPA repositories:

	sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable	
	sudo apt install libmrpt-dev mrpt-apps

Go into the source folder of the catkin workspace and clone the specific mrpt branch for ROS1: 

	cd catkin_ws/_src
	git clone -b ros1 git@github.com:mrpt-ros-pkg/mrpt\_navigation.git

Go back to the catkin workspace and build the package 

	cd ..
	catkin_make

Test the build was done succesfully and source:

	catkin_make tests
	source devel/setup.bash

Up to this point the mrpt_map node is ready to be used. In order to launch the maps used in this project with this node follow these steps:

Navigate to the folder where the simplemap files are located. 
 
Select the desired environment map and run inside the folder: 

	rosrun mrpt_map map_server_node map.simplemap
  
Visualize in rviz the map selecting /map as fixed frame


