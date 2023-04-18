# Map Files

There are four folders with the map files for each one of the environments: alma, pare, rx2 and sarmis. 
The pgm and yaml files are required to publish the map in the /map topic using the map_server node from ROS's navigation stack.
The ini and simplemap files are required to run the map using the mrpt_map node. 

## ROS's Navigation Stack map_server 


## MRPT_map node use and installation guide 
The instructions to install and run the mrpt tools based on the   [MRPT Navigation Installing Tutorial]{http://wiki.ros.org/mrpt_navigation/Tutorials/Installing#Get_from_apt_packages}.
1. Install the ROS mrpt navigation metapackage:

  sudo apt-get install ros-noetic-mrpt-navigation

2. Install latests MRPT version from PPA repositories:

  sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable
  sudo apt install libmrpt-dev mrpt-apps

3. Go into the source folder of the catkin workspace and clone the specific mrpt branch for ROS1: 

  cd catkin_ws/_src
  git clone -b ros1 git@github.com:mrpt-ros-pkg/mrpt\_navigation.git

4. Go back to the catkin workspace and build the package 

  cd ..
  catkin_make

5. Test the build was done succesfully and source:

  catkin_make tests
  source devel/setup.bash

Up to this point the mrpt_map node is ready to be used. In order to launch the maps used in this project with this node follow these steps:

1. Navigate to the folder where the simplemap files are located. For this [project]{https://github.com/fernandaroeg/RoDaAd_ROS/tree/main/maps}. 
2. Select the desired environment map and run inside the folder: 

  rosrun mrpt_map map_server_node map.simplemap
  
3. Visualize in rviz the map selecting /map as fixed frame


