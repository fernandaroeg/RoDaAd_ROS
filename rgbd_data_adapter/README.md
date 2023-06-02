# RGB-D Data Adapter

In the data section, there are 3 folders corresponding to the environments in the Robo@Home dataset: alma, pare and rx2. 
Inside each folder there is a set of files with the RGB and Depth images traken from the fullhouse trajectory of the robot in the scenarios.
There is also a log file with the timestamps for each reading:

1.  fullhouse1_rgbd (folder with images)
2.  fullhouse1_rgbd.txt

## Image

In order to compile the images taken from the 4 RGB-D cameras with the CameraInfo parameters the launch file  compile_rgbd_img.launch must
be modified with the name of the environment and the path to the images folder and timestamp file. 

	roslaunch rgbd_data_adapter compile_rgbd_img.launch
	
The output is 4 bag files corresponding to each one of the cameras. 

imagen, rqt bag, tf

## PointCloud
The name of these files along with the environment name have to be configured in the launch file in order to compile the bags with the RGB-D data.

	roslaunch rgbd_data_adapter compile_rgbd_data.launch

Two bag files are created for each one of the 4 RGB-D cameras, a total of 8 files for each scenario. 

## Visualization

The generated bag contains the data provided in the Robo@Home dataset adapted to the ROS message and the required TF data.
TF tree needs baselink. 

robot labels

imagen, rqt bag, tf

For more specific details on the rgbd_data_adapter please refer to section BBB of 
[file](https://github.com/fernandaroeg/ROS_AMCL_Hybrid_Localization/blob/master/TFM_Localizacion_Rodriguez_Fernanda.pdf)

Visualization in rviz of the rgbd data for the environments alma, pare and rx2: 
![]()




