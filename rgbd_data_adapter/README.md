# RGB-D Data Adapter

In the data section, there are 3 folders corresponding to the environments in the Robo@Home dataset: alma, pare and rx2. 
Inside each folder there is a set of files with the RGB and Depth images traken from the fullhouse trajectory of the robot in the scenarios.
There is also a log file with the timestamps for each reading:

1.  fullhouse1_rgbd (folder with images)
2.  fullhouse1_rgbd.txt

## Image
The images from the 4 RGB-D cameras can be compiled by running the launch file "compile_rgbd_img.launch".
In this file 4  parameters have to be set:
1. environment: alma, pare, rx2
2. rgbd_raw_path: path to the folder with the png images
3. tstamp_file: path to the file with the timestamps
4. img_type: type of image to be compiled, "intensity" for colored images or "depth" for grayscale depth images.

The CameraInfo is specificied in the source code "rgbd_data_adapter_img.py" with values taken from the dataset
Robo@Home.  The launch file can be run with the command: 

	roslaunch rgbd_data_adapter compile_rgbd_img.launch
	
The output is 4 bag files corresponding to each one of the cameras.
In the following image it can be seen in the left, the content in each bag file with the topics tf, cameraInfo and image, in the 
middle the image visualization using the rqt_image_view tool, and in the right the tf tree corresponding to this data. 
![rbgd_image](rbgd_image.png)

## PointCloud
The name of these files along with the environment name have to be configured in the launch file in order to compile the bags with the RGB-D data.

	roslaunch rgbd_data_adapter compile_rgbd_data.launch

Two bag files are created for each one of the 4 RGB-D cameras, a total of 8 files for each scenario. 

## Visualization

The generated bag contains the data provided in the Robo@Home dataset adapted to the ROS message and the required TF data.
TF tree needs baselink.  rqt_image_view

robot labels

imagen, rqt bag, tf

For more specific details on the rgbd_data_adapter please refer to section BBB of 
[file](https://github.com/fernandaroeg/ROS_AMCL_Hybrid_Localization/blob/master/TFM_Localizacion_Rodriguez_Fernanda.pdf)

Visualization in rviz of the rgbd data for the environments alma, pare and rx2: 
![]()

## Synchronization Problems


## Converting PointCloud data to laser

