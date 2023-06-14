//Ubuntu 20.04LTS, ROS Noetic
//Node that reads data published from node pointcloud_to_laserscan and rewrites data with correct topic name and framed to be accepted by the AMCL node
#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<sensor_msgs/LaserScan.h>
#include<tf2_msgs/TFMessage.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
#include<string>

//Global Variables
sensor_msgs::LaserScan scan_received;
int rgbd_id, TF_yaw;
float TF_x, TF_y, TF_z;
std::string pc_to_laser_topic = "depth";

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    scan_received.header.seq             = msg->header.seq; 
    scan_received.header.stamp         = msg->header.stamp;
    scan_received.angle_min              = msg->angle_min;      
    scan_received.angle_max             = msg->angle_max;      
    scan_received.angle_increment   = msg->angle_increment;
    scan_received.time_increment      = msg->time_increment; 
    scan_received.scan_time               = msg->scan_time;      
    scan_received.range_min              = msg->range_min;      
    scan_received.range_max             = msg->range_max;      
    scan_received.ranges                     = msg->ranges;         
    }
 
//MAIN PROGRAM
int main(int argc, char **argv)
{
	//Initialize
	ros::init(argc, argv, "depth_to_laser_adapter");
	ros::NodeHandle nh;
	
	//Declare publishers and suscriber
	ros::Subscriber sub            = nh.subscribe(pc_to_laser_topic, 1000, scanCallback);
	ros::Publisher  pub_scan  = nh.advertise<sensor_msgs::LaserScan>("laser", 1000); 
	//ros::Publisher  pub_tf         = nh.advertise<tf2_msgs::TFMessage>("tf",1000);
	tf::TransformBroadcaster lasertf_broadcaster;
	
	//Get params from launch file
	nh.getParam("/depth_to_laser_adapter/TF_x",TF_x);
	nh.getParam("/depth_to_laser_adapter/TF_y",TF_y);
	nh.getParam("/depth_to_laser_adapter/TF_z",TF_z);
	nh.getParam("/depth_to_laser_adapter/TF_yaw",TF_yaw);
	nh.getParam("/depth_to_laser_adapter/rgbd_id",rgbd_id);
	nh.getParam("/depth_to_laser_adapter/pc_to_laser_topic",pc_to_laser_topic);
	
	//Node frequency
	ros::Rate r(10);
	
	while(nh.ok()){
		//current_time = ros::Time::now(); //////////////////////////////////////////get current time
		//publish laser msg over ROS
	    sensor_msgs::LaserScan laser_msg;
		laser_msg.header.frame_id = "laser";
		laser_msg.header.seq         = scan_received.header.seq;
		laser_msg.header.stamp     = scan_received.header.stamp;
		laser_msg.angle_min           =scan_received.angle_min;      
		laser_msg.angle_max          =scan_received.angle_max;      
		laser_msg.angle_increment=scan_received.angle_increment;
		laser_msg.time_increment   =scan_received.time_increment; 
		laser_msg.scan_time            =scan_received.scan_time;      
		laser_msg.range_min           =scan_received.range_min;      
		laser_msg.range_max          =scan_received.range_max;      
		laser_msg.ranges                  =scan_received.ranges;    
		//publish the msg
		pub_scan.publish(laser_msg);
		
		////transform theta to quaternion for odom msg, odom_quat
		//geometry_msgs::Quaternion lasertf_quat = tf::createQuaternionMsgFromYaw(TF_yaw);
		////publishh transform over tf
		//geometry_msgs::TransformStamped laser_tf;
		//laser_tf.header.seq                = scan_received.header.seq;
		//laser_tf.header.stamp            = scan_received.header.stamp;
		//laser_tf.header.frame_id       = "laser";
		//laser_tf.child_frame_id          = "base_link";
		//laser_tf.transform.translation.x = TF_x;
		//laser_tf.transform.translation.y = TF_y;
		//laser_tf.transform.translation.z = TF_z;
		//laser_tf.transform.rotation         = lasertf_quat;
		////send transform
		//lasertf_broadcaster.sendTransform(laser_tf);	
		
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(TF_x, TF_y, TF_z) );
		transform.setRotation( tf::Quaternion(TF_yaw, 0, 0) );
		br.sendTransform(tf::StampedTransform(transform, scan_received.header.stamp, "base_link", "laser"));

		ros::spinOnce();  //check for incoming messages
		r.sleep();
	}
	//ros::spin();
	//return 0;
}

