#include<ros/ros.h>
#include<std_msgs/Int32.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
#include<tf/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include<message_filters/subscriber.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/TransformStamped.h>
//Ubuntu 20.04, ROS Noetic, NODE that listens to pc,tf topics, waits for the tf to be available and then publishes tf_filtered and pc_filtered
sensor_msgs::PointCloud2 pc_rcvd;

//void pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
void pcCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	//pc_rcvd.header            = msg->header; //si esto funciona hacer lo mismo para toda la estructura de pc
	//pc_rcvd.height             = msg->height;           
	//pc_rcvd.width              = msg->width;             
	//pc_rcvd.fields              = msg->fields;             
	//pc_rcvd.is_bigendian = msg->is_bigendian;
	//pc_rcvd.point_step     = msg->point_step;    
	//pc_rcvd.row_step       = msg->row_step;    
	//pc_rcvd.data                = msg->data;               
	//pc_rcvd.is_dense        = msg->is_dense;      
	ROS_INFO("degugg: callback funciona");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_filter_pc");
    ros::NodeHandle nh;
	
	//delcarar publisher y boadcaster
	ros::Subscriber sub                     = nh.subscribe("laser", 1000, pcCallback);
	ros::Publisher  pc_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("pc1_filtered", 1000);
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	tf::TransformBroadcaster br;
   
    ros::Rate rate(10.0);	 
    while (nh.ok())
	{   
		geometry_msgs::TransformStamped transformStamped_listener;
		ROS_INFO("degugg: al menos entra al while");
		try
		{
			//ros::Time now = ros::Time::now();
			//listener.waitForTransform("pc1", "base_link", now, ros::Duration(2.0));
			//listener.lookupTransform( "pc1", "base_link", now, transformStamped_listener);
			ROS_INFO("degugg: listener succesfull");
			transformStamped_listener = tfBuffer.lookupTransform("laser", "base_link", ros::Time::now(), ros::Duration(2.0));
		}
		
		catch (tf2::TransformException &ex)
		{
			ROS_INFO("catch found");
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		
		ROS_INFO("degugg: despues de try y catch");
		//publicar pc1 filtrado con tf nuevo
		//sensor_msgs::PointCloud2 pc_new;	   
		//pc_new = pc_rcvd;
		//pc_new.header.stamp = transformStamped_listener.header.stamp;
		//pc_new.header.frame_id = "pc1_filtered";
		//pc_filtered_pub.publish(pc_new);
		//
		//publishh transform over tf
		geometry_msgs::TransformStamped pc_trans;
		pc_trans.header.stamp            = transformStamped_listener.header.stamp;
		pc_trans.header.frame_id       = "pc1_filtered";
		pc_trans.child_frame_id          = "base_link";
		pc_trans.transform.translation    = transformStamped_listener.transform.translation;
		pc_trans.transform.rotation         = transformStamped_listener.transform.rotation;
		//send transform
		br.sendTransform(pc_trans);
		
		ros::spinOnce();  //check for incoming messages
		rate.sleep();			 
	}
	//return 0;
};



	
