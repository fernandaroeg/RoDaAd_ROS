#! /usr/bin/env python
#Ubuntu 16.04, ROS Kinetic, python 2.7
import rospy
import rosbag
from sensor_msgs.msg   import LaserScan
from tf2_msgs.msg      import TFMessage 
from geometry_msgs.msg import TransformStamped
import tf
from tf.transformations import euler_from_quaternion
#Node that reads data published from node pointcloud_to_laserscan and rewrites data with correct topic name and framed to be accepted by the AMCL node

#Get Launch File Parameters
TF_x      = rospy.get_param('/depth_to_laser_adapter/TF_x')
TF_y      = rospy.get_param('/depth_to_laser_adapter/TF_y')
TF_z      = rospy.get_param('/depth_to_laser_adapter/TF_z')
TF_yaw    = rospy.get_param('/depth_to_laser_adapter/TF_yaw')
rgbd_id = rospy.get_param('/depth_to_laser_adapter/rgbd_id')
pc_to_laser_topic = rospy.get_param('/depth_to_laser_adapter/pc_to_laser_topic')

pub_scan = rospy.Publisher('depth'+str(rgbd_id), LaserScan, queue_size = 11)
pub_tf   = rospy.Publisher("tf", TFMessage, queue_size=11)   
scan_received = LaserScan()    

def create_TFmsg(x, y, z, yaw, frame, child_frame, seq, t):
    trans = TransformStamped()
    trans.header.seq = seq
    trans.header.stamp = t
    trans.header.frame_id = frame
    trans.child_frame_id = child_frame
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    q = tf.transformations.quaternion_from_euler(0,0,yaw)
    trans.transform.rotation.x = q[0]
    trans.transform.rotation.y = q[1]
    trans.transform.rotation.z = q[2]
    trans.transform.rotation.w = q[3]
    msg = TFMessage()
    msg.transforms.append(trans)
    return msg

def callback(msg):
    current_time = rospy.Time(0)
    scan_received.header.seq         = msg.header.seq 
    scan_received.header.stamp     = current_time
    scan_received.header.frame_id = 'depth'+str(rgbd_id)
    scan_received.angle_min           = msg.angle_min      
    scan_received.angle_max          = msg.angle_max      
    scan_received.angle_increment= msg.angle_increment
    scan_received.time_increment   = msg.time_increment 
    scan_received.scan_time            = msg.scan_time      
    scan_received.range_min           = msg.range_min      
    scan_received.range_max          = msg.range_max      
    scan_received.ranges                  = msg.ranges         
    
    new_tf_msg   = create_TFmsg(TF_x, TF_y, TF_z, TF_yaw, 'base_link','depth'+str(rgbd_id), msg.header.seq+1, current_time)
    pub_scan.publish(scan_received)
    pub_tf.publish(new_tf_msg)

def listener():
    rospy.init_node('depth_to_laser_adapter', anonymous=True)
    sub = rospy.Subscriber(pc_to_laser_topic, LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
 