#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist

def main():
	rospy.init_node('Ori_cam', anonymous=False)
	cam_pos = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10)
	vel_msg = Twist()
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.z = 0
	vel_msg.angular.y = -60
	while not rospy.is_shutdown():
		cam_pos.publish(vel_msg)
if __name__=='__main__':
	main()
