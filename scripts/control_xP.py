#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int32

x_d = 70
Kp = 0.002
Kd = 0.0005
global x0
e0 = 0.0

def callback(data):
	global e0
	x = data.data
	e = x_d - x
	de = e - e0
	u = Kp*e# + Kd*de
	if abs(e) < 5:
		u= 0.0
	if u > 0.05:
		u = 0.05
	elif u < -0.05:
		u = -0.05
	else:
		u = u
	e0 = e
	
	pub.publish(u)
	print 'control x'
	print x,u

def main():
	global pub
	pub = rospy.Publisher('controlX',Float32,queue_size=10)
	rospy.init_node('control_x',anonymous=False)
	#rospy.Subscriber('bebop/odom',Odometry,callback) 
	rospy.Subscriber('/Radio',Int32,callback) 
	rospy.spin()

if __name__ == '__main__':
	main()
