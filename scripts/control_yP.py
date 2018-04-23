#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

y_d = 120 #valor del centro de la imagen en x en px
Kp = 0.00037
Kd = 0.00009
global y0
e0 = 0.0

def callback(data):
	global e0
	y = data.data
	e = y_d - y
	de = e - e0
	u = Kp*e + Kd*de
	if abs(e) < 32:
		u=0.0	
	if u > 0.1:
		u = 0.1
	elif u < -0.1:
		u = -0.1
	else:
		u = u
	e0 = e
	
	pub.publish(u)
	print 'control y'
	print y,u, e

def main():
	global pub
	pub = rospy.Publisher('controlY',Float32,queue_size=10)
	rospy.init_node('control_y',anonymous=False)
	#rospy.Subscriber('bebop/odom',Odometry,callback) #y viene de la odometria
	rospy.Subscriber('/c_x',Int32,callback)
	#rospy.Subscriber('centroide',Int16MultiArray,reference) #y viene del centroide
	rospy.spin()

if __name__ == '__main__':
	main()
