#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32

z_d = 164 #valor del centro de la imagen en y en px
Kp = 0.00222
Kd = 0.00055
global z0
e0 = 0.0

def callback(data):
	global e0
	z = data.data
	e = z_d - z
	de = e - e0
	u = Kp*e + Kd*de
	if abs(e) < 5:
		u= 0.0
	if u > 0.1:
		u = 0.1
	elif u < -0.1:
		u = -0.1
	else:
		u = u
	e0 = e
	
	pub.publish(u)
	print 'control hover'
	print z,u

def main():
	global pub
	pub = rospy.Publisher('controlZ',Float32,queue_size=10)
	rospy.init_node('control_z',anonymous=False)
	rospy.Subscriber('/MY',Int32,callback)
	rospy.spin()

if __name__ == '__main__':
	main()
