#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry  # nav_msgs is the package name
from location_monitor.msg import LandmarkDistance

class LandmarkMonitor(object):
	def __init__(self, pub, landmarks):
		self._pub = pub
		self._landmarks = landmarks

	def distance(self, x1, y1, x2, y2):
		xd = x1 - x2
		yd = y1 - y2
		return math.sqrt(xd*xd+yd*yd)

	def callback(self, msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		closest_name = None
		closest_distance = None
		for l_name, l_x, l_y in self._landmarks:
			dist = self.distance(x, y, l_x, l_y)
			if closest_distance is None or closest_distance > dist:
				closest_name = l_name
				closest_distance = dist
		ld = LandmarkDistance()
		ld.name = closest_name
		ld.distance = closest_distance
		self._pub.publish(ld)

		if closest_distance <= 0.5:
				rospy.loginfo("I am near the {}".format(closest_name))
	

def main():
	rospy.init_node('location_monitor')

	landmarks = []
	landmarks.append(("Cube",0.02,-0.28))
	landmarks.append(("Dumpster",0.05,-1.77))
	landmarks.append(("Cylinder",-1.98,-1.89))
	landmarks.append(("Barrier",-2.59,-0.83))
	landmarks.append(("Bookshelf",-0.66,1.68))	
	
	pub = rospy.Publisher('closest_landmark', LandmarkDistance, queue_size = 10)
	monitor = LandmarkMonitor(pub, landmarks)	

	rospy.Subscriber("/odom", Odometry, monitor.callback)	
	rospy.spin()

if __name__ == "__main__":
	main()
