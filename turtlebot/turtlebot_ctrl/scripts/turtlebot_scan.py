#!/usr/bin/env python
import rospy
import numpy as np
from turtlebot_ctrl.msg import TurtleBotScan
from sensor_msgs.msg import LaserScan

class TurtlebotScanner:
	def __init__(self):
		rospy.init_node("turtlebot_scan_handler", anonymous=True)

		self.pub = rospy.Publisher("turtlebot_scan", TurtleBotScan, queue_size=10)

		rospy.Subscriber("/scan", LaserScan, self.scanCallBack)

	def scanCallBack(self, scan_msg):
		n = len(scan_msg.ranges)
		print(n)
		# Generating Gaussian noise with mean=0, std.dev=0.5 and len=n
		deviation = (rospy.get_param("/scan_noise"))
		noise = np.random.normal(0,deviation,n)

		msg = TurtleBotScan()
		counter = 0
		for i in range(n):
    			if i == counter:
				msg.ranges.append(scan_msg.ranges[i]+noise[i])
				counter += 12							

                self.pub.publish(msg)

	def run(self):
		rospy.spin()

if __name__ == "__main__":
	try:
		turtlebot_scanner = TurtlebotScanner()
		turtlebot_scanner.run()
	except rospy.ROSInterruptException:
		pass



