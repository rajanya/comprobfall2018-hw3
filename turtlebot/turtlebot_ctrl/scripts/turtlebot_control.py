#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from kobuki_msgs.msg import BumperEvent
from turtlebot_ctrl.srv import TurtleBotControl, TurtleBotControlResponse
from turtlebot_ctrl.msg import TurtleBotScan

class TurtlebotControlServer:
	def __init__(self):
		rospy.init_node("turtlebot_control_server")

		self.service = rospy.Service("turtlebot_control",TurtleBotControl,self.call_service)

		self.cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size=5)

		rospy.wait_for_service("/gazebo/get_model_state")
		self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)

		rospy.wait_for_service("/gazebo/set_model_state")
		self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

		self.bumper_sub =  rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.processBump)

		self.angular_noise = rospy.get_param("/angular_noise")
		self.linear_noise = rospy.get_param("/linear_noise")

		self.step_size = 10

	def processBump(self,data):
		# @aravind: Has to be a better way of handling this, but will leave this here.
		rospy.logerr("Collision detected!")
		rospy.signal_shutdown("Collision detected!")

	def get_current_pose(self):
		# current_odom = rospy.wait_for_message("/odom",Odometry)
		current_model_state = self.get_model_state(model_name="mobile_base")
		return current_model_state.pose

	def set_steering_angle(self,steering_angle):
		model_state_resp = self.get_model_state(model_name="mobile_base")
		model_state = SetModelState()
		model_state.model_name = "mobile_base"
		model_state.pose = model_state_resp.pose
		model_state.twist = Twist()
		model_state.reference_frame = "world"
		quat = tf.transformations.quaternion_from_euler(0,0,steering_angle)
		model_state.pose.orientation.x = quat[0]
		model_state.pose.orientation.y = quat[1]
		model_state.pose.orientation.z = quat[2]
		model_state.pose.orientation.w = quat[3]

		self.set_model_state(model_state=model_state)

	def set_position(self,position):
		model_state_resp = self.get_model_state(model_name="mobile_base")
		model_state = SetModelState()
		model_state.model_name = "mobile_base"
		model_state.twist = Twist()
		model_state.reference_frame = "world"
		model_state.pose = model_state_resp.pose
		model_state.pose.position.x = position[0]
		model_state.pose.position.y = position[1]

		self.set_model_state(model_state=model_state)

	def get_current_angle(self,current_pose):
		current_quat = current_pose.orientation
		current_angle = tf.transformations.euler_from_quaternion([current_quat.x, current_quat.y, current_quat.z, current_quat.w])[2]
		return current_angle

	def get_current_position(self,current_pose):
		return np.array([current_pose.position.x,current_pose.position.y])

	def call_service(self,req):
		"""
		Move the turtlebot to the requested pose. 
		if collision: return turtlebot to original pose, return
		"""
		current_pose = self.get_current_pose()
		target_pose = np.array([current_pose.position.x + np.clip(req.point.x,-1.0,1.0),
								current_pose.position.y + np.clip(req.point.y,-1.0,1.0)])

		steering_angle = np.arctan2((target_pose[1] - current_pose.position.y),(target_pose[0] - current_pose.position.x)) + np.random.normal(0,self.angular_noise)
		self.set_steering_angle(steering_angle)

		target_pose += np.random.normal(0,self.linear_noise,2)

		current_position = self.get_current_position(current_pose)

		waypoints_x = np.linspace(current_position[0],target_pose[0],self.step_size)
		waypoints_y = np.linspace(current_position[1],target_pose[1],self.step_size)
		waypoints = np.vstack([waypoints_x,waypoints_y]).T

		for i in range(waypoints.shape[0]):
			self.set_position(waypoints[i])
			rospy.sleep(rospy.Duration(0.1))

		scan_data = rospy.wait_for_message("/turtlebot_scan",TurtleBotScan)

		model_state = self.get_model_state(model_name="mobile_base")
		ground_truth = ModelState()
		if req.return_ground_truth.data:
			ground_truth.pose = model_state.pose
			ground_truth.twist = model_state.twist
		ground_truth.model_name = "mobile_base"

		return TurtleBotControlResponse(ground_truth=ground_truth,scan_data=scan_data)

	def run(self):
		rospy.spin()

if __name__ == "__main__":
	try:
		turtlebot_control_server = TurtlebotControlServer()
		turtlebot_control_server.run()
	except rospy.ROSInterruptException:
		pass



