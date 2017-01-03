#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


from tf.transformations import euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]


class OdometryProportionalControl(object):
	def __init__(self):
		rospy.init_node('odometry_proportional_control')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/odom', Odometry, self.process_odom)
		self.x0 = None
		self.target_x = None
		self.current_x = None
		self.k = -0.1

	def process_odom(self, msg):
		if self.x0 == None:
			self.x0 = msg.pose.pose.position.x
			self.target_x = self.x0 + 1.0
		self.current_x = msg.pose.pose.position.x
		print convert_pose_to_xy_and_theta(msg.pose.pose)

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			#print self.current_x, self.target_x

			if self.current_x != None and self.target_x != None:
				error = self.current_x - self.target_x
				msg = Twist(linear=Vector3(x=error*self.k))
				#self.pub.publish(msg)
			r.sleep()

if __name__ == '__main__':
	node = OdometryProportionalControl()
	node.run()