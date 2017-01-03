#!/usr/bin/env python

""" A node that explores the Neato's odometry """

import rospy
from tf.transformations import euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]


class TestOdometry(object):
    """ A class to represent the odometry node """
    def __init__(self):
        rospy.init_node('test_odometry')

    def run(self):
        """ Our main 5Hz run loop """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = TestOdometry()
    node.run()