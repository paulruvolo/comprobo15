#!/usr/bin/env python

""" A very simple node for testing out the Python debugger """

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rospy

class SimpleNode(object):
    """ A simple node that listens on the chatter topic and echos it to the console """
    def __init__(self):
        rospy.init_node('simple_node')
        rospy.Subscriber('/chatter', String, self.process_chatter)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)

    def process_chatter(self, msg):
        print msg

    def process_scan(self, msg):
        print msg.header.seq

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = SimpleNode()
    node.run()