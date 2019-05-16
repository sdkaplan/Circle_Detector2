'''
listener_ subscribes to the data published on the 'center_points' node, using ROS.
This is a template for future listeners/subscribers
units: [cm]
'''

#!/usr/bin/env python
import rospy
import json
import math
from std_msgs.msg import String


def callback(data):
    input = data.data
    rospy.loginfo(rospy.get_caller_id() + '- %s', input)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('center_points', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    print('listener - waiting for messages...')
    listener()

