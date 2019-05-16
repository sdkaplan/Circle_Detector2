'''
listener_published_velocity subscribes to the data published on the 'velocity' node, using ROS.
This is later what the robots will be subsribing to.
Velocity units: [cm/s]
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
    print("listener node initiated")
    rospy.Subscriber('velocity', String, callback)
    print('has been subscribed')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    print('listener_published_velocity - waiting for messages...')
    listener()

