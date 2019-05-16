'''
listener_velocity subscribes to the data published on the 'center_points' node, using ROS.
The incoming data of the locations of the center points is used to calculate the velocity.
The velocity is published along the 'velocity' node
Velocity units: [cm/s]
'''

#!/usr/bin/env python
import rospy
import json
import math
from std_msgs.msg import String

last_frame = {}
pub = rospy.Publisher('velocity', String, queue_size=10)
velocity = {'dark pink': 0.0, 'green': 0.0, 'blue': 0.0, 'yellow': 0.0, 'orange': 0.0, 'brown': 0.0, 'light pink': 0.0}


def dist(a, b):
    return round(math.sqrt(math.pow(a, 2) + math.pow(b, 2)), 3)


def minus(a, b):
    return b[0] - a[0], b[1] - a[1]


def callback(data):
    global last_frame, pub, velocity

    # changing the data to dictionary from string
    input = data.data
    current_frame = json.loads(input)

    time = 1  # temp
    # diff_frame = {}
    if len(last_frame) > 0:  # if its not the first frame
        for key, position in current_frame.iteritems():  # looks at each of the items in the dictionary

            # calculates difference between the "key" colored circle in the current and previous frames
            diff_frame = minus(last_frame[key], current_frame[key])

            x = float(diff_frame[0])
            y = float(diff_frame[1])
            dist_moved = dist(x, y)
            velocity[key] = dist_moved / time

    rospy.loginfo(rospy.get_caller_id() + ' - ' + str(velocity))
    last_frame = current_frame
    pub.publish(str(velocity))


def main():

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
    print('listener_velocity - waiting for messages...')
    main()

