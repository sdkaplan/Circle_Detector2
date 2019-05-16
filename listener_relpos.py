'''
listener_relpos.py subscribes to the data published on the 'center_points' node, using ROS.
The incoming data of the locations of the center points is used to calculate relative distance to each circle.
The this will eventually be published along another node for robots to subscribe to
units: [cm]
'''

#!/usr/bin/env python
import rospy
import json
import math
from std_msgs.msg import String


def dist(a, b):
    return round(math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2)), 3)


# given an array of positions at a given moment in time, return an array of the relative distances between them
def getRelPos(positions):
    # positions is a list of 12 tuples that are the positions at the given time

    rel_distance = [[] for i in positions]

    count = 0
    for pos in positions:

        # The coordinates of the main circle center
        x = float(pos[0])
        y = float(pos[1])

        for compare in positions:
            # the position of the coordinate we are comparing too
            x2 = float(compare[0])
            y2 = float(compare[1])

            # The diagonal distance between the two coordinates
            # rounded to two decimal places
            distance = dist((x, y), (x2, y2))

            rel_distance[count].append(distance)

        count = count + 1
    # print(relDistance)
    return rel_distance


def callback(data):

    input = data.data
    current_frame = json.loads(input)

    rel_pos = getRelPos(current_frame)

    rospy.loginfo(rospy.get_caller_id() + ' - ' + str(rel_pos))


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
    print('listener_relpos - waiting for messages...')
    listener()

