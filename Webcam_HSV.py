'''
Webcam_HSV_ reads the image from the camera then looks for circles on every frame. 
To find circles it applies HSV color filters to each frame and looks for circular contours
with each filter applied. 
Using ROS, The center point data is published on 3 nodes: 
'center_points'                 The data is the center points at every frame
'center_points_formatted'       the data is formatted to CSV data (color,(x,y),color1,(x1,y1) etc.)
'center_points_less_frames'     The data is the center points at every 5th frame
the frame rate for 'center_points_less_frames' can be set on line 44: frame_num
UNITS: cm -> is converted from pixels, must to be recalibrated with new camera or monitor
'''

'''
HOW TO SAVE THE LOCATION DATA TO A SPECIFIC FILE TYPE:
run this betwel line in a terminal to publish the data to a .txt file
rostopic echo /center_points >>file.txt
the data can also be saved to a .bag file of rosbag. This would not be a .txt file but rather a file of data
rosbag is not implemented
'''

# import the necessary packages
import math
import json
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
import datetime


def format_for_csv(data):
	# takes data in the form of a dictionary and formats it for .csv
	# 'blue',(x,y),'green',(x1,y1) etc

	data_string = ''
	for color, cp in data.iteritems():
		data_string = data_string + color + ',' + str(cp) + ','

	return data_string

def main():
    #sets video to save as current date and time
    now = datetime.datetime.now()
    video_name = now.strftime("%Y-%m-%d %H:%M.avi")


    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_name,fourcc, 20.0, (640,480))

    frame_num = 5 # looks at every 5th frame

    # 120 pixels = 88.85 mm
    # this was measured by printing the radius of the circle and measuring it using a caliper
    conversion = 120 / 8.885  # PIXELS / CM    eg [pixels /  cm]

    # define the lower and upper boundaries of the colors in the HSV color space
    lower = {'dark pink': (138, 67, 38),
             'green': (65, 32, 67),
             'blue': (99, 50, 46),
               'yellow': (21, 75, 45),
             'orange': (11, 134, 115),
             'brown': (0, 41, 47), #not calibrated
             'light pink': (145, 115, 130)} #not calibrated
	     # three more to be added

    upper = {'dark pink': (191, 216, 219),
             'green': (92, 85, 255),
             'blue': (140, 255, 198),
             'yellow': (38, 136, 196),
             'orange': (38, 255, 196),
             'brown': (18, 190, 106), #not calibrated 
             'light pink': (160, 125, 150)} #not calibrated
	     # three more to be added

    # define standard colors for printed circle around the object
    colors = {'dark pink': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0), 'yellow': (0, 255, 217),
              'orange': (0, 140, 255), 'brown': (101, 67, 33), 'light pink': (255, 192, 203)} # three more to be added

    # Array of all the center points for given frame
    centerPoints_current = {'dark pink': (0, 0), 'green': (0, 0), 'blue': (0, 0), 'yellow': (0, 0),
                            'orange': (0, 0), 'brown': (0, 0), 'light pink': (0, 0)}
			    # three more to be added

    center_points_less_frames = {'dark pink': (0, 0), 'green': (0, 0), 'blue': (0, 0), 'yellow': (0, 0),
                                 'orange': (0, 0), 'brown': (0, 0), 'light pink': (0, 0)}

    # initialize ROS publisher nodes
    pub1 = rospy.Publisher('center_points', String, queue_size=10)
    pub2 = rospy.Publisher('center_points_formatted', String, queue_size=10)
    pub3 = rospy.Publisher('center_points_less_frames', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # grab the reference to the camera
    camera = cv2.VideoCapture(0)  # IMPORTANT: 0 for default webcam, 1 for usb webcam
    print("video received")

    frame_count = 0
    while True:
        # cv2.waitKey(50) # this is to take less frames per second, 5000 is 1 frame every 5 sec

        # grab the current frame
        (grabbed, frame) = camera.read()
        frame_count = frame_count + 1

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)  # blur frame
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)  # convert frame to HSV

        # for each color in dictionary check object in frame
        for key, _ in upper.items():

            # construct a mask for the color from dictionary, then perform
            # a series of dilations and erosions to remove any small blobs left in the mask
            kernel = np.ones((9, 9), np.uint8)
            mask = cv2.inRange(hsv, lower[key], upper[key])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # find contours in the mask and initialize the current (x, y) center of the ball
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                xc = round(int(M["m10"] / M["m00"]) / conversion, 3)  # ROUNDING
                yc = round(int(M["m01"] / M["m00"]) / conversion, 3)  # ROUNDING
                center = (xc, yc)
                # print(radius)

                # if the radius is between 40 and 60 pixels (current size about 50)
                # This statement removes noise from larger blobs that may appear
                # It may not be necessary when only the table appears in the frame
                if 45 < radius < 65:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 2)
                    cv2.putText(frame, key, (int(x - radius), int(y - radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                colors[key], 2)
                else:
                    center = (0, 0)  # If it is finding a larger circle ignore it by setting center to (0,0)
            else:
                center = (0, 0)  # If it doesn't see a color set the center (0, 0) so we know its not right

            # STORING THE CENTER POINTS:
            centerPoints_current[key] = center
	    
	    #writes frome to video saver
            out.write(frame)

            # show the frame to our screen
            cv2.imshow("Frame", frame)
            # cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break

        # records every frame_num frames    
        if (frame_count % frame_num == 0):

            center_points_less_frames = centerPoints_current

        centerPoints_csv = format_for_csv(centerPoints_current)
	#rospy.loginfo(centerPoints_csv)
        #rospy.loginfo(rospy.get_caller_id() + ' - ' + str(centerPoints_current))

        # Publish current frame center points to ROS node
        pub1.publish(json.dumps(centerPoints_current))
        pub2.publish(centerPoints_csv)
        # pub3.publish(str(center_points_less_frames))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # cleanup the camera, stop saving video and close any open windows
    camera.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


