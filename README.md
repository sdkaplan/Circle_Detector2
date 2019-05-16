# Circle_Detector2
Cylinder Vision Tracking System
by Sydney Kaplan, skaplan9@hawk.iit.edu
Spring 2019

This document provides a step by instructions for the vision tracking system that uses ROS

Within Files navigate to:
Home --> soft_robotics

Open a new terminal
	Type: roscore
to begin running ROS

Open a new terminal or new tab within the same terminal

	Type: cd soft_robotics/

This will navigate the terminal to the soft_robotics folder

	Type:  python Webcam_HSV.py

Wait a few seconds. The terminal will read "video received" and shortly after the video feed will appear.
This will run the python file Webcam_HSV.py, which can be seen in the soft_robotics folder. 

	Webcam_HSV_ reads the image from the camera then looks for circles on every frame. 

	To find circles it applies HSV color filters to each frame and looks for circular contours
	with each filter applied. The code currently tracks: dark pink, green, blue, yellow, orange, brown, and light pink. 
	Future colors could include red, and darker or lighter versions of the colors already in use.

	Using ROS, The center point data is published on 3 nodes: 
	'center_points'                 The data is the center points at every frame
	'center_points_formatted'       the data is formatted to CSV data (color,(x,y),color1,(x1,y1) etc.)
	'center_points_less_frames'     The data is the center points at every 5th frame
		Note: The frame rate for 'center_points_less_frames' can be set on line 45: frame_num
	
UNITS: cm -> is converted from pixels, must to be recalibrated with new camera or monitor. As of May 16th, 2019 the pixel to cm has been calibrated.

In a new terminal window run one (or multiple) of the listeners:
listener.py				replays the center point data published over ROS	
listener_relpos.py			from the center points data publishes the relative position of the circles with each other *Work in progress
listener_velocity.py			from the center points data publishes the velocity of each cirlce
listener_published_velocity.py		listens to the published velocity of each circle




HOW TO SAVE THE LOCATION DATA TO A SPECIFIC FILE TYPE:

To publish the data to a .txt file:
	Type: rostopic echo /center_points >>file.txt


WHAT IF THE CIRLCES ARE NOT BEING TRACKED?

-Remove all other shapes from the screen, this includes hands. When each color filter is applied, the only shape should be the circle

-The software is filtering out large and small circles. Open Webcam_HSV.py and navigate to line 126, remove the comment to see the size of the radius' being found by teh software. Adjust the if statement on line 131 accordingly

-adjust the HSV color space values being used. In a new terminal run color_chocie_HSV.py by typing the following line in a new terminal: 
        python color_choice_HSV.py --filter HSV --webcam
using the trackbar pop-up, adjust the HSV ranges until only one color is being seen with each filter. Update the "lower" and upper" values on lines 52-68 of Webcam_HSV.py

-Make sure there is not a blue and purple circle on the screen. In the HSV volor space blue and purple cannot be distinguished from eachother. 
