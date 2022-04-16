#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time


class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

        def image_callback(self, msg):
                global thistime,lasttime

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                stop = ardetect(image)
                thistime = time.time()
                if stop == 1 and thistime-lasttime>5:
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    time.sleep(10)
                    lasttime = time.time()

                



                # lower_yellow = numpy.array([ 10, 10, 10])
                # upper_yellow = numpy.array([255, 255, 250])

                lower_yellow = numpy.array([ 26, 43, 46])
                upper_yellow = numpy.array([ 34, 255, 255])

                lower_white = numpy.array([0, 0, 221])
                upper_white = numpy.array([180, 30, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                # print(mask1.shape)

                mask3 = mask1 + mask2

                H = numpy.array([[-1.01622597e+00, -1.40189411e+00,  3.27899940e+02],[ 1.16602867e-02, -4.65824866e+00,  6.62601183e+02],[ 1.78918665e-05, -8.32309418e-03,  1.00000000e+00]]) #
                # H = numpy.array([[-3.86344764e-01, -1.38196714e+00,  2.27734675e+02],[-7.45750284e-02, -4.47924693e+00,  6.48460362e+02],[-1.67067788e-04, -8.06934419e-03,  1.00000000e+00]]) #

                # H = numpy.array([[-0.434, -1.33,  229],[0.0, -2.88,  462],[-0.0, -0.00833,  1.00000000e+00]]) #
                # plane_view = cv2.warpPerspective(mask3, H, (320,240))
                plane_view = cv2.warpPerspective(mask3, H, (340,480))
                h, w, d = image.shape
                # print(image.shape)(240, 320, 3)

                search_top = 2*h/3
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = (err*90.0/160)/15
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)

                cv2.imshow("plane_view", plane_view )
                cv2.waitKey(1)
                

desired_aruco_dictionary = "DICT_6X6_250"          
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250, #set
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
  
def ardetect(frame):
        """
        Main method of the program.
        """


        dist_matrix = numpy.array(([[-0.0 , 0.0, -0.0 , 0.0 ,-0.0]]))
        camera_matrix = numpy.array([[265, 0, 160],[0, 265, 120],[0, 0, 1]])
        # mtx=np.array([[398.12724231  , 0.      ,   304.35638757],
        #  [  0.       ,  345.38259888, 282.49861858],
        #  [  0.,           0.,           1.        ]])

        # Check that we have a valid ArUco marker

        # Load the ArUco dictionary
        # print("[INFO] detecting '{}' markers...".format(
        # desired_aruco_dictionary))
        this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Start the video stream
        #   cap = cv2.VideoCapture(0)

        #   while(True):

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        #     ret, frame = cap.read()  

        # Detect ArUco markers in the video frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, this_aruco_dictionary, parameters=this_aruco_parameters)
        stop = 0

        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
                # stop = 1
                # Flatten the ArUco IDs list
                ids = ids.flatten()

                # Loop over the detected ArUco corners
                for (marker_corner, marker_id) in zip(corners, ids):
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.05, camera_matrix, dist_matrix)
                        distance = ((tvec[0][0][2] + 0.02) * 0.0254) * 100
                        print(distance)
                        if distance <1.2:
                                stop = 1

                        # Extract the marker corners
                        corners = marker_corner.reshape((4, 2))
                        (top_left, top_right, bottom_right, bottom_left) = corners
                                
                        # Convert the (x,y) coordinate pairs to integers
                        top_right = (int(top_right[0]), int(top_right[1]))
                        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                        top_left = (int(top_left[0]), int(top_left[1]))
                                
                        # Draw the bounding box of the ArUco detection
                        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                                
                        # Calculate and draw the center of the ArUco marker
                        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                                
                        # Draw the ArUco marker ID on the video frame
                        # The ID is always located at the top_left of the ArUco marker
                        cv2.putText(frame, str(marker_id), 
                                (top_left[0], top_left[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                        print('Aruco Marker Detected! ID:  ' + str(marker_id))
                        # Display the resulting frame
                        #     cv2.imshow('frame',frame)
                        
        return stop





thistime = float(0)
lasttime = float(0)
rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
