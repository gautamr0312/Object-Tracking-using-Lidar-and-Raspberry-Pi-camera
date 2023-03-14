#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
from math import dist, atan2

# Constant Parameters
FRAME_WIDTH = 320               # Capture frame width 
FRAME_HEIGHT = 240              # Capture frame height
KERNEL_GAU = (7,7)              # Gaussian Blur Kernel
KERNEL_GAU_STD = 1              # Gaussian Blur Std. Deviation        
KERNEL_DIL = np.ones((5,5))     # Dilation Kernel Size
DP = 1.2                        # Inverse Ratios of accumulator resolution, the larger dp the smaller the accum array
PI = np.pi
FOV = 62.2



class VideoSubscriber(Node):

    def __init__(self):		
        # Creates the node.
        super().__init__('video_subs')

        # Set Parameters
        self.declare_parameter('show_image_bool', False)
        self.declare_parameter('window_name', "Raw Image")
        self.declare_parameter('video_source','/camera/image/compressed')       # Parameter to change the video topic 
       
        #Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
        
        #Only create image frames if we are not running headless (_display_image sets this)
        if(self._display_image):
        # Set Up Image Viewing
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
            cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
        
        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subs = self.create_subscription(
                CompressedImage,
                self.get_parameter('video_source').value,
                self._image_callback,
                image_qos_profile)
        self._video_subs # Prevents unused variable warning.

        # #HSV object detection 
        # blue = np.uint8([[[255, 0, 0]]]) #here insert the bgr values which you want to convert to hsv
        # hsvBlue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
        # self.start_detect = False
        # self.lower_bound = np.array([hsvBlue[0][0][0] - 10,100,100])
        # self.upper_bound = np.array([hsvBlue[0][0][0] + 10,255,255])
        # print(f"Lower Bound : {self.lower_bound} and Upper Bound: {self.upper_bound}")

        #HSV object detection 
        self.start_detect = False
        # self.lower_bound = np.empty([1, 3])
        # self.upper_bound = np.empty([1, 3])

        # HSV Value for Yellow Color
        self.lower_bound = np.array([19, 100, 100])	 
        self.upper_bound = np.array([39, 255, 255])

        #Angle publisher
        self.ang_wrt_lid = 0.0
        self.ang_pubs = self.create_publisher(Float64, "/angle_wrt_lidar", 10)
        # self.timer_period = 0.5  # seconds
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)


        # Hough Cirlces Parameters
        self.min_dist = 25                  # Min dist. between center of detected circles
        self.param1 = 75                   # Sensitivity of detection (Gradient Value)
        self.param2 = 28                     # Accuracy of detection (Accumulator Threshold)
        self.min_rad = 5                   # Min radius of circle
        self.max_rad = 200                  # Max Radius of circle

        # Canny Edge Detector Parameters
        self.thresh1 = 128
        self.thresh2 = 44

        # FPS Variables
        self.prev_frame_time = 0             # Processed last frame
        self.new_frame_time = 0              # Processed current frame

        self.prev_circle = None 

    def mouseRGB(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # checks mouse left button down condition
            print("Mouse Clicked")
            self.start_detect = True
            colorsB = self._imgBGR[y, x, 0]
            colorsG = self._imgBGR[y, x, 1]
            colorsR = self._imgBGR[y, x, 2]
            colors = self._imgBGR[y, x]
            hsv_value = np.uint8([[[colorsB, colorsG, colorsR]]])
            hsv = cv2.cvtColor(hsv_value, cv2.COLOR_BGR2HSV)
            self.lower_bound[0][0] = hsv[0][0][0] - 20
            self.upper_bound[0][0] = hsv[0][0][0] + 20
            self.lower_bound[0][1] = 100
            self.upper_bound[0][1] = 255
            self.lower_bound[0][2] = 100
            self.upper_bound[0][2] = 255
            print(f"Lower Bound : {self.lower_bound} and Upper Bound: {self.upper_bound}")

    def _image_callback(self, CompressedImage):	
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        # self.shape_detection()
        self.detect_object_hsv_hough()

    def dist_comp(self,chosen, prev, curr):
        if dist([chosen[0], chosen[1]], [prev[0], prev[1]]) >= dist([curr[0], curr[1]], [prev[0], prev[1]]):
            return True
        else:
            return False

    def shape_detection(self):

        #TODO.1 Blur the image using Gaussian Blurr
        imgBlur = cv2.GaussianBlur(self._imgBGR, KERNEL_GAU, KERNEL_GAU_STD)

        #TODO.2 Convert into GrayScale
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

        #TODO.4 Use Canny Edge Detector
        imgCanny = cv2.Canny(imgGray, self.thresh1, self.thresh2)

        #TODO.5 Perform Dilation to remove noise
        imgDil = cv2.dilate(imgGray, KERNEL_DIL, iterations=1)

        #TODO.6 Detect Circles
        circles_list = cv2.HoughCircles(imgDil, 
                                        cv2.HOUGH_GRADIENT, 
                                        DP,
                                        self.min_dist,
                                        param1 = self.param1,
                                        param2 = self.param2,
                                        minRadius = self.min_rad,
                                        maxRadius = self.max_rad)

        if circles_list is not None:
            circles_list = np.uint16(np.around(circles_list))
            chosen_circle = None
            for curr_circle in circles_list[0,:]:
                if chosen_circle is None:
                    chosen_circle = curr_circle

                if self.prev_circle is not None:
                    if self.dist_comp(chosen_circle, self.prev_circle, curr_circle) :
                        chosen_circle = curr_circle

            cv2.circle(self._imgBGR, (chosen_circle[0], chosen_circle[1]), 1, (255, 0,0), 3)
            cv2.circle(self._imgBGR, (chosen_circle[0], chosen_circle[1]), chosen_circle[2], (255, 0, 255), 3)
            cv2.putText(self._imgBGR, f"[{chosen_circle[0]}, {chosen_circle[1]}]", (chosen_circle[0] + 5, chosen_circle[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(self._imgBGR, f"Radius : {chosen_circle[2]}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            self.prev_circle = chosen_circle

            self.ang_wrt_lid =  - (( (chosen_circle[0] - 160) / FRAME_WIDTH ) * FOV ) * (PI / 180)
            # if self.ang_wrt_lid < 0:
            #     self.ang_wrt_lid = 2*PI - self.ang_wrt_lid
            self.publish_angle()

    def detect_object_hsv_hough(self):

        # TODO. convert to hsv colorspace
        hsv_img = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)

        # TODO. Find the colors within the boundaries
        mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)

        # TODO. Define kernel size
        kernel = np.ones((7, 7), np.uint8)

        # TODO. Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # TODO. Segment only the detected region
        segmented_img = cv2.bitwise_and(self._imgBGR, self._imgBGR, mask=mask)

        # # TODO. convert segmented image into grayscale
        img_gray = cv2.cvtColor(segmented_img, cv2.COLOR_BGR2GRAY)
        
        # TODO. dilate the grayscale image 
        img_dil = cv2.dilate(img_gray, KERNEL_DIL)

        # TODO. dilate the grayscale image 
        img_blur = cv2.GaussianBlur(img_dil, KERNEL_GAU, KERNEL_GAU_STD)

        circles_list = cv2.HoughCircles(img_blur, 
                                        cv2.HOUGH_GRADIENT, 
                                        DP,
                                        self.min_dist,
                                        param1 = self.param1,
                                        param2 = self.param2,
                                        minRadius = self.min_rad,
                                        maxRadius = self.max_rad)

        if circles_list is not None:
            circles_list = np.uint16(np.around(circles_list))
            chosen_circle = None
            for curr_circle in circles_list[0,:]:
                if chosen_circle is None:
                    chosen_circle = curr_circle

                if self.prev_circle is not None:
                    if self.dist_comp(chosen_circle, self.prev_circle, curr_circle) :
                        chosen_circle = curr_circle

            cv2.circle(self._imgBGR, (chosen_circle[0], chosen_circle[1]), 1, (255, 0,0), 3)
            cv2.circle(self._imgBGR, (chosen_circle[0], chosen_circle[1]), chosen_circle[2], (255, 0, 255), 3)
            cv2.putText(self._imgBGR, f"[{chosen_circle[0]}, {chosen_circle[1]}]", (chosen_circle[0] + 5, chosen_circle[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(self._imgBGR, f"Radius : {chosen_circle[2]}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            self.prev_circle = chosen_circle

            self.ang_wrt_lid =  - (( (chosen_circle[0] - 160) / FRAME_WIDTH ) * FOV ) * (PI / 180)

            # if self.ang_wrt_lid < 0:
            #     self.ang_wrt_lid = 2*PI - self.ang_wrt_lid
            self.publish_angle()
           
            # cv2.imshow("HSV", hsv_img)
            # cv2.imshow("Segmented Inage", segmented_img)
            # cv2.imshow("Final Image", self._imgBGR)
        else:
            self.ang_wrt_lid = 0.0

    def publish_angle(self):
        msg = Float64()
        msg.data = self.ang_wrt_lid
        print(f"the angle wrt lidar is {self.ang_wrt_lid}")
        self.ang_pubs.publish(msg)

    def get_image(self):
        return self._imgBGR

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        # cv2.setMouseCallback(self._titleOriginal, self.mouseRGB)
        # Cause a slight delay so image is displayed
        self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

    def get_user_input(self):
        return self._user_input


def main():
    rclpy.init() #init routine needed for ROS2.
    video_subscriber = VideoSubscriber() #Create class object to be used.

    while rclpy.ok():
        rclpy.spin_once(video_subscriber) # Trigger callback processing.
        if(video_subscriber._display_image):
            video_subscriber.show_image(video_subscriber.get_image())		
            if video_subscriber.get_user_input() == ord('q'):
                cv2.destroyAllWindows()
                break

    #Clean up and shutdown.
    video_subscriber.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()