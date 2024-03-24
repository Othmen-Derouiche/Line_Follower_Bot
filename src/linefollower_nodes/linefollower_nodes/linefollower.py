#!/usr/bin/env python3


"""
A ROS2 node used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# Image is a message type defined in the sensor_msgs package of ROS 2. 
# It represents images captured by sensors such as cameras.

from geometry_msgs.msg import Twist
# Twist is a message type defined in the geometry_msgs package of ROS 2. 
# It represents linear and angular velocities in 3D space.

from std_srvs.srv import Empty
"""--------------------------------------------------------------------------------"""
import cv2 
import numpy as np 
import cv_bridge
"""--------------------------------------------------------------------------------"""
# BGR values to filter only the selected color range

global lower_bgr_values
global upper_bgr_values
global crop_h_start, crop_h_stop, crop_w_start, crop_w_stop 
class LineFollowerNode(Node): 
    def __init__(self):
        super().__init__("line_follower") 

        lower_bgr_values = np.array([31,  42,  53])
        upper_bgr_values = np.array([255, 255, 255])
        
        self.timer_frequency = 1.0 / 0.06
        self.image_input = 0 
    
        # Minimum size for a contour to be considered anything
        self.minArea = 500 
        # Minimum size for a contour to be considered part of the track
        self.minAreaTrack = 5000

        # Robot's speed when following the line
        self.robot_speed = 0.2
        
        # create a bridge between ROS and OPENCV
        self.bridge = cv_bridge.CvBridge()

        self.publisher = self.create_publisher(Twist , '/cmd_vel',10)
        self.get_logger().info("Line_Follower_publisher_node has been started ")

        self.subscriber = self.create_subscription(Image , 'camera/image_raw',self.convert_image_callback)
        self.get_logger().info("Line_Follower_node_subscriber has been started ")

        self.timer = self.create_timer(1.0/self.timer_frequency,self.timer_callback)

        self.start_service = self.create_service(Empty , 'start_follower' , self.start_follower_callback)
        self.stop_service = self.create_service(Empty , 'stop_follower' , self.stop_follower_callback)
    """--------------------------------------------------------------------------------"""
    # function called whenever a new Image message arrives
    def convert_image_callback(self , msg):

        self.image_input = self.bridge.imgmsg_to_cv2(msg,desired_encoding= 'bgr8')
        self.get_logger().info('Received image')

    """--------------------------------------------------------------------------------"""   
    def timer_callback(self):

        global crop_w_start

        # Wait for the first image to be received
        if type(self.image_input) != np.ndarray:
            return

        height , width , _ = self.image_input.shape 
        image = self.image_input.copy()

        crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = self.crop_size(height, width)



        pass

    """--------------------------------------------------------------------------------"""
    def crop_size(height,width):
        # output :
        # (Height_upper_boundary, Height_lower_boundary,Width_left_boundary, Width_right_boundary)
        return(height//3 , height , width//4 , 3*width//4)
    
    """--------------------------------------------------------------------------------"""
    #  function aiming to find and process contours within this binary image, 
    # extracting relevant information and returning it.
    def get_contour_data(self , mask , out):
        # Input : 
        # - a binary image mask as input, which typically
        # represents some segmented area or object in an image 

        # get a list of contours 
        contours , _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

        mark = {}
        line = {}

        for contour in contours :

            # calculate the moments which are mathematical descriptors of the shape and spatial distribution of the contour.
            M = cv2.moments(contour)

            if M['m00'] > self.minArea :
                if ['m00'] > self.minAreaTrack:
                    # coutour is part of the track 
                    line['x'] = crop_w_start + int(M['m10']/M['m00'])
                    line['y'] = int(M['m01']/M['m00'])

                    # plot the area in light blue
                    cv2.drawContours(out,contour,-1 , (255,255,0) , -1)
                    cv2.putText(out,str(M['M00']) , int(M['m10']/M['m00']) , int(M['m01']/M['m00']) , 
                                cv2.FONT_HERSHEY_PLAIN , 2 , (255,0,255) , 2)

        if mark and line : 
            # if both contours exist 
            if mark['x'] > line['x']            


        pass
    """--------------------------------------------------------------------------------"""
    def start_follower_callback(self):
        
        pass
    
    def stop_follower_callback(self):
        
        pass
    """--------------------------------------------------------------------------------"""
def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()