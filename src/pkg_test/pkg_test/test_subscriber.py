#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriberNode(Node): 
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscriber = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") 
        
        gray_image = cv2.cvtColor('cv_image',cv2.COLOR_BGR2GRAY)
        cv2.imshow('gray_image',gray_image)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()