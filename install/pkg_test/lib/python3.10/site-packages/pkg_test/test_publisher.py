#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 
from cv_bridge import CvBridge
class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__("image_publisher")
        self.imahge_publisher = self.create_publisher(Image , 'camera_image',10)
        self.timer = self.create_timer(1.0,self.publish_image)
        
        self.bridge =CvBridge() 

        self.cap = cv2.VideoCapture(0)

    def publish_image(self):
        
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()