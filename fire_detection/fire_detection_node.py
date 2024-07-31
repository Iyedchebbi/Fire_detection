#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage  # For publishing compressed images
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        
        # Subscriber to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            '/fire_detection/image_processed',
            10
        )
        
        # Initialize CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detect fire and process the frame
        fire_detected, processed_frame = self.detect_fire(frame)
        
        if fire_detected:
            self.get_logger().info('Fire detected!')
        
        # Draw text on the frame
        cv2.putText(processed_frame, 'Fire Detected!' if fire_detected else 'No Fire Detected', 
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        # Publish the processed frame
        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
            self.publisher.publish(processed_image_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
        
    def detect_fire(self, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for red color
        lower_red1 = np.array([0, 50, 50], dtype="uint8")
        upper_red1 = np.array([10, 255, 255], dtype="uint8")
        lower_red2 = np.array([160, 50, 50], dtype="uint8")
        upper_red2 = np.array([180, 255, 255], dtype="uint8")

        # Create masks for the two red ranges
        mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask
        fire_contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        fire_detected = False
        for contour in fire_contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Adjust this threshold as needed
                # Draw rectangle around detected red area
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                fire_detected = True

        return fire_detected, frame

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

