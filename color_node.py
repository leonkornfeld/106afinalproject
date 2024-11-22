#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

COLOR_RANGES = {
    "red": [(np.array([0, 70, 50]), np.array([10, 255, 255])),
            (np.array([170, 50, 50]), np.array([180, 255, 255]))],
    "yellow": [(np.array([20, 100, 100]), np.array([30, 255, 255]))],
    "blue": [(np.array([100, 150, 50]), np.array([140, 255, 255]))],
    "orange": [(np.array([10, 100, 100]), np.array([20, 255, 255]))],
    "pink": [(np.array([140, 50, 50]), np.array([170, 255, 255]))],
    "green": [(np.array([40, 50, 50]), np.array([80, 255, 255]))],
    "black": [(np.array([0, 0, 0]), np.array([180, 255, 30]))],  # Low V values for black
    "purple": [(np.array([125, 50, 50]), np.array([150, 255, 255]))]
}

BOX_COLORS = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "blue": (255, 0, 0),
    "orange": (0, 165, 255),
    "pink": (255, 105, 180),
    "green": (0, 255, 0),
    "black": (0, 0, 0),
    "purple": (128, 0, 128)
}

class RightHandCamera:
    def __init__(self):
        rospy.init_node('right_hand_camera_node', anonymous=True)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            '/io/internal_camera/right_hand_camera/image',
            Image,
            self.image_callback
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            image_with_boxes, color_order = self.find_and_draw_colored_boxes(cv_image)

            cv2.imshow('Right Hand Camera (detected boxes)', image_with_boxes)

            print(color_order)

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def find_and_draw_colored_boxes(self, image):
        if image is None:
            print("Error: Could not read the image.")
            return
        
        image = np.copy(image)
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_order = []

        for color_name, ranges in COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 20 and h > 20:  


                    color_order.append((x, color_name))

                    cv2.rectangle(image, (x, y), (x + w, y + h), BOX_COLORS[color_name], 2)
                    cv2.putText(image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLORS[color_name], 2)

        color_order = sorted(color_order)

        return image, color_order

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        camera = RightHandCamera()
        camera.run()
    except rospy.ROSInterruptException:
        pass