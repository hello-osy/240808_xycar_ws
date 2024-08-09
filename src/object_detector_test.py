#!/usr/bin/env python3

import rospy
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class YoloV5RosNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('yolov5_ros_node', anonymous=True)

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/xytron/object_detector_model/best.pt')  # replace with your trained model path

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Perform inference
            results = self.model(cv_image)

            # Draw bounding boxes on the image
            img_with_boxes = results.render()[0]

            # Display the image
            cv2.imshow('YOLOv5 Detection', img_with_boxes)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error in image_callback: %s", e)

if __name__ == '__main__':
    try:
        yolo_node = YoloV5RosNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()

