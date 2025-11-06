#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImageExtractor:
    def __init__(self):
        rospy.init_node("image_extract_node", anonymous=True)

        self.image_counter = 0
        self.downsample_scale = 100
        self.bridge = CvBridge()
        self.image_output_dir = "./data/images"

        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=10)
        
        rospy.loginfo("The image_extract_node is up.")
        rospy.spin()

    def callback(self, image_msg):
        if self.image_counter % self.downsample_scale == 0:
            bgr_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            image_name = f'{self.image_counter}.png'
            image_path = os.path.join(self.image_output_dir, image_name)
            cv2.imwrite(image_path, bgr_image)      # image will be saved in rgb format
            rospy.loginfo(image_path + " has been saved.")
        self.image_counter += 1


def main():
    ImageExtractor()


if __name__ == "__main__":
    main()
        
