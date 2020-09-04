#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
import pytesseract
import numpy as np

kernel = np.ones((5, 5), np.uint8)
bordersize = 20
desk_img = [None] * 9

INTEGRATE_RATE = 100


class CameraDetectionNode(object):
    def __init__(self):
        rospy.init_node('camera_detection_node', anonymous=True)
        self.publisher_cells = rospy.Publisher("/human_turns", String, queue_size=0.1)
        self.human_side_is_zeros = bool(rospy.get_param("~human_side_is_zeros", True))
        self.camera_channel = rospy.get_param("~camera_channel", 2)
        self.videoCaptureObject = cv2.VideoCapture(self.camera_channel)
        self.output = ''
        self.output_history = ''
        self.stable_output_tick_threshold = 0

        self.integrate_rate = rospy.Rate(INTEGRATE_RATE)

    def current_status_update(self):
        ret, img = self.videoCaptureObject.read()
        grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, img) = cv2.threshold(grayImage, 220, 255, cv2.THRESH_BINARY)
        img = cv2.erode(img, kernel, iterations=1)

        desk_img[0] = img[15:134, 75:220]
        desk_img[1] = img[20:134, 240:390]
        desk_img[2] = img[25:134, 410:560]

        desk_img[3] = img[158:274, 75:230]
        desk_img[4] = img[158:274, 240:400]
        desk_img[5] = img[158:274, 410:565]

        desk_img[6] = img[293:414, 75:230]
        desk_img[7] = img[293:414, 240:400]
        desk_img[8] = img[293:414, 410:570]

        current_output = ''

        for i in range(len(desk_img)):

            cell = desk_img[i]
            cell = cv2.bitwise_not(cell)
            contours, _ = cv2.findContours(cell, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(cell, contours, -1, (255, 255, 255), thickness=-1)
            white_density = cv2.countNonZero(cell) / cell.size

            if not self.human_side_is_zeros:
                if 0.08 <= white_density <= 0.30 and str(i + 1) not in self.output_history:
                    current_output = str(i + 1)
            else:
                if 0.30 <= white_density <= 0.6 and str(i + 1) not in self.output_history:
                    current_output = str(i + 1)

        if self.output == current_output:
            self.stable_output_tick_threshold += 1
        else:
            self.stable_output_tick_threshold = 0
            self.output = current_output

    def publish_cells(self):
        self.current_status_update()
        if self.output != '' and self.stable_output_tick_threshold > 100 and self.output not in self.output_history:
            self.stable_output_tick_threshold = 0
            self.output_history += self.output
            self.publisher_cells.publish(self.output)

    def init_detection(self):
        while not rospy.is_shutdown():
            self.publish_cells()
            self.integrate_rate.sleep()


if __name__ == '__main__':
    try:
        node = CameraDetectionNode()
        node.init_detection()
    except rospy.ROSInterruptException:
        pass
