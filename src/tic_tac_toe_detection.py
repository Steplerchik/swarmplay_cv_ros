#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
import pytesseract
import numpy as np

videoCaptureObject = cv2.VideoCapture(0)
kernel = np.ones((5, 5), np.uint8)
kernel2 = np.ones((7, 7), np.uint8)
bordersize = 20
desk_img = [None] * 9
desk_status = ['_'] * 9

INTEGRATE_RATE = 100


class CameraDetectionNode(object):
    def __init__(self):
        rospy.init_node('camera_detection_node', anonymous=True)
        self.publisher_cells = rospy.Publisher("/human_turns", String, queue_size=0.1)

        self.integrate_rate = rospy.Rate(INTEGRATE_RATE)

    def current_status(self):
        ret, img = videoCaptureObject.read()
        grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, img) = cv2.threshold(grayImage, 217, 255, cv2.THRESH_BINARY)
        img = cv2.erode(img, kernel, iterations=1)

        desk_img[0] = img[25:134, 75:250]
        desk_img[1] = img[20:134, 200:430]
        desk_img[2] = img[25:134, 410:570]

        desk_img[3] = img[158:274, 75:250]
        desk_img[4] = img[158:274, 200:430]
        desk_img[5] = img[158:274, 410:575]

        desk_img[6] = img[293:414, 75:250]
        desk_img[7] = img[293:414, 200:430]
        desk_img[8] = img[293:414, 410:570]

        for i in range(len(desk_img)):
            cell = desk_img[i]
            cell = cv2.copyMakeBorder(
                cell,
                top=bordersize,
                bottom=bordersize,
                left=bordersize,
                right=bordersize,
                borderType=cv2.BORDER_CONSTANT,
                value=[255, 255, 255])

            cell_text = pytesseract.image_to_string(cell).lower()
            if 'x' in cell_text:
                desk_status[i] = 'x'
            elif 'o' in cell_text:
                desk_status[i] = '0'

        return ''.join(desk_status)

    def publish_cells(self):
        output = self.current_status()
        self.publisher_cells.publish(output)

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
