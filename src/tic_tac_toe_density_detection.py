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
        rospy.Subscriber("/move_drone", String, self.new_game_callback)
        self.human_side_is_zeros = bool(rospy.get_param("~human_side_is_zeros", True))
        self.camera_channel = rospy.get_param("~camera_channel", 2)
        self.videoCaptureObject = cv2.VideoCapture(self.camera_channel)
        self.videoCaptureObject.set(cv2.CAP_PROP_ZOOM, 253)
        self.videoCaptureObject.set(cv2.CAP_PROP_TILT, 7200)
        self.output = ''
        self.output_history = ''
        self.stable_output_tick_threshold = 0

        self.integrate_rate = rospy.Rate(INTEGRATE_RATE)

    def new_game_callback(self, data):
        if data.data == "New_game":
            self.output_history = ''
            print('Output history cleared')

    def current_status_update(self):
        ret, img = self.videoCaptureObject.read()
        grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, img) = cv2.threshold(grayImage, 190, 255, cv2.THRESH_BINARY)
        img = cv2.erode(img, kernel, iterations=1)

        desk_img[0] = img[0:148, 35:208]
        desk_img[1] = img[0:148, 223:400]
        desk_img[2] = img[0:148, 418:585]

        desk_img[3] = img[164:304, 26:200]
        desk_img[4] = img[163:308, 219:397]
        desk_img[5] = img[160:307, 415:596]

        desk_img[6] = img[320:466, 18:193]
        desk_img[7] = img[320:466, 213:393]
        desk_img[8] = img[324:466, 412:597]

        current_output = ''

        for i in range(len(desk_img)):

            cell = desk_img[i]
            cell = cv2.bitwise_not(cell)
            contours, _ = cv2.findContours(cell, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(cell, contours, -1, (255, 255, 255), thickness=-1)
            white_density = cv2.countNonZero(cell) / cell.size

            if not self.human_side_is_zeros:
                if 0.08 <= white_density <= 0.25 and str(i + 1) not in self.output_history:
                    current_output = str(i + 1)
            else:
                if 0.25 <= white_density <= 0.6 and str(i + 1) not in self.output_history:
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
