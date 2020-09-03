#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

INTEGRATE_RATE = 100


class CameraDetectionNode(object):
    def __init__(self):
        rospy.init_node('camera_detection_node', anonymous=True)
        self.publisher_cells = rospy.Publisher("/human_turns", String, queue_size=0.1)

        self.integrate_rate = rospy.Rate(INTEGRATE_RATE)

    def publish_cells(self):
        self.publisher_cells.publish('Stepa')

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