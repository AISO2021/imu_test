#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import sys

import rospy
from sensor_msgs.msg import Imu

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler(sys.stdout))

logger.setLevel(logging.DEBUG)

a = []

class RosImu:
    def __init__(self):
        rospy.init_node('MyTeleop')
        rospy.on_shutdown(self.on_shutdown)
        self.subscriber = rospy.Subscriber('/imu/data_raw', Imu, self.callback)
        self.ang = []
    def callback(self, data_raw):
        self.ang = [data_raw.angular_velocity.x, data_raw.angular_velocity.y, data_raw.angular_velocity.z]
        #logger.debug(self.ang)
        
    def on_shutdown(self):
        print("shutdown!")
        self.subscriber.unregister()

    def get_zang(self):
        return self.ang

if __name__ == '__main__':
    try:
        i = RosImu()
        while not rospy.is_shutdown():  # rospy.spin()と同じ
            rospy.sleep(0.2)
            a = i.get_zang()
            print(a)
            if a[2] > 0:
                print('ok')

    except rospy.ROSInterruptException:
        pass
