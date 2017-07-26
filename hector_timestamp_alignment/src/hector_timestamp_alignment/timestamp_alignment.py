#!/usr/bin/env python
from __future__ import print_function, division
import rospy

import sensor_msgs.msg
from dynamic_reconfigure.server import Server
from hector_timestamp_alignment.cfg import timestamp_alignmentConfig

class TimestampAlignment:
    def __init__(self, verbose):
        self.imu_sub = rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, self.imu_cb)
        self.imu_aligned_pub = rospy.Publisher("/imu/data_aligned", sensor_msgs.msg.Imu, queue_size=10)
        self.srv = Server(timestamp_alignmentConfig, self.dynamic_reconfigure_cb)
        self.delay = 0.0

    def imu_cb(self, imu_msg):
        print("Before: ", imu_msg.header.stamp)
        imu_msg.header.stamp = imu_msg.header.stamp + rospy.Duration(self.delay)
        print("After: ", imu_msg.header.stamp)
        self.imu_aligned_pub.publish(imu_msg)

    def dynamic_reconfigure_cb(self, config, level):
        print(config.imu_offset)
        self.delay = config.imu_offset
        return config


if __name__ == "__main__":
    rospy.init_node("timestamp_alignment_node")
    hazmat_detection = TimestampAlignment(verbose=False)
    while not rospy.is_shutdown():
        rospy.spin()
