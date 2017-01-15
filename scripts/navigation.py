#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
import rospy
import math 
import time
import tf
from tf import TransformListener
from geometry_msgs.msg import Point, Quaternion, Twist

class navigation(object):
    def __init__(self): 
        rospy.init_node('sweep_nav')
        self.tf = TransformListener()
        rospy.loginfo("Waiting for the tf to come up")
        time.sleep(2)

    def tf_change(self):
        try:
            if self.tf.frameExists("/map") and self.tf.frameExists("/base_footprint"):
                now = rospy.Time.now()
                self.tf.waitForTransform("/map", "/base_footprint", now, rospy.Duration(2.0))
                t = self.tf.getLatestCommonTime("/map", "/base_footprint")
                position, quaternion = self.tf.lookupTransform("/map", "/base_footprint", t)
            else: position, quaternion = [0,0,0,0], [0,0,0,0]
        except:
            pass
        return position, quaternion

    def quaternion_to_euler_to_deg(self):
        p, q = self.tf_change()
        e = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
        d = math.degrees(e[2])
        return d


if __name__ == '__main__':
    t = navigation()
    try:
        while not rospy.is_shutdown():
            t.quaternion_to_euler_to_deg()
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
