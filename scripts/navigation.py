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
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.tf = TransformListener()
        self.vel = Twist()
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
        return position, self.quaternion_to_euler_to_deg(quaternion)

    def quaternion_to_euler_to_deg(self, q):
        e = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
        deg = math.degrees(e[2])
        #print d 
        return deg

    def nav(self):   
        c = 0
        x, y, z = self.read_waypoint()
        while not rospy.is_shutdown():
            pos, deg = self.tf_change()
            if x[c] == "exit": break

            #if not deg < -170:
                #self.vel.angular.z = 0.4
            #else: self.vel.angular.z = 0; 
            #self.pub.publish(self.vel)
            #print x[c]
            print deg
            c += 1

    def read_waypoint(self):
        x, y, z = [], [], []
        f = open('/home/masaya/catkin_ws/src/turtlebot_sweeping/map/map.txt','r')
        way = f.readlines()
        for i in range(1,len(way)):
            way[i] = way[i].replace('\n',"")
            point = way[i].split(",")
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        f.close()
        x.append("exit")
        y.append("exit")
        z.append("exit")
        return x, y, z


if __name__ == '__main__':
    t = navigation()
    try:
            t.nav()
            #rospy.sleep(0.1)
            
    except rospy.ROSInterruptException:
        pass
