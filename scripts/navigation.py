#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
import rospy
import math 
import tf
from tf import TransformListener
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Twist

class navigation(object):
    def __init__(self): 
        rospy.init_node('sweep_nav')
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('way_point', Marker, queue_size=10)
        self.tf = TransformListener()
        self.vel = Twist()
        rospy.loginfo("Waiting for the tf to come up")
        rospy.sleep(1)

    def tf_change(self):
        try:
            if self.tf.frameExists("/map") and self.tf.frameExists("/base_footprint"):
                now = rospy.Time.now()
                self.tf.waitForTransform("/map", "/base_footprint", now, rospy.Duration(2.0))
                t = self.tf.getLatestCommonTime("/map", "/base_footprint")
                position, quaternion = self.tf.lookupTransform("/map", "/base_footprint", t)
            else: position, quaternion = [0,0,0,0], [0,0,0,0]
        except:
            position, quaternion = [0,0,0,0], [0,0,0,0]
        return position, self.quaternion_to_euler_to_deg(quaternion)

    def quaternion_to_euler_to_deg(self, q):
        e = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
        deg = math.degrees(e[2])
        if deg < 0: deg = 360 + deg
        return deg

    def ask_destance_and_ang(self,c=0, a=10):
        range_ang = [0, 0, 0, 0] # [0]:- [1]:+ [2]:deg [3]:difference
        x, y, z = self.read_waypoint()
        pos, deg = self.tf_change()
        ang = math.degrees(math.atan2(x[c] - pos[0], y[c] - pos[1]))
        self.mark(x[c], y[c])
        destance = math.sqrt(math.pow(x[c] - pos[0], 2.0) + math.pow(y[c] - pos[1], 2.0))
        ang = 90 - ang
        if ang < 0: ang = ang + 360
        if ang < 0: ang = ang + 180
        range_ang[0] = ang - a
        if deg < 0: range_ang[0] = ang + 360
        range_ang[1] = ang + a
        range_ang[2] = deg
        range_ang[3] = ang - deg
        return destance, range_ang

    def angle_loop(self, c = 0, vel = 1, a=10):
        destance, ang = self.ask_destance_and_ang(c, a)
        while not ang[0] < ang[2] < ang[1] and not rospy.is_shutdown(): 
            destance, ang= self.ask_destance_and_ang(c, a)
            if ang[3] >= 0: self.vel.angular.z =  vel
            if ang[3] <= 0: self.vel.angular.z = -vel
            self.pub.publish(self.vel)
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

    def destance_loop(self, c = 0, vel = 0.15,a = 1):
        destance, ang = self.ask_destance_and_ang(c, a)
        while not destance <=0.1 and not rospy.is_shutdown():
            destance, ang = self.ask_destance_and_ang(c, a)
            while not ang[0] < ang[2] < ang[1] and not rospy.is_shutdown(): 
                destance, ang= self.ask_destance_and_ang(c, a)
                if ang[3] >= 0: self.vel.angular.z =  0.3
                if ang[3] <= 0: self.vel.angular.z = -0.3
                self.vel.linear.x = 0
                self.pub.publish(self.vel)#turn
            self.vel.angular.z = 0
            self.vel.linear.x = vel
            self.pub.publish(self.vel)#straight
        self.vel.linear.x = 0
        self.pub.publish(self.vel)
            

    def nav(self):   
        i = 45
        while not rospy.is_shutdown():
            print i
            self.angle_loop(i, 2.5, 10)
            self.angle_loop(i, 0.3, 2)
            self.destance_loop(i)
            i += 1
            self.vel.angular.z = 0; 
            self.pub.publish(self.vel)

    def read_waypoint(self):
        x, y, z = [], [], []
        f = open('/home/masaya/catkin_ws/src/turtlebot_sweeping/map/map.txt','r')
        way = f.readlines()
        for i in range(1,len(way)):
            way[i] = way[i].replace('\n',"")
            point = way[i].split(",")
            x.append(float(point[0]))
            y.append(float(point[1]))
            z.append(point[2])
        f.close()
        x.append("exit")
        y.append("exit")
        z.append("exit")
        return x, y, z

    def mark(self, x = 0, y = 0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = 2
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1
        marker.color.r = 1
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration()
        self.pub2.publish(marker)

if __name__ == '__main__':
    t = navigation()
    try:
            t.nav()
            #rospy.sleep(0.1)
            
    except rospy.ROSInterruptException:
        pass
