#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
import rospy
import math 
import tf
import colorsys
from tf import TransformListener
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Twist
from sensor_msgs.msg import MagneticField

class navigation(object):
    def __init__(self): 
        rospy.loginfo("Waiting for the tf to come up")
        rospy.sleep(2)
        rospy.init_node('sweep_nav')
        sub = rospy.Subscriber('/imu/mag', MagneticField, self.callback, queue_size = 10)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('way_point', Marker, queue_size=10)
        self.pub3 = rospy.Publisher('z_mag', Marker, queue_size=10)

        self.marker = Marker()
        self.id_new = 0
        self.id_old = 0
        self.count = 1
        self.z = []
        self.marker.header.frame_id = 'map'
        self.marker.type = 3
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 0.5
        self.marker.scale.z = 0.1

        self.tf = TransformListener()
        self.vel = Twist()
        self.speed = 0.15
        self.X_KOUSHI = 150
        self.Y_KOUSHI = 150
        self.id_list = [[i + self.X_KOUSHI * j for i in range(self.X_KOUSHI)] for j in range(self.Y_KOUSHI)]
        self.mag = 0
        self.f = open('../map/way.txt','w')
        self.now = rospy.get_time()

    def callback(self, msg): 
        #T
        self.mag = (msg.magnetic_field.z + 4.35 * 0.00001) * 0.15 #bias

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
        self.mark(x[c], y[c], pos[0], pos[1])
        self.mark_mag(x[0], y[0], pos[0], pos[1])
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

    def angle_loop(self, c = 0, vel = 1, a=8):
        destance, ang = self.ask_destance_and_ang(c, a)
        while not ang[0] < ang[2] < ang[1] and not rospy.is_shutdown(): 
            destance, ang= self.ask_destance_and_ang(c, a)
            if vel < 0.5:
                if abs(ang[3]) > 10:
                    speed = 1
                else: speed = vel
            else: speed = vel
            if ang[3] >= 0: self.vel.angular.z =  speed
            else : self.vel.angular.z = -speed
            self.vel.linear.x = 0
            self.pub.publish(self.vel)
        self.vel.angular.z = 0
        self.pub.publish(self.vel)
        return destance

    def destance_loop(self, c = 0, vel = 0, a = 2):
        destance, ang = self.ask_destance_and_ang(c, a)
        while not destance <= 0.03 and not rospy.is_shutdown():
            destance = self.angle_loop(c, 0.35, 1)
            self.vel.linear.x = vel
            self.pub.publish(self.vel)#straight
        self.speed_0(self.speed)
        self.pub.publish(self.vel)
    
    def speed_0(self, s):
        for i in range(1,11):
            if rospy.is_shutdown(): break
            self.vel.linear.x = self.speed - s/10*i 
            self.pub.publish(self.vel)#straight
            rospy.sleep(0.05)

    def nav(self):   
        i = 100 #start_waypoint
        while not rospy.is_shutdown():
            print str(i)+"times"
            self.angle_loop(i, 2, 5)
            self.angle_loop(i, 0.35, 1)
            self.destance_loop(i, self.speed)
            i += 1
            self.vel.angular.z = 0; 
            self.pub.publish(self.vel)
        self.f.close()

    def read_waypoint(self):
        x, y, z = [], [], []
        f = open('../map/map.txt','r')
        way = f.readlines()
        for i in range(len(way)):
            way[i] = way[i].replace('\n',"")
            point = way[i].split(",")
            x.append(float(point[0]))
            y.append(float(point[1]))
        f.close()
        x.append("exit")
        y.append("exit")
        z.append("exit")
        return x, y, z

    def mark(self, x = 0, y = 0, x2 = 0, y2 = 0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = 4#line_strip
        marker.action = marker.ADD
        marker.scale.x = 0.01
        marker.color.a = 1
        marker.color.b = 1
        marker.points.append(Point(x,y,0))
        marker.points.append(Point(x2,y2,0))
        self.pub2.publish(marker)

    def mark_mag(self,x_length = 240,y_length = 238, x = 0, y = 0):
        self.id_old = self.id_new 
        z = self.mag / 2.0 * -10000000 / 2.0
        x_math = x_length * 0.05 / self.X_KOUSHI
        y_math = y_length * 0.05 / self.Y_KOUSHI
        for i in range(self.X_KOUSHI):
            for j in range(self.Y_KOUSHI):
                if 0 < x - x_math * i < x_math and 0 < y - y_math * j < y_math :  
                    self.id_new = self.id_list[j][i]
        if self.id_new == self.id_old:
            self.count += 1
            self.z.append(z)
        else : 
            self.count = 1
            self.z = []
            self.z.append(z)
        self.marker.id = self.id_new
        hsv = (self.mag * 100000 + 1 )*120
        r, g, b = colorsys.hsv_to_rgb(hsv/360,1,1)
        self.marker.color.r = r
        self.marker.color.g = g
        self.marker.color.b = b
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = sum(self.z) / self.count /2.0 +0.10
        self.marker.scale.z = sum(self.z) /  self.count
        self.pub3.publish(self.marker)
        after = rospy.get_time()
        if after - self.now > 0.05: #0.1秒ごとに記録
            self.now = rospy.get_time()
            self.f.write(str(x)+" "+str(y)+" "+str(self.mag)+"\n")

if __name__ == '__main__':
    t = navigation()
    try:
        t.nav()
    except rospy.ROSInterruptException:
        pass
