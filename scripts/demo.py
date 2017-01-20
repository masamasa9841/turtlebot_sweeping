#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
import rospy
import colorsys
from visualization_msgs.msg import Marker

class navigation(object):
    def __init__(self): 
        rospy.init_node('demo')
        self.pub3 = rospy.Publisher('z_mag', Marker, queue_size=1000)
        read, self.x, self.y, self.mag = [], [], [], []
        f = open('../map/way.txt')
        read = f.readlines()
        for i in range(len(read)):
            read[i] = read[i].replace('\n',"")
            x, y, z = read[i].split(" ")
            self.x.append(float(x))
            self.y.append(float(y))
            self.mag.append(float(z))
        f.close()
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.type = 3
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 0.5
        self.X_KOUSHI = 100
        self.Y_KOUSHI = 100
        self.x_math = 240 * 0.05 / self.X_KOUSHI
        self.y_math = 238 * 0.05 / self.Y_KOUSHI
        self.id_list = [[i + self.X_KOUSHI * j for i in range(self.X_KOUSHI)] for j in range(self.Y_KOUSHI)]
        self.id_new = 0

    def mark_mag(self,x_length = 240,y_length = 238, x = 0, y = 0):
        id_lin = [],[]
        for i in range(len(self.x)):
            self.id_old = self.id_new
            z = self.mag[i] / -4.0 * 10000000
            for j in range(self.X_KOUSHI):
                for k in range(self.Y_KOUSHI):
                    if 0 < self.x[i] - self.x_math * j < self.x_math and 0 < self.y[i] - self.y_math * k < self.y_math:
                        self.id_new = self.id_list[k][j]
                        #id_lin.append(self.id_new)
            self.marker.id = self.id_new
            hsv = (self.mag[i] * 100000 + 1) * 120
            r, g, b = colorsys.hsv_to_rgb(hsv/360,1,1)
            self.marker.color.r = r
            self.marker.color.g = g
            self.marker.color.b = b
            self.marker.pose.position.x = self.x[i]
            self.marker.pose.position.y = self.y[i]
            self.marker.pose.position.z = z / 2.0
            self.marker.scale.z = z 
            if not self.id_new == self.id_old:
                self.pub3.publish(self.marker)
            if rospy.is_shutdown():
                break

if __name__ == '__main__':
    t = navigation()
    try:
        t.mark_mag()
    except rospy.ROSInterruptException:
        pass
