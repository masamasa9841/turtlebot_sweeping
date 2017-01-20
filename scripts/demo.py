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
        self.marker.id +=1
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.a = 0.5
        self.X_KOUSHI = 10
        self.Y_KOUSHI = 10
        self.x_math = 240 * 0.05 / self.X_KOUSHI
        self.y_math = 238 * 0.05 / self.Y_KOUSHI
        self.id_list = [[0 for i in range(self.X_KOUSHI)] for j in range(self.Y_KOUSHI)]
        self.mag_list = [[0 for i in range(self.X_KOUSHI)] for j in range(self.Y_KOUSHI)]

    def mark_mag(self):
        for i in range(len(self.x)):
            for j in range(self.X_KOUSHI):
                for k in range(self.Y_KOUSHI):
                    if 0 < self.x[i] - self.x_math * j < self.x_math and 0 < self.y[i] - self.y_math * k < self.y_math:
                        self.id_list[k][j] += 1
                        self.mag_list[k][j] += self.mag[i]
                    if rospy.is_shutdown():
                        break

        for j in range(self.X_KOUSHI):
            for k in range(self.Y_KOUSHI):
                if not self.id_list[k][j] == 0:
                    self.marker.id += 1
                    mag_ave = self.mag_list[k][j] / self.id_list[k][j]
                    mag_height = mag_ave / -4.0 * 10000000
                    hsv = (mag_ave * 100000 + 1) * 120
                    r, g, b = colorsys.hsv_to_rgb(hsv/360,1,1)
                    self.marker.color.r = r
                    self.marker.color.g = g
                    self.marker.color.b = b
                    self.marker.pose.position.x = 240 * 0.05 / self.X_KOUSHI * j
                    self.marker.pose.position.y = 238 * 0.05 / self.Y_KOUSHI * k
                    self.marker.pose.position.z = mag_height / 2.0
                    self.marker.scale.z = mag_height 
                    self.pub3.publish(self.marker)
                if rospy.is_shutdown():
                    break

if __name__ == '__main__':
    t = navigation()
    try:
        t.mark_mag()
    except rospy.ROSInterruptException:
        pass
