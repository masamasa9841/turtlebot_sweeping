#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
import rospy
import colorsys
from visualization_msgs.msg import Marker
if __name__ == '__main__':
    X_KOUSHI = 30
    Y_KOUSHI = 30
    rospy.init_node('demo')
    pub3 = rospy.Publisher('z_mag', Marker, queue_size=1000)
    read, x, y, mag = [], [], [], []
    f = open('../map/way.txt')
    read = f.readlines()
    for i in range(len(read)):
        read[i] = read[i].replace('\n',"")
        a, b, c = read[i].split(" ")
        x.append(float(a))
        y.append(float(b))
        mag.append(float(c))
    f.close()
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.type = 3
    marker.action = marker.ADD
    marker.id += 0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.color.a = 0.5
    x_math = 240 * 0.05 / X_KOUSHI
    y_math = 238 * 0.05 / Y_KOUSHI
    id_list = [[0 for i in range(X_KOUSHI)] for j in range(Y_KOUSHI)]
    mag_list = [[0 for i in range(X_KOUSHI)] for j in range(Y_KOUSHI)]
    for i in range(len(x)):
        for j in range(X_KOUSHI):
            for k in range(Y_KOUSHI):
                if 0 < x[i] - x_math * j < x_math and 0 < y[i] - y_math * k < y_math:
                    id_list[k][j] += 1
                    mag_list[k][j] += mag[i]
                if rospy.is_shutdown():
                    break
    for j in range(X_KOUSHI):
        for k in range(Y_KOUSHI):
            if not id_list[k][j] == 0:
                marker.id += 1
                mag_ave = mag_list[k][j] / id_list[k][j]
                mag_height = mag_ave / -4.0 * 10000000
                hsv = (mag_ave * 100000 + 1) * 120
                r, g, b = colorsys.hsv_to_rgb(hsv / 360,1,1)
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.pose.position.x = 240 * 0.05 / X_KOUSHI * j
                marker.pose.position.y = 238 * 0.05 / Y_KOUSHI * k
                marker.pose.position.z = mag_height / 2.0
                marker.scale.z = mag_height 
                pub3.publish(marker)
                rospy.sleep(0.1)
            if rospy.is_shutdown():
                break
