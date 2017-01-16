#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import math 
import rospy
import time
import numpy as np
from tf import TransformListener
from actionlib_msgs.msg import *
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist

class aho(object):
    def __init__(self):
        self.count = 0
        self.kazu = [0,0]
        rospy.init_node('simple_marker')
        self.marker_array = MarkerArray()
        self.tf = TransformListener() 
        self.x = 0 
        self.y = 0 
        self.id = 0
        self.z  = 0.0
        self.sub = rospy.Subscriber('/imu/mag',MagneticField,self.callback2,queue_size=10)
        self.pub = rospy.Publisher('marker',Marker,queue_size=10) 
        self.pub2 = rospy.Publisher('marker_array',MarkerArray,queue_size=10)
        self.pub3 = rospy.Publisher('TB_Point',Point,queue_size=10)
        self.pub4 = rospy.Publisher('tff',Point,queue_size=10)
        #pgm,yamlの読み込み
        pgm,yaml = [],[]
        f = open('/home/masaya/map/19_floor_extended.pgm')
        pgm = f.readlines()
        for i in range(len(pgm)):
            pgm[i] = pgm[i].replace('\n',"")
            pgm[i] = pgm[i].replace('\r',"")
        f.close()
        self.length = pgm[2].split(" ")
        f = open('/home/masaya/map/19_floor_extended.yaml')
        yaml = f.readlines()
        for i in range(len(yaml)):
            yaml[i] = yaml[i].replace('\n',"")
            yaml[i] = yaml[i].replace('\r',"")
        f.close()
        
        self.x_plan   = 1  #1,25 でうまくいく
        self.X_KOUSHI = 150#分割
        self.X_MIN    = 0.0
        self.X_MAX    = float(self.length[0]) * float(yaml[1].replace('resolution: ',''))
        self.X_LENGTH = abs(self.X_MAX - self.X_MIN)
        self.X_MATH   = self.X_LENGTH / self.X_KOUSHI
        self.y_plan   = 213
        self.Y_KOUSHI = 150#分割
        self.Y_MIN    = 0.0
        self.Y_MAX    = float(self.length[1]) * float(yaml[1].replace('resolution: ',''))
        self.X_LENGTH = abs(self.X_MAX - self.X_MIN)
        self.Y_LENGTH = abs(self.Y_MAX - self.Y_MIN)
        self.Y_MATH   = self.Y_LENGTH / self.Y_KOUSHI
        self.coordinate = np.array([[pgm[i + j * int(self.length[0]) + 4 ] for i in range(int(self.length[0]))] for j in range(int(self.length[1]))])
        self.id_list = [[i + self.X_KOUSHI * j for i in range(self.X_KOUSHI)] for j in range(self.Y_KOUSHI)]
        self.south, self.west, self.east, self.north = False, False, False, False
        self.aho = [0,0]
        self.point = Point()
        self.point.x = 1 
        self.point.y = 1
        self.point.z = 180
        self.point2 = Point()

        for i in range(int(self.length[1])):
            for j in range(int(self.length[0])):
                if self.coordinate[i][j] == "0":
                    aaa = 5
                    bbb = aaa + 1
                    ccc = aaa
                    ddd = aaa - 1
                    if not i > aaa: aaa = i
                    self.coordinate[i-aaa:i+bbb,j-ccc:j+ddd] = np.where(self.coordinate[i-aaa:i+bbb,j-ccc:j+ddd] == "0","0","100")

    def callback2(self, msg): self.z = msg.magnetic_field.z * -100000 
    def tf_change(self):
        try:
            if self.tf.frameExists("/map") and self.tf.frameExists("/base_footprint"):
                t = self.tf.getLatestCommonTime("/map", "/base_footprint")
                position, quaternion = self.tf.lookupTransform("/map", "/base_footprint", t)
                self.x, self.y = position[0], position[1]
                self.point2.x = position[0]
                self.point2.y = position[1]
                self.pub4.publish(self.point2)
                #print self.x,self.y

        except:
            print "勃ち待ち"

    def mark(self):
        #格子のidの計
        for i in range (self.X_KOUSHI):
            for j in range (self.Y_KOUSHI):
                if 0 < self.x - (self.X_MIN + self.X_MATH * i) < self.X_MATH and\
                   0 < self.y - (self.Y_MIN + self.Y_MATH * j) < self.Y_MATH:
                    self.id = self.id_list[j][i]
        #markerの設定
        marker = Marker()
        marker.id = self.id
        marker.header.frame_id = 'map'
        marker.type = 3
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = self.z
        marker.color.a = 0.6
        marker.color.r = 1.0
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z / 2.0 
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration()
        self.marker_array.markers.append(marker)
        self.pub2.publish(self.marker_array)


    def plan(self):
        #周りの判別
        if self.y_plan < int(self.length[1])-1 and  not self.coordinate[self.y_plan+1][self.x_plan] == "100": self.south = True
        else: self.south = False
        if self.y_plan > 1 and  not self.coordinate[self.y_plan-1][self.x_plan] == "100": self.north = True
        else: self.north = False
        if self.x_plan < int(self.length[0])-1 and  not self.coordinate[self.y_plan][self.x_plan+1] == "100": self.east  = True
        else: self.east = False
        if self.x_plan > 1 and not self.coordinate[self.y_plan][self.x_plan-1] == "100": self.west  = True
        else: self.west = False


        #通ったところを壁にする
        self.coordinate[self.y_plan][self.x_plan] = "100"
        #print self.south, self.north, self.east, self.west
        if self.x_plan < (float(self.length[0]) / 2.0):
            if self.west:   
                self.x_plan-=1
                self.kazu.append(180)
            elif self.south:
                self.y_plan+=1
                self.kazu.append(270)
            elif self.north: 
                self.y_plan-=1
                self.kazu.append(90)
            elif self.east:  
                self.x_plan+=1
                self.kazu.append(0)
            else: self.kazu.append(100)
        else:
            if self.east:    
                self.x_plan+=1
                self.kazu.append(0)
            elif self.south: 
                self.y_plan+=1
                self.kazu.append(270)
            elif self.north:
                self.y_plan-=1
                self.kazu.append(90)
            elif self.west: 
                self.x_plan-=1
                self.kazu.append(180)
            else: self.kazu.append(100)


        self.point.x = self.x_plan * 0.05
        self.point.y = (float(self.length[1])  - self.y_plan ) * 0.05
        self.point.z = self.kazu[1]

        if not self.kazu[1] == self.kazu[0] :
            self.count +=1
        self.kazu.pop(0)
        if self.count % 6 == 0:
            self.pub3.publish(self.point)
            print self.point.x,self.point.y
        #print float(self.length[1]), self.y_plan


    def run(self):
        while not rospy.is_shutdown():
            self.tf_change()
            self.plan()
            self.mark()
            rospy.sleep(0.001)

if __name__ == '__main__':
    hoge = aho()
    try:
        hoge.run()
    except rospy.ROSInterruptException:
        pass
