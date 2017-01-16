#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import math 
import numpy as np
from geometry_msgs.msg import Point
if __name__ == '__main__':

    #pgm,yamlの読み込み-----------
    pgm,yaml = [],[]
    f = open('/home/masaya/map/19_floor_extended.pgm')
    pgm = f.readlines()
    for i in range(len(pgm)):
        pgm[i] = pgm[i].replace('\n',"")
    f.close()
    length = pgm[2].split(" ")
    f = open('/home/masaya/map/19_floor_extended.yaml')
    yaml = f.readlines()
    for i in range(len(yaml)):
        yaml[i] = yaml[i].replace('\n',"")
    f.close()
    #読み込み終了----------------
    deg = [0,0]
    point = Point()
    x_plan   = 1  #x初期位置
    y_plan   = 213#y初期位置
    xy = np.array([[pgm[i + j * int(length[0]) + 4 ] for i in range(int(length[0]))] for j in range(int(length[1]))])
    south, west, east, north = True, True, True, True
    #障害物の判定
    for i in range(int(length[1])):
        for j in range(int(length[0])):
            if xy[i][j] == "0":
                r = 5#行けない可能性の範囲
                x = r
                if not i > r: r = i
                xy[i-r:i+x+1,j-x:j+x+1] = np.where(xy[i-r:i+x+1,j-x:j+x+1] == "0","0","100")

    f = open('/home/masaya/catkin_ws/src/turtlebot_sweeping/map/map.txt','w')
    f.write(str(length[0])+","+str(length[1])+"\n")
    count = 0

    while(west or east or south or north):
        #周りの判別
        if y_plan < int(length[1])-1 and  not xy[y_plan+1][x_plan] == "100": south = True
        else: south = False
        if y_plan > 1 and  not xy[y_plan-1][x_plan] == "100": north = True
        else: north = False
        if x_plan < int(length[0])-1 and  not xy[y_plan][x_plan+1] == "100": east  = True
        else: east = False
        if x_plan > 1 and not xy[y_plan][x_plan-1] == "100": west  = True
        else: west = False
        #通ったところを障害物にする
        xy[y_plan][x_plan] = "100"

        if x_plan < (float(length[0]) / 2.0):
            if west: x_plan-=1;    deg.append(180)
            elif south: y_plan+=1; deg.append(90)
            elif north: y_plan-=1; deg.append(270)
            elif east: x_plan+=1;  deg.append(0)
        else:
            if east: x_plan+=1;    deg.append(0)
            elif south: y_plan+=1; deg.append(90)
            elif north: y_plan-=1; deg.append(270)
            elif west: x_plan-=1;  deg.append(180)
        point.x = x_plan * 0.05
        point.y = (float(length[1]) - y_plan ) * 0.05
        point.z = deg[1]
        if not deg[0] == deg[1]:
            count +=1
        if count %6 ==0:
            f.write(str(point.x)+","+str(point.y)+","+str(point.z)+"\n")

        deg.pop(0)
    f.close()
