#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
class mag(object):
    def __init__(self):
        rospy.init_node('sweep_nav')
        sub = rospy.Subscriber('/imu/mag', MagneticField, self.mag_data, queue_size = 10)
        rospy.loginfo("Waiting for the mag to come up")
        rospy.sleep(2)
        now = rospy.get_time()
        after = 0
        mag = []
        while not after - now >=10 and not rospy.is_shutdown():
            after = rospy.get_time()
            mag.append(self.mag)
            #print self.mag
        print "bias"
        print max(mag)-(max(mag)-min(mag))/2

    def mag_data(self, msg):
        self.mag = msg.magnetic_field.z# + 4.635 * 0.00001

if __name__ == '__main__':
    hoge = mag()
        
