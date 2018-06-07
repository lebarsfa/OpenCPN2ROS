#!/usr/bin/env python
# coding=utf-8

# Author : Maël LE GALLIC
# Date : 06/06/2018

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion, TwistStamped
from math import floor
from nmea_msgs.msg import Sentence
import tf
from numpy import rad2deg


def quaternionToQuaternionMsg(quat):
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]
    return q

def quaternionMsgToQuaternion(quat):
    return ( quat.x, quat.y, quat.z, quat.w )

def getYaw(pose):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternionMsgToQuaternion(pose.pose.orientation))
    return yaw

def nmeaToDeg(nmeaNro):
    return floor(nmeaNro/100.) + (nmeaNro/100.-floor(nmeaNro/100.))*100/60

def degToNmea(deg):
    deg = abs(deg)
    return floor(deg)*100. + (deg - floor(deg))*60

def latH(lat):
    if lat<0:
        return "S"
    return "N"

def lonH(lon):
    if lon<0:
        return "W"
    return "E"

def checksum(msg):
    return str(hex(reduce((lambda a,b : a^b), map(ord, msg))))[2:]

def secsToNmeaTime(secs):
    hours = floor(secs/3600)
    secs -= hours*3600
    hours = (hours/24. - floor(hours/24.))*24
    minutes = floor(secs/60)
    secs -= minutes*60
    return "{0:02}{1:02}{2:02}".format(int(hours), int(minutes), int(floor(secs)))

class ROS2OpenCPN:
    def __init__(self):
        rospy.Subscriber('fix', NavSatFix, self.cb_fix, queue_size=10000)
        rospy.Subscriber('boat_heading', Float64, self.cb_heading, queue_size=10000)
        self.pub_nmea = rospy.Publisher('nmea_sentence', Sentence, queue_size=10000)
        self.rate = rospy.Rate(rospy.get_param('~rate', 3))
        self.cur_fix = None
        self.cur_heading = None

    def cb_fix(self, fix_msg):
        self.cur_fix = fix_msg

    def cb_heading(self, heading_msg):
        self.cur_heading = heading_msg

    def sendGPGGA(self):
        if self.cur_fix == None:
            return
        timeInSecs = self.cur_fix.header.stamp.to_sec()
        time = secsToNmeaTime(timeInSecs)
        lat, lon, alt = self.cur_fix.latitude, self.cur_fix.longitude, self.cur_fix.altitude
        print "time: ", time, ", sec: ",timeInSecs
        msg_comp = "GPGGA," + time + "," + "{0:08.3f}".format(degToNmea(lat)) + "," + latH(lat) + "," + "{0:09.3f}".format(degToNmea(lon)) + "," + lonH(lon) + "," + "4,10,0," + str(alt) + ",M," + str(alt) + ",M,,"
        self.sendSentence(msg_comp)

    def sendGPHDT(self):
        if self.cur_heading == None:
            return
        # yaw = -rad2deg(getYaw(self.cur_pose))+90
        yaw = self.cur_heading
        msg_comp = "GPHDT," + str(yaw) + ",T"
        self.sendSentence(msg_comp)

        # msg_comp = "GPRMC," + time + ",A," + "{0:08.3f}".format(degToNmea(lat)) + "," + latH(lat) + "," + "{0:09.3f}".format(degToNmea(lon)) + "," + lonH(lon) + "," + "0," + str(yaw) + ",230518,003.1,W"
        # self.sendSentence(msg_comp)
        # msg_comp = "GPHDM," + str(yaw) + ",T"
        # self.sendSentence(msg_comp)
        # msg_comp = "GPHSC," + str(yaw) + ",T,"+ str(yaw) + ",M  "
        # self.sendSentence(msg_comp)
        # msg_comp = "HCHDG," + str(yaw) + ",,,7.1,W "
        # self.sendSentence(msg_comp)

    def sendSentence(self, msg):
        sentence = Sentence()
        sentence.header.stamp = rospy.Time.now()
        sentence.sentence = "$" + msg + "*" + checksum(msg)
        self.pub_nmea.publish(sentence)

    def run(self):
        while not rospy.is_shutdown():
            self.sendGPGGA()
            self.sendGPHDT()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ros2opencpn')
    ROS2OpenCPN().run()
