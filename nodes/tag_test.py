#!/usr/bin/env python

import math
import rospy
import serial
import sys
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
import numpy as np
import kalman

NODE_NAME = "decawave_tag"
DIST_TOPIC = "range"


class DecaWaveTag:

    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baud', 115200)
        self.tag_names = rospy.get_param("tag_names")
        self.offsets = rospy.get_param("offsets")
        self.rate = rospy.Rate(rospy.get_param("frequency", 100))
        self.poseStamped = PoseStamped()
        self.kposeStamped = PoseStamped()
        self.ser = serial.Serial(port=port, timeout=None, baudrate=baud)
        self.pub = None
        self.offset = 0.0

    def start(self):
        self.ser.close()
        self.ser.open()
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            dist = self.get_dist()
            kdist = self.get_kdist()
            # print dist
            if dist is not None and self.pub is not None:
                position = self.trilaterate3D(dist)
                kposition = self.trilaterate3D(kdist)
                print(position)
                self.poseStamped.pose.position.x = position[0]
                self.poseStamped.pose.position.y = position[1]
                self.poseStamped.pose.position.z = position[2]
                self.kposeStamped.pose.position.x = kposition[0]
                self.kposeStamped.pose.position.y = kposition[1]
                self.kposeStamped.pose.position.z = kposition[2]
                # # if self.poseStamped.header.frame_id == "tag_right_front":
                # #     print dist
                self.pub.publish(self.poseStamped)
                self.pub.publish(self.kposeStamped)
            self.rate.sleep()
        self.ser.close()

    def get_dist(self):
        raw_data = self.ser.readline()
        data = raw_data.split()

        if self.pub is None:
            try:
                # tag_id = int(data[-1].split(":")[0][-1])
                # self.offset = float(self.offsets[tag_id])
                # self.poseStamped.header.frame_id = self.tag_names[tag_id]
                # topic_name = "/{}/{}".format(self.tag_names[tag_id], DIST_TOPIC)
                self.pub = rospy.Publisher("uwb_position", PoseStamped, queue_size=1)
            except IndexError:
                pass
        self.poseStamped.header.stamp = rospy.Time.now()

        if len(data) > 0 and data[0] == 'mc':
            mask = int(data[1], 16)
            if (mask & 0x01):
                global dist1, dist2, dist3
                dist1 = int(data[2], 16) 
                dist2 = int(data[3], 16)
                dist3 = int(data[4], 16) 
            return [dist1, dist2, dist3]
        else:
            return None
    def get_kdist(self):
        raw_data = self.ser.readline()
        data = raw_data.split()

        if self.pub is None:
            try:
                # tag_id = int(data[-1].split(":")[0][-1])
                # self.offset = float(self.offsets[tag_id])
                # self.poseStamped.header.frame_id = self.tag_names[tag_id]
                # topic_name = "/{}/{}".format(self.tag_names[tag_id], DIST_TOPIC)
                self.pub = rospy.Publisher("uwb_position", PoseStamped, queue_size=1)
            except IndexError:
                pass
        self.kposeStamped.header.stamp = rospy.Time.now()

        if len(data) > 0 and data[0] == 'mc':
            mask = int(data[1], 16)
            if (mask & 0x01):
                global kdist1, kdist2, kdist3
                kdist1 = kalman(int(data[2], 16) )
                kdist2 = kalman(int(data[3], 16) )
                kdist3 = kalman(int(data[4], 16) )
            return [kdist1, kdist2, kdist3]
        else:
            return None

    def trilaterate3D(self,distances):
        p1=np.array([0 ,0 ,0])
        p2=np.array([3610 ,0 ,0])
        p3=np.array([1800 ,8070 ,0])       

        r1=distances[0]
        r2=distances[1]
        r3=distances[2]

        e_x=(p2-p1)/np.linalg.norm(p2-p1)
        i=np.dot(e_x,(p3-p1))
        e_y=(p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
        e_z=np.cross(e_x,e_y)
        d=np.linalg.norm(p2-p1)
        j=np.dot(e_y,(p3-p1))
        x=((r1**2)-(r2**2)+(d**2))/(2*d)
        y=(((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
        z1=np.sqrt(r1**2-x**2-y**2)
        z2=np.sqrt((r1**2-x**2-y**2)*(-1))
        ans1=p1+(x*e_x)+(y*e_y)+(z1*e_z)
        ans2=p1+(x*e_x)+(y*e_y)+(z2*e_z)

        return ans1 if r1**2-x**2-y**2 > 0 else ans2
     


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    da = DecaWaveTag()
    da.start()
