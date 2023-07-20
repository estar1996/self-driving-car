#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from morai_msgs.msg import IntersectionControl, EgoVehicleStatus
from math import sqrt
from std_msgs.msg import Bool, Int8
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *


# mgeo_pub 은 Mgeo 데이터를 읽어온 뒤 도로 정보를 Point Cloud Data 로 변환하는 예제입니다.
# Point Cloud 형식으로 변환 후 Rviz 를 이용해 정밀도로지도 데이터를 시각화 할 수 있습니다.

# 노드 실행 순서
# 1. Mgeo data 읽어온 후 데이터 확인
# 2. Link 정보 Point Cloud 데이터로 변환
# 3. Node 정보 Point Cloud 데이터로 변환
# 4. 변환한 Link, Node 정보 Publish

class check_stop_line:
    def __init__(self):
        rospy.init_node('stop_line', anonymous=True)


        rospy.Subscriber('is_stop', Bool, self.is_stop_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.is_stop_pub = rospy.Publisher('is_stop', Bool, queue_size=1)
        self.insn_pub = rospy.Publisher('InsnControl', IntersectionControl, queue_size=1)

        self.is_insn_status = False
        self.is_status = False
        self.is_stop = False

        self.insn = IntersectionControl()

        self.insn8 = [104, 1160, 8]
        self.insn6 = [108, 1236, 6]
        self.insn5 = [136, 1351, 5]
        self.insn4 = [75, 1470, 4]
        self.insn1 = [118, 1608, 1]

        self.insns = [self.insn8, self.insn6, self.insn5, self.insn4, self.insn1]

        self.current_position = Point()


        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.cal_cur_insn()
            if self.is_stop:
                if self.insn.intersection_index == 8:
                    print('insn8')
                    rospy.sleep(3)
                    self.insn.intersection_status = 0
                    print(self.insn)
                elif self.insn.intersection_index == 6:
                    print('insn6')
                    rospy.sleep(3)
                    self.insn.intersection_status = 0
                elif self.insn.intersection_index == 5:
                    print('insn5')
                    rospy.sleep(3)
                    self.insn.intersection_status = 2
                elif self.insn.intersection_index == 4:
                    print('insn4')
                    rospy.sleep(3)
                    self.insn.intersection_status = 1
                elif self.insn.intersection_index == 1:
                    print('insn1')
                    rospy.sleep(3)
                    self.insn.intersection_status = 1
                self.insn_pub.publish(self.insn)
            rate.sleep()


    def is_stop_callback(self, msg):
        self.is_stop = msg.data

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
    def cal_cur_insn(self):
        min_dist = float('inf')
        for insn in self.insns:
            x = insn[0]
            y = insn[1]

            dist = sqrt(pow(x - self.current_position.x, 2) + pow(y - self.current_position.y, 2))
            if dist < min_dist:
                min_dist = dist
                self.insn.intersection_index = insn[2]



if __name__ == '__main__':
    test_track = check_stop_line()
