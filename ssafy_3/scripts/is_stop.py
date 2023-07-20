#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus
from math import sqrt
from std_msgs.msg import Bool, Int8

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

        self.is_stop_pub = rospy.Publisher('is_stop', Bool, queue_size=1)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.status_callback)
        rospy.Subscriber('GetTrafficLightStatus', GetTrafficLightStatus, self.traffic_callback)
        rospy.Subscriber('dir', Int8, self.dir_callback)

        self.is_traffic_sign = False
        self.is_stop = False
        self.dir = 0

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        light_set = mgeo_planner_map.light_set

        self.nodes = node_set.nodes
        self.lights = light_set.signals



        # print(self.lights['C119BS010046'].point[0])


        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # print(self.is_stop)
            if self.is_traffic_sign:
                self.cur_light = self.lights['{}'.format(self.traffic_sign.trafficLightIndex)]
                self.checkStop(self.status_msg)
            self.is_stop_pub.publish(self.is_stop)

            rate.sleep()

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

    def dir_callback(self, msg):
        self.dir = msg.data

    def checkStop(self, ego_status):
        min_dist = float('inf')
        min_dist2 = float('inf')
        min_dist_node_idx = -1
        min_dist_node_idx2 = -1

        for node_idx in self.nodes:
            dx = self.cur_light.point[0] - self.nodes[node_idx].point[0]
            dy = self.cur_light.point[1] - self.nodes[node_idx].point[1]

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if dist < min_dist:
                min_dist2 = min_dist
                min_dist_node_idx2 = min_dist_node_idx

                min_dist = dist
                min_dist_node_idx = node_idx

            elif dist < min_dist2:
                min_dist2 = dist
                min_dist_node_idx2 = node_idx


        is_stop_line = self.nodes[min_dist_node_idx2].on_stop_line
        print(self.nodes[min_dist_node_idx].to_dict()['idx'], self.nodes[min_dist_node_idx2].to_dict()['idx'])
        dx = ego_status.position.x - self.nodes[min_dist_node_idx2].point[0]
        dy = ego_status.position.y - self.nodes[min_dist_node_idx2].point[1]

        dist = sqrt(pow(dx, 2) + pow(dy, 2))
        print(self.traffic_sign.trafficLightStatus, self.dir, dist)

        if self.traffic_sign.trafficLightStatus & 16 and (self.dir == 0 or self.dir == 1):
            print('straight')
            self.is_stop = False
        elif self.traffic_sign.trafficLightStatus & 32 and self.dir == -1:
            print('left')
            self.is_stop = False
        else:
            if not self.traffic_sign:
                self.is_stop = False
            elif self.traffic_sign.trafficLightIndex == 'C119BS010075':
                if dist < 50:
                    print('stop 50')
                    self.is_stop = True
                else:
                    self.is_stop = False
            elif self.traffic_sign.trafficLightIndex == 'C119BS010072':
                if dist < 18:
                    print('stop 18')
                    self.is_stop = True
                else:
                    self.is_stop = False
            else:
                if dist < 10:
                    print('stop 10')
                    self.is_stop = True
                else:
                    self.is_stop = False


    def traffic_callback(self, msg):
        self.is_traffic_sign = True
        self.traffic_sign = msg

if __name__ == '__main__':
    test_track = check_stop_line()
