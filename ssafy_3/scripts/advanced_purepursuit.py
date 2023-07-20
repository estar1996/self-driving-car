#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, ObjectStatusList, Lamps, EventInfo, GetTrafficLightStatus
from morai_msgs.srv import MoraiEventCmdSrv
from std_msgs.msg import Bool, Int8, String
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 상태 정보 Odometry 를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서
# 0. 필수 학습 지식
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish

# TODO: (0) 필수 학습 지식
'''
# advanced_purepursuit 은 Pure Pursuit 알고리즘을 강화 한 예제입니다.
# 이전까지 사용한 Pure Pursuit 알고리즘은 고정된 전방주시거리(Look Forward Distance) 값을 사용하였습니다.
# 해당 예제에서는 전방주시거리(Look Forward Distance) 값을 주행 속도에 비례한 값으로 설정합니다.
# 이때 최소 최대 전방주시거리(Look Forward Distance) 를 설정합니다.
# 주행 속도에 비례한 값으로 변경 한 뒤 "self.lfd_gain" 을 변경 하여서 직접 제어기 성능을 튜닝 해보세요.
# 

'''


class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber('lattice_path', Path, self.lattice_path_callback)
        rospy.Subscriber('lane_path', Path, self.lane_path_callback)
        rospy.Subscriber('is_stop', Bool, self.is_stop_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber('/Object_topic', ObjectStatusList, self.object_info_callback)
        rospy.Subscriber('/is_lattice', Int8, self.is_lattice_callback)

        # rospy.Subscriber('GetTrafficLightStatus', GetTrafficLightStatus, self.traffic_callback)
        self.lamp_cmd = Lamps()
        self.lamp_cmd.turnSignal = 0  # 0: no signal, 1: left signal, 2: right signal, 3: 이전 상태 유지
        self.lamp_cmd.emergencySignal = 0  # 0:no signal, 1: emergency signal

        set_Event_control = EventInfo()
        set_Event_control.option = 7  # 1: ctrl_mode, 2: gear, 4:lamps, 8: set_pause
        set_Event_control.ctrl_mode = 1  # 1: keyboard, 2:gamewheel, 3:automode, 4:cruisemode
        set_Event_control.gear = 1  # 1:P, 2:R, 3:N, 4:D
        set_Event_control.lamps = self.lamp_cmd

        ros_srv = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)
        result = ros_srv(set_Event_control)

        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.dir_pub = rospy.Publisher('dir', Int8, queue_size=1)
        self.mode_pub = rospy.Publisher('mode', String, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_lane_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False
        self.is_stop = False
        self.is_lattice = False
        self.mode = 'GPS'

        # self.forward_point = Point()
        self.forward_point = []
        self.current_position = Point()
        # self.far_forward_point = Point()
        self.far_forward_point = []

        self.vehicle_length = 2.6
        self.lfd = 8
        self.min_lfd = 5
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 55

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)
        self.adaptive_cruise_control = AdaptiveCruiseControl(velocity_gain=0.5, distance_gain=1, time_gap=0.8,
                                                             vehicle_length=2.7)

        rospy.wait_for_service('/Service_MoraiEventCmd')
        while True:
            if self.is_global_path == True:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                set_Event_control.option = 3  # 1: ctrl_mode, 2: gear, 4:lamps, 8: set_pause
                set_Event_control.ctrl_mode = 3  # 1: keyboard, 2:gamewheel, 3:automode, 4:cruisemode
                set_Event_control.gear = 4  # 1:P, 2:R, 3:N, 4:D
                result = ros_srv(set_Event_control)
                break
            else:
                rospy.loginfo('Waiting global path data')

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            print(self.is_path, self.is_odom, self.is_status)
            if self.is_path == True and self.is_odom == True and self.is_status == True:
                result = self.calc_valid_obj([self.current_position.x, self.current_position.y, self.vehicle_yaw],
                                             self.object_data)

                global_npc_info = result[0]
                local_npc_info = result[1]
                global_ped_info = result[2]
                local_ped_info = result[3]

                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6
                # print(self.target_velocity)

                self.mode = 'GPS'
                if self.current_position.x == 0 and self.current_position.y == 0 and self.is_lane_path:
                    print('lane')
                    self.path = self.lane_path
                    self.mode = 'LANE_WITH_CAMERA'

                if self.is_lattice:
                    self.mode = "OBSTACLE"

                steering, dir = self.calc_pure_pursuit()
                # print(steering, dir)

                if dir > 0.02:
                    self.dir_pub.publish(-1)
                elif dir < -0.02:
                    self.dir_pub.publish(1)
                else:
                    self.dir_pub.publish(0)


                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else:
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                self.adaptive_cruise_control.check_object(self.path, global_npc_info, local_npc_info
                                                          , global_ped_info, local_ped_info)

                self.target_velocity_cruise = self.adaptive_cruise_control.get_target_velocity(local_npc_info,
                                                                                               local_ped_info,
                                                                                               self.status_msg.velocity.x,
                                                                                               self.target_velocity / 3.6)

                self.target_velocity = min(self.target_velocity, self.target_velocity_cruise)
                if self.current_position.x == 0 and self.current_position.y == 0 and self.is_lane_path:
                    self.target_velocity -= 5
                # print(self.target_velocity)
                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)
                # print(output)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                if self.is_stop is True:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                self.mode_pub.publish(self.mode)

            rate.sleep()

    def lattice_path_callback(self, msg):
        self.is_path = True
        # self.lattice_path = msg
        self.path = msg

    # def local_path_callback(self, msg):
    #     self.is_path = True
    #     self.local_path = msg

    def lane_path_callback(self, msg):
        self.is_lane_path = True
        self.lane_path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def status_callback(self, msg):  ## Vehicl Status Subscriber
        self.is_status = True
        self.status_msg = msg

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def object_info_callback(self, data):  ## Object information Subscriber
        self.is_object_info = True
        self.object_data = data

    def is_stop_callback(self, msg):
        self.is_stop = msg.data

    def is_lattice_callback(self, msg):
        self.is_lattice = msg.data

    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_pure_pursuit(self, ):
        self.lfd = self.status_msg.velocity.x * self.lfd_gain

        if self.lfd < self.min_lfd:
            self.lfd = self.min_lfd
        elif self.lfd > self.max_lfd:
            self.lfd = self.max_lfd

        vehicle_position = self.current_position
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        trans_matrix = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])

        det_trans_matrix = np.linalg.inv(trans_matrix)
        local_path_point = None

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)
            # print (self.vehicle_yaw)
            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd and self.is_look_forward_point is False:
                    self.forward_point = local_path_point
                    self.is_look_forward_point = True

                if dis >= 20:
                    self.far_forward_point = local_path_point
                    break

        theta = atan2(self.forward_point[1], self.forward_point[0])
        y = self.lfd
        x = 2 * self.vehicle_length * sin(theta)
        steering = atan2(x, y)

        dir = 0
        if len(self.far_forward_point) > 0:
            theta_dir = atan2(self.far_forward_point[1], self.far_forward_point[0])
            y = self.max_lfd
            x = 2 * self.vehicle_length * sin(theta_dir)
            dir = atan2(x, y)

        return steering, dir

    def calc_valid_obj(self, status_msg, object_data):

        self.all_object = object_data
        ego_pose_x = status_msg[0]
        ego_pose_y = status_msg[1]
        ego_heading = status_msg[2]

        global_npc_info = []
        local_npc_info = []
        global_ped_info = []
        local_ped_info = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian
        if num_of_object > 0:

            # translation
            tmp_theta = ego_heading
            tmp_translation = [ego_pose_x, ego_pose_y]
            tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                              [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]],
                              [0, 0, 1]])
            tmp_det_t = np.array(
                [[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0] * tmp_translation[1])],
                 [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1] * tmp_translation[1])],
                 [0, 0, 1]])

            # npc vehicle translation
            for npc_list in self.all_object.npc_list:
                # print(npc_list)
                if npc_list.velocity.x <= 1:
                    continue

                # print(npc_list.unique_id)

                global_result = np.array([[npc_list.position.x], [npc_list.position.y], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0:
                    global_npc_info.append(
                        [npc_list.type, npc_list.position.x, npc_list.position.y, npc_list.velocity.x])
                    local_npc_info.append([npc_list.type, local_result[0][0], local_result[1][0], npc_list.velocity.x])

            # ped translation
            for ped_list in self.all_object.pedestrian_list:
                # print(ped_list)
                if ped_list.velocity.x <= 1:
                    continue
                # print(ped_list.unique_id)

                global_result = np.array([[ped_list.position.x], [ped_list.position.y], [1]])
                local_result = tmp_det_t.dot(global_result)
                if local_result[0][0] > 0:
                    global_ped_info.append(
                        [ped_list.type, ped_list.position.x, ped_list.position.y, ped_list.velocity.x])
                    local_ped_info.append([ped_list.type, local_result[0][0], local_result[1][0], ped_list.velocity.x])

        return global_npc_info, local_npc_info, global_ped_info, local_ped_info


class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        p_control = self.p_gain * error
        # self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + d_control
        output = p_control
        self.prev_error = error

        return output


class velocityPlanning:
    def __init__(self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        is_child_area = False

        for i in range(point_num, len(global_path.poses) - point_num):
            x_list = []
            y_list = []
            cur_x = global_path.poses[i].pose.position.x
            cur_y = global_path.poses[i].pose.position.y

            child_area_x1 = 94
            child_area_y1 = 1143

            child_area_x2 = 129
            child_area_y2 = 1206

            dist1 = sqrt(pow(cur_x - child_area_x1, 2) + pow(cur_y - child_area_y1, 2))
            dist2 = sqrt(pow(cur_x - child_area_x2, 2) + pow(cur_y - child_area_y2, 2))

            if dist1 < 10 and is_child_area is False:
                is_child_area = True

            if dist2 < 10 and is_child_area is True:
                is_child_area = False

            if is_child_area is True:
                # print('is_child')
                v_max = 30 / 3.6

            else:
                for box in range(-point_num, point_num):
                    x = global_path.poses[i + box].pose.position.x
                    y = global_path.poses[i + box].pose.position.y
                    x_list.append([-2 * x, -2 * y, 1])
                    y_list.append((-x * x) - (y * y))

                x_array = np.array(x_list)
                y_array = np.array(y_list)

                x_trans = x_array.transpose()

                xtrans_x_inv = np.linalg.inv(x_trans.dot(x_array))
                tmp_array = xtrans_x_inv.dot(x_trans)
                result_array = tmp_array.dot(y_array)

                r = sqrt(pow(result_array[0], 2) + pow(result_array[1], 2) - result_array[2])
                # print(r)

                v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)

        # rospy.loginfo('')
        rospy.loginfo('velocity complete')
        return out_vel_plan


class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.npc_vehicle = [False, 0]
        self.object = [False, 0]
        self.Person = [False, 0]
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length

        self.object_type = None
        self.object_distance = 0
        self.object_velocity = 0

    def check_object(self, ref_path, global_npc_info, local_npc_info,
                     global_ped_info, local_ped_info):
        # TODO: (8) 경로상의 장애물 유무 확인 (차량, 사람, 정지선 신호)
        '''
        # 주행 경로 상의 장애물의 유무를 파악합니다.
        # 장애물이 한개 이상 있다면 self.object 변수의 첫번째 값을 True 로 둡니다.
        # 장애물의 대한 정보는 List 형식으로 self.object 변수의 두번째 값으로 둡니다.
        # 장애물의 유무 판단은 주행 할 경로에서 얼마나 떨어져 있는지를 보고 판단 합니다.
        # 아래 예제는 주행 경로에서 Object 까지의 거리를 파악하여
        # 경로를 기준으로 2.5 m 안쪽에 있다면 주행 경로 내 장애물이 있다고 판단 합니다.
        # 주행 경로 상 장애물이 여러게 있는 경우 가장 가까이 있는 장애물 정보를 가지도록 합니다.

        '''

        min_rel_distance = float('inf')
        self.npc_vehicle[0] = False

        if len(global_ped_info) > 0:
            for i in range(len(global_ped_info)):
                for path in ref_path.poses:
                    if global_ped_info[i][0] == 0:  # type=0 [pedestrian]
                        dis = sqrt(pow(path.pose.position.x - global_ped_info[i][1], 2) + pow(
                            path.pose.position.y - global_ped_info[i][2], 2))

                        if dis < 5:
                            rel_distance = dis
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.Person = [True, i]

        # 주행 경로 상 NPC 차량 유무 파악
        if len(global_npc_info) > 0:
            for i in range(len(global_npc_info)):
                # print("ASD")
                # print(global_npc_info)
                # print('\n')
                for path in ref_path.poses:
                    # if global_npc_info[i][0] == -1:  # type=-1 [ego_vehicle]
                    if global_npc_info[i][0] == 1:  # type=1 [npc_vehicle]
                        dis = sqrt(pow(path.pose.position.x - global_npc_info[i][1], 2) + pow(
                            path.pose.position.y - global_npc_info[i][2], 2))

                        if dis < 2.35:
                            rel_distance = dis
                            if rel_distance < min_rel_distance:
                                min_rel_distance = rel_distance
                                self.npc_vehicle = [True, i]

    def get_target_velocity(self, local_npc_info, local_ped_info, ego_vel, target_vel):
        # TODO: (9) 장애물과의 속도와 거리 차이를 이용하여 ACC 를 진행 목표 속도를 설정
        out_vel = target_vel
        default_space = 7.5
        time_gap = self.time_gap
        v_gain = self.velocity_gain
        x_errgain = self.distance_gain

        if self.npc_vehicle[0] and len(local_npc_info) != 0:  # ACC ON_vehicle
            # print("ACC ON NPC_Vehicle")
            # print(local_npc_info)
            # print(self.npc_vehicle)
            # print('===')
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2],
                             local_npc_info[self.npc_vehicle[1]][3]]

            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0], 2) + pow(front_vehicle[1], 2))
            vel_rel = ((front_vehicle[2] / 3.6) - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration

        if self.Person[0] and len(local_ped_info) != 0:  # ACC ON_Pedestrian
            print("ACC ON Pedestrian")
            Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2],
                          local_ped_info[self.Person[1]][3]]

            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(Pedestrian[0], 2) + pow(Pedestrian[1], 2))
            vel_rel = (Pedestrian[2] - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration

        return out_vel * 3.6


if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
