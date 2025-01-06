#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus
from std_msgs.msg import Float64MultiArray, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import cvxpy as cp
import math
import tf

class ACC :
    def __init__(self):
        rospy.init_node('acc', anonymous=True)

        # Odometry, Ego 차량 상태, 선행 차량 상태,  입력으로 받기
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("lead_vehicle", Float64MultiArray, self.lead_vehicle_callback)
        rospy.Subscriber('desired_velocity', Int32, self.velocity_callback)

        self.is_odom=False
        self.is_status = False
        self.is_lead_vehicle = False
        self.is_velocity = False
        self.current_postion = Point()

        # MPC 파라미터
        self.Q_d = 10.0  # Distance error weight
        self.Q_v = 1.0  # Velocity error weight
        self.R_a = 0.1  # Acceleration weight
        self.N = 20  # MPC prediction horizon

        # 제약 조건
        self.a_min = -5 # m/s^2
        self.a_max = 5
        self.v_min = 0
        self.v_max = 80/3.6 # m/s

        # ACC 파라미터
        self.d_safe = 20
        self.T_gap = 1/30
        self.TIME_STEP = 1/30

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_status ==True and self.is_odom==True and self.is_lead_vehicle == True and self.is_velocity == True:
                output = self.calc_acc(self.status_msg, self.lead_vehicle_msg, self.velocity_msg)
            else:
                print("Ego_status = {}, Ego_pose = {}, Lead_car_status = {}, velocity = {}".format(self.is_status, self.is_odom, self.is_lead_vehicle, self.is_velocity))
            rate.sleep()

    def odom_callback(self,msg): # Ego vehicle Odometry 입력 받기
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.ego_yaw=tf.transformations.euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): # Ego 차량 상태 받기(속도 받으려고)
        self.is_status = True
        self.status_msg = msg

    def lead_vehicle_callback(self, msg):
        self.is_lead_vehicle = True
        self.lead_vehicle_msg=msg.data

    def velocity_callback(self, msg):
        self.is_velocity = True
        self.velocity_msg = msg

    def trans_matrix(self, ego_x, ego_y, lead_x, lead_y, trigger):
        if trigger:
            trans_matrix = np.array([
            [math.cos(self.ego_yaw), -math.sin(self.ego_yaw), ego_x],
            [math.sin(self.ego_yaw),math.cos(self.ego_yaw), ego_y],
            [0                    ,0                    ,1            ]])
            det_trans_matrix = np.linalg.inv(trans_matrix)
            output = det_trans_matrix.dot([lead_x, lead_y, 1])
        else:
            trans_matrix = np.array([
            [math.cos(self.ego_yaw), -math.sin(self.ego_yaw)],
            [math.sin(self.ego_yaw),math.cos(self.ego_yaw)]
           ])
            det_trans_matrix = np.linalg.inv(trans_matrix)
            output = det_trans_matrix.dot([lead_x, lead_y])

        return output

    def calc_acc(self, ego_status, lead_car_status, desired_velocity): # 필요한 종방향 제어 명령 계산

        ego_position=self.current_postion
        x_ego = ego_position.x # UTM 좌표
        y_ego = ego_position.y # UTM 좌표
        vx_ego = ego_status.velocity.x # m/s 
        vy_ego = ego_status.velocity.y # m/s

        has_lead_car = True if len(lead_car_status) > 0 else False

        cost = 0.0
        constraints = []
        if has_lead_car:
            x_lead = lead_car_status[0] # UTM 좌표
            y_lead = lead_car_status[1] # UTM 좌표
            vx_lead = lead_car_status[2] # m/s
            vy_lead = lead_car_status[3] # m/s

            local_lead_pose = self.trans_matrix(x_ego, y_ego, x_lead, y_lead, True) # Lead 차량 좌표를 Ego 좌표계로 변환
            local_lead_vel = self.trans_matrix(None, None, vx_lead, vy_lead, False) # Lead 차량 속도를 Ego 좌표계로 변환
            d_curr = local_lead_pose[0]  # Ego와 Lead 사이 상대 거리 <Ego 좌표계로 표현>
            v_rel = local_lead_vel[0] - vx_ego # Ego와 Lead 사이 상대 속도 <Ego 좌표계로 표현>

            # 초기 상태 제약 조건
            constraints += [
                d_next[0] == d_curr + self.TIME_STEP * v_rel,
                v_next[0] == vx_ego + self.TIME_STEP * a[0]
            ]

        else:
            d_curr = None  # Lead 차량이 없을 경우
            v_rel = None  # Lead 차량이 없을 경우

            # 초기 상태 제약 조건
            constraints += [
                v_next[0] == vx_ego + self.TIME_STEP * a[0]
            ]

        # cvxpy로 MPC 최적화 설정하기 
        a = cp.Variable(self.N)  # 제어 입력 (EGO의 가속도) 초기화
        d_next = cp.Variable(self.N) if has_lead_car else None # 예측 상대 거리 초기화
        v_next = cp.Variable(self.N)  # 속도 초기화

        for i in range(self.N):
            # 비용 함수 정의
            if has_lead_car:
                # Lead 차량이 있을 경우
                cost += self.Q_d * cp.square(d_next[i] - (self.d_safe + self.T_gap * vx_ego))
            cost += self.Q_v * cp.square(v_next[i] - desired_velocity.data)
            cost += self.R_a * cp.square(a[i])

            # 제약 조건 정의
            constraints += [
                a[i] >= self.a_min,
                a[i] <= self.a_max,
                v_next[i] >= 0,
                v_next[i] <= self.v_max
            ]

            if has_lead_car:
                constraints += [
                    d_next[i] >= 0  # 상대 거리는 음수가 될 수 없음
                ]

            # 다음 상태 업데이트 제약 조건
            if i < self.N - 1:
                if has_lead_car:
                    constraints += [
                        d_next[i + 1] == d_next[i] + self.TIME_STEP * (local_lead_vel[0] - v_next[i]),
                        v_next[i + 1] == v_next[i] + self.TIME_STEP * a[i + 1]
                    ]
                else:
                    constraints += [
                        v_next[i + 1] == v_next[i] + self.TIME_STEP * a[i + 1]
                    ]

        # 최적화 풀기
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP)

        # 최적화 문제의 첫 step을 가속도 명령으로 사용
        a_ego = a.value[0] if a.value is not None else 0.0

        return a_ego

if __name__ == '__main__':
    try:
        longitudinal_control = ACC()
    except rospy.ROSInterruptException:
        pass
