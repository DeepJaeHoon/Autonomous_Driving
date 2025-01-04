#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj


class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False

        self.prev_x = None
        self.prev_y = None
        self.prev_time = None

        #변환 하고자 하는 좌표계를 선언
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        #송신 될 Odometry 메세지 변수 선언
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        # 30Hz로 msg 송신
        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertGPS2UTM()
                self.odom_pub.publish(self.odom_msg)
                rate.sleep()

    def navsat_callback(self, gps_msg):
        # GPS의 위도/경도 데이터 수신
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

    def convertGPS2UTM(self):    
        #위도 경도 데이터 UTM 죄표로 변환
        xy_zone = self.proj_UTM(self.lon, self.lat)

        self.x = xy_zone[0] #- self.e_o
        self.y = xy_zone[1] #- self.n_o

        #Odometry msg에 UTM 좌표 데이터 담기
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.

    def imu_callback(self, data):

        #Odometry msg에 IMU 데이터 담기
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w
        self.linear_acc_x = data.linear_acceleration.x
        self.linear_acc_y = data.linear_acceleration.y

        self.is_imu=True


if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass

