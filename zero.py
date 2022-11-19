#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import time
import collections
import numpy as np
import rospy
#import pigpio
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
import message_filters

fps = 100.
delay = 1/fps*0.5
autoware_ang = 0
imu_ang = []

Rmotor_ini = 1
Lmotor_ini = 1
T = 0.33

autoware_Lpower, autoware_Rpower = 0, 0
imu_Lpower, imu_Rpower = 0, 0
autoware_Lduty, autoware_Rduty = 0, 0
imu_Lduty, imu_Rduty = 0, 0

dst_min = 50.
dst_max = 250.
dst_gap = 10.
#最大測定距離、最低確保距離、分解能から出力の割合表を作成
elem = (dst_max-dst_min)/dst_gap
dst_ratio = [i/elem for i in range(int(elem+1))]
print(dst_ratio)

base_duty = 100

def culc_power(v, omg):
    # a = np.array([[1/2,1/2],[1/T,-1/T]])
    # b = np.array([[v],[omg]])
    # return np.linalg.solve(a,b)
    a = np.array([[0.5,0.5],[1/T,-1/T]])
    # print(a)
    inverse = np.linalg.pinv(a)
    # print(inverse)
    b = np.array([[v],[omg]])
    v = inverse.dot(b)
    # print(v)
    return v

def comp_zero(ang):
        return -ang*0.1

class Motor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.kill = False
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        self.Rduty = 0
        self.Lduty = 0
        self.type = 'None'

    def set_motor(self, motor):
        self.speed = motor[0]
        self.ang = motor[1]
        self.Rpower = motor[2]
        self.Lpower = motor[3]
        self.type = motor[4]

    def stop_motor(self):
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0

    def run(self):
        while not self.kill:
            if -1 < self.speed <= 0:
                print("Motor Stop")
                #pi.write(SWpin,0)
            elif self.speed == -1:
                print('back')
                #pi.write(DIRpin,0)
                #pi.write(SWpin,1)
                #pi.hardware_PWM(gpio_pinR, Freq, duty2per(50))
                #pi.hardware_PWM(gpio_pinL, Freq, duty2per(50))
            else:
                #print("Motor On")
                #pi.write(DIRpin,1)
                #pi.write(SWpin,1)
                self.Rduty = base_duty*Rmotor_ini*self.Rpower*dst_ratio[18]
                self.Lduty = base_duty*Lmotor_ini*self.Lpower*dst_ratio[18]
                #dst_ratio[us.get_level()]
                #print(us.get_level())
                if self.Rduty > 100:
                    #self.Lduty = self.Lduty - (self.Rduty - 100)
                    self.Rduty = 100
                elif self.Rduty < 0:
                    #self.Lduty = self.Lduty + (abs(self.Rduty))
                    self.Rduty = 0
                if self.Lduty > 100:
                    #self.Rduty = self.Rduty - (self.Lduty - 100)
                    self.Lduty = 100
                elif self.Lduty < 0:
                    #self.Rduty = self.Rduty + (abs(self.Lduty))
                    self.Lduty = 0
                
                #pi.hardware_PWM(gpio_pinR, Freq, duty2per(self.Rduty))
                #pi.hardware_PWM(gpio_pinL, Freq, duty2per(self.Lduty))
                print(self.type, self.Lduty, self.Rduty)
            time.sleep(0.1)
class Autoware:
    def __init__(self):
        self.twist = {}
        self.speed = 0
        self.ang = 0
        self.Rpower = 0
        self.Lpower = 0
        self.ang_imu = []
        rospy.init_node('Speed')  # , log_level=rospy.DEBUG
        rospy.on_shutdown(self.__on_shutdown)
        self.sub1 = message_filters.Subscriber('/twist_cmd', TwistStamped)
        self.sub2 = message_filters.Subscriber('/imu/data_raw', Imu)
        #self.subscriber = rospy.Subscriber('/twist_cmd', TwistStamped, self.__callback)
        ts = message_filters.ApproximateTimeSynchronizer([self.sub1,self.sub2], 10, delay)
        ts.registerCallback(self.callback)

    def callback(self, raw, data_raw):
        twist = {"speed": raw.twist.linear.x, "ang": raw.twist.angular.z}  # speed: m/s, angular: radian/s
        # angular: 右カーブ -> マイナス
        #          左カーブ -> プラス
        self.ang_imu = [data_raw.angular_velocity.x, data_raw.angular_velocity.y, data_raw.angular_velocity.z]
        rospy.logdebug("Autoware > %s" % self.twist)
        self.ang = twist["ang"]
        self.speed = twist["speed"]
        #power = culc_power(self.speed, self.ang)
        #self.Rpower = power[0][0]
        #self.Lpower = power[1][0]

    def __on_shutdown(self):
        rospy.loginfo("shutdown!")
        #self.subscriber.unregister()

    def get_twist(self):
	#print(self.speed, self.ang, self.Rpower, self.Lpower)
        return self.speed, self.ang, self.Rpower, self.Lpower

    def get_ang(self):
        return self.ang

    def get_speed(self):
        return self.speed

    def get_ang_imu(self):
        if len(self.ang_imu) == 3:
            return self.ang_imu[2]
        else:
            print(len(self.ang_imu))
            return 0


if __name__ == '__main__':
    imu_m = Motor()
    #autoware_m = Motor()
    imu_m.start()
    #autoware_m.start()
    try:
        a = Autoware()

        while not rospy.is_shutdown():  # rospy.spin()と同じ
            rospy.sleep(0.1)
            #autoware_ang = a.get_ang()
            imu_ang = a.get_ang_imu()
            #speed = a.get_speed()
            speed = 0.5
            imu_ang = comp_zero(imu_ang)

            #print('imu', imu_ang, 'autoware', autoware_ang)

            #autoware_power = culc_power(speed, autoware_ang)
            imu_power = culc_power(speed, imu_ang)

            #autoware_Rpower = autoware_power[0][0]
            #autoware_Lpower = autoware_power[1][0]
            imu_Rpower = imu_power[0][0]
            imu_Lpower = imu_power[1][0]

            #print('autoware power', autoware_Lpower, autoware_Rpower)
            #print('imu power', imu_Lpower, imu_Rpower)
            #print(speed)
            imu_motor = [speed, imu_ang, imu_Rpower, imu_Lpower, 'IMU     ']
            #autoware_motor = [speed, imu_ang, autoware_Rpower, autoware_Lpower, 'Autoware']
            imu_m.set_motor(imu_motor)
            #autoware_m.set_motor(autoware_motor)
            #print('imu duty', imu_Lduty, imu_Rduty)
            #print('autoware duty', autoware_Lduty, autoware_Rduty)

    except rospy.ROSInterruptException:
        pass