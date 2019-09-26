#!/usr/bin/env python
# coding: utf-8

from __future__ import print_function

import Robot
import numpy as np
import rospy
import roslib
import time
import math
import random
import sys
import os
import datetime

from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import tf

#クラスを継承している
class SixlegClass(Robot.SixlegController, object):
    def __init__(self, robotID, posofsetx, posofsety):
        super(SixlegClass, self).__init__(robotID, posofsetx, posofsety)
        #モーターのパワーを出力するもの　1-1:0 1-2:1 1-3:2 2-1:3・・・6-3:17となっている
        self.power = [0]*JOINT_NUM
        self.now_angle = 0.0

    def math_ang_power(self, dif_time):
        #時間　rosの時間を使用(遅延)予定
        #なんとかtなど
        #角速度[w/s]
        ang_vel = 1.0
        #角速度*差分時間で動く距離を算出
        ang_move = ang_vel * dif_time
        #動いた距離と、現在距離を加算
        self.now_angle += ang_move
        #これはsin波を生成して動かしているので、その制限を行う
        ang_limit = math.radians(40)
        ang = math.sin(self.now_angle) * ang_limit
        print(ang)
        for i in range(18):
            if i % 3 == 0:
                self.power[i] = ang
            elif i % 3 == 1:
                pass
                #self.power[i] =  math.radians(20)
            elif i % 3 == 2:
                pass
                #self.power[i] = math.radians(-90)
            #if i >= 9 and i % 3 == 0: self.power[i] += math.radians(180)
        #print(self.power)

def timer_callback(event):
    try:
        c.get_pos()
    except:
        print('service error')
    #print c.base_y


if __name__ == '__main__':
    #ノードの初期化と宣言
    rospy.init_node("robot1", anonymous=False)
    #一定時間ごとに実行される関数を生成　基本的にはセンサーの情報を取得のために使う
    rospy.Timer(rospy.Duration(0.2), timer_callback)
    while not rospy.is_shutdown():
        #クラスを宣言
        c = SixlegClass(1, 0, 0)
        #最初だけsleepをかけないとなぜか一部動かないものがある
        time.sleep(2.0)
        c.reset_model()
        c.reset_model()
        
        dif_time = 0.0
        while(1):
            s = time.time()
            c.math_ang_power(dif_time)
            c.pub_motor()
            dif_time = time.time() - s
