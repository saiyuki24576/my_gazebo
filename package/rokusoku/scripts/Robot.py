#!/usr/bin/env python
# coding: utf-8

import rospy
import roslib
import time
import random
import time
import sys
import os
import tf
import math

from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetJointProperties
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

"""
更新
self.powerをこちらのクラスの変数へ(使いがってが悪かったので)
pub_motor()を大幅改造 高速化
"""
"""
関数一覧
pub_motor():すべての関節を動かす すべての関節の角度(動かさなければそのままの同じ角度)が必要
set_start_self.power():ロボットの関節を初期状態へ戻す
reset_model():初期座標・初期状態へ戻す
get_pos():現在の位置を取得
get_joint_ang():現在のJointの角度を必要
get_center():重心を求める
"""
class SixlegController:
    #コンストラクタ
    def __init__(self, robotID=1, posofsetx=0, posofsety=0):
        #print robotID, "init"
        self.ID = str(robotID)
        self._link_num = 18
        self._joint_num = 18
        self.power = [0] * self._joint_num
        self.now_ang = [0] * self._joint_num
        #モーターが回転し終わったあとの遅延時間
        self._MOTER_DELAY = 0.1
        #ロボットの初期座標を補正
        self.offset_x = posofsetx
        self.offset_y = posofsety
        #モデルの変数の設定
        #MB=massBase MM=massSum
        self.center_x = 0
        self.center_y = 0
        self.center_z = 0
        #リンクの位置
        self.link_pos = [[0 for i in range(3)] for j in range(self._link_num)]
        #ロボットのベースの位置 リンクの位置と同じだがこれだけ特別
        self.base_x = 0
        self.base_y = 0
        self.base_z = 0
        #ロボットのベースの傾き
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.model_name = "rokusoku" + self.ID
        #link_nameの順番:base,1-1,1-2,1-3,2-1,・・・・とする
        self.link_name = [str(self.model_name)+"::base",str(self.model_name)+"::link1_1", str(self.model_name)+"::link1_2", str(self.model_name)+"::link1_3", str(self.model_name)+"::link2_1", str(self.model_name)+"::link2_2", str(self.model_name)+"::link2_3", str(self.model_name)+"::link3_1", str(self.model_name)+"::link3_2", str(self.model_name)+"::link3_3", str(self.model_name)+"::link4_1", str(self.model_name)+"::link4_2",  str(self.model_name)+"::link4_4", str(self.model_name)+"::link5_1", str(self.model_name)+"::link5_2", str(self.model_name)+"::link5_3", str(self.model_name)+"::link6_1", str(self.model_name)+"::link6_2", str(self.model_name)+"::link6_3"]
        #ros関連の変数の設定
        self.set = ModelState()
        self.link = LinkState()
        self.getlink= GetLinkState()
        self.jointcall = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        
        #publishなどの設定
        self.set_pub = rospy.Publisher('/gazebo/set_model_state',ModelState,queue_size = 10)
        self.posecall  = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        s1 = '/rokusoku'+self.ID+'/Joint'
        s2 = '_position_controller/command'
        self.pub_motor1_1 = rospy.Publisher(s1+'1_1'+s2,Float64,queue_size = 1)
        self.pub_motor1_2 = rospy.Publisher(s1+'1_2'+s2,Float64,queue_size = 1)
        self.pub_motor1_3 = rospy.Publisher(s1+'1_3'+s2,Float64,queue_size = 1)
        self.pub_motor2_1 = rospy.Publisher(s1+'2_1'+s2,Float64,queue_size = 1)
        self.pub_motor2_2 = rospy.Publisher(s1+'2_2'+s2,Float64,queue_size = 1)
        self.pub_motor2_3 = rospy.Publisher(s1+'2_3'+s2,Float64,queue_size = 1)
        self.pub_motor3_1 = rospy.Publisher(s1+'3_1'+s2,Float64,queue_size = 1)
        self.pub_motor3_2 = rospy.Publisher(s1+'3_2'+s2,Float64,queue_size = 1)
        self.pub_motor3_3 = rospy.Publisher(s1+'3_3'+s2,Float64,queue_size = 1)
        self.pub_motor4_1 = rospy.Publisher(s1+'4_1'+s2,Float64,queue_size = 1)
        self.pub_motor4_2 = rospy.Publisher(s1+'4_2'+s2,Float64,queue_size = 1)
        self.pub_motor4_3 = rospy.Publisher(s1+'4_3'+s2,Float64,queue_size = 1)
        self.pub_motor5_1 = rospy.Publisher(s1+'5_1'+s2,Float64,queue_size = 1)
        self.pub_motor5_2 = rospy.Publisher(s1+'5_2'+s2,Float64,queue_size = 1)
        self.pub_motor5_3 = rospy.Publisher(s1+'5_3'+s2,Float64,queue_size = 1)
        self.pub_motor6_1 = rospy.Publisher(s1+'6_1'+s2,Float64,queue_size = 1)
        self.pub_motor6_2 = rospy.Publisher(s1+'6_2'+s2,Float64,queue_size = 1)
        self.pub_motor6_3 = rospy.Publisher(s1+'6_3'+s2,Float64,queue_size = 1)
        self.pub = [self.pub_motor1_1, self.pub_motor1_2, self.pub_motor1_3, self.pub_motor2_1,
                    self.pub_motor2_2, self.pub_motor2_3, self.pub_motor3_1, self.pub_motor3_2,
                    self.pub_motor3_3, self.pub_motor4_1, self.pub_motor4_2, self.pub_motor4_3,
                    self.pub_motor5_1, self.pub_motor5_2, self.pub_motor5_3, self.pub_motor6_1,
                    self.pub_motor6_2, self.pub_motor6_3]

    def reset_model(self):
        #基準位置に持っていく
        self.set.model_name = self.model_name
        self.set.pose.position.x = 0.0 + self.offset_x
        self.set.pose.position.y = 0.0 + self.offset_y
        self.set.pose.position.z = 0.3
        self.set.pose.orientation.x = 0.0
        self.set.pose.orientation.y = 0.0
        self.set.pose.orientation.z = 0.0
        self.set.twist.linear.x = 0.0
        self.set.twist.linear.y = 0.0
        self.set.twist.linear.z = 0.0
        self.set.twist.angular.x = 0.0
        self.set.twist.angular.y = 0.0
        self.set.twist.angular.z = 0.0
        self.set.reference_frame = ''
        self.set_pub.publish(self.set)
        self.set_start_power()
        time.sleep(1)
        #ロボットを初期の状態に持っていく
        self.set_start_power()

    def pub_motor(self):
        #現在のself.power変数の値をpublish
        for i in range(self._joint_num):
            self.pub[i].publish(self.power[i])
        flag = 0
        t = 0
        loop_start = time.time()
        while(1):
            flag = 0
            self.get_joint_ang()
            for i in range(self._joint_num):
                #角度ごとに、ある一定の角度誤差以下ではなかったら終わってないフラグを立てる 
                if( abs((abs(self.power[i]) - abs(self.now_ang[i]))) > 0.01):
                    flag = 1
            if flag == 0:
                #ここで終了 ただしモーターが動き終わってからすぐ動くのは不自然なため、delayをつけた
                rospy.sleep(self._MOTER_DELAY)
                return
            loop_time = time.time() - loop_start
            #print(loop_time)
            if loop_time > 1.0:
                for i in range(self._joint_num):
                    self.pub[i].publish(self.power[i])
            if loop_time > 2.0:
                #2秒動かなかったら
                #print("moter error!")
                return

    def set_start_power(self):
        for i in range(18):
            if i % 3 == 0:
                self.power[i] = 0.0
            elif i % 3 == 1:
                self.power[i] = math.radians(20)
            elif i % 3 == 2:
                self.power[i] = math.radians(-90)
        self.pub_motor()

    def get_pos(self):
        #各Linkの位置を取得

        reference_frame = ''

        #baseだけ特別処理        
        base_pos_temp = self.posecall(self.link_name[0],reference_frame)
        self.base_x = base_pos_temp.link_state.pose.position.x
        self.base_y = base_pos_temp.link_state.pose.position.y
        self.base_z = base_pos_temp.link_state.pose.position.z
        x_ori  = base_pos_temp.link_state.pose.orientation.x        
        y_ori  = base_pos_temp.link_state.pose.orientation.y        
        z_ori  = base_pos_temp.link_state.pose.orientation.z        
        w_ori  = base_pos_temp.link_state.pose.orientation.w        
        #これでroll, pitch, yawの情報がもらえる
        t = tf.transformations.euler_from_quaternion((x_ori, y_ori, z_ori, w_ori))
        self.roll = t[0]
        self.pitch = t[1]
        self.yaw = t[2]

        link_pos_temp = [0]*18
        link_pos_temp[0] = self.posecall(self.link_name[1],reference_frame)
        link_pos_temp[1] = self.posecall(self.link_name[2],reference_frame)
        link_pos_temp[2] = self.posecall(self.link_name[3],reference_frame)
        link_pos_temp[3] = self.posecall(self.link_name[4],reference_frame)
        link_pos_temp[4] = self.posecall(self.link_name[5],reference_frame)
        link_pos_temp[5] = self.posecall(self.link_name[6],reference_frame)
        link_pos_temp[6] = self.posecall(self.link_name[7],reference_frame)
        link_pos_temp[7] = self.posecall(self.link_name[8],reference_frame)
        link_pos_temp[8] = self.posecall(self.link_name[9],reference_frame)
        link_pos_temp[9] = self.posecall(self.link_name[10],reference_frame)
        link_pos_temp[10] = self.posecall(self.link_name[11],reference_frame)
        link_pos_temp[11] = self.posecall(self.link_name[12],reference_frame)
        link_pos_temp[12] = self.posecall(self.link_name[13],reference_frame)
        link_pos_temp[13] = self.posecall(self.link_name[14],reference_frame)
        link_pos_temp[14] = self.posecall(self.link_name[15],reference_frame)
        link_pos_temp[15] = self.posecall(self.link_name[16],reference_frame)
        link_pos_temp[16] = self.posecall(self.link_name[17],reference_frame)
        link_pos_temp[17] = self.posecall(self.link_name[18],reference_frame)

        for i in range(18):
            for j in range(3):
                if j == 0: self.link_pos[i][j] = link_pos_temp[i].link_state.pose.position.x
                if j == 1: self.link_pos[i][j] = link_pos_temp[i].link_state.pose.position.y
                if j == 2: self.link_pos[i][j] = link_pos_temp[i].link_state.pose.position.z

        #print(self.linkcall1_1.link_state.pose.position.x)

    def get_joint_ang(self):
        #現在のモーター角度を取得
        #rosserviceの/gazebo/get_joint_propertiesを使用
        ang = []
        for i in range(6):
            for j in range(3):
                #Joint2_2など
                joint_name = "Joint" + str(i+1) + "_" + str(j+1)
                #print(joint_name)
                a = self.jointcall(joint_name).position
                a = list(a)
                ang.append(a[0])
        self.now_ang = ang
  
    def get_center(self):
        #重心を取得
        MB = 0.3
        ML1 = 0.06
        ML2 = 0.08
        ML3 = 0.12
        MS = MB + ML1*6 + ML2*6 + ML3*6
        
        self.get_pos()

        x_pos_sum = 0 + base_x
        for i in range(18):
            x_pos_sum += self.link_pos[i][0]
        y_pos_sum = 0 + base_y
        for i in range(18):
            y_pos_sum += self.link_pos[i][1]
        z_pos_sum = 0 + base_z
        for i in range(18):
            z_pos_sum += self.link_pos[i][2]

        self.center_x = x_pos_sum / MM
        self.center_y = y_pos_sum / MM 
        self.center_z = z_pos_sum / MM 

