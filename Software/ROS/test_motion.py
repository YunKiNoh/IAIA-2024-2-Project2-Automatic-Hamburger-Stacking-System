#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

from move_group_python_interface import MoveGroupPythonInterface
from math import tau
import rospy

# import serial
# import time

import sys
import moveit_commander
import moveit_msgs
import geometry_msgs

import tf
import numpy as np
from math import pi
from indy_driver.msg import hamburger_info, object_info

DEG2RAD = pi/180
RAD2DEG = 180/pi

# # 아두이노와 연결 설정
# arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)  # 포트 확인 필요
# time.sleep(2)  # 아두이노 초기화 대기

# def send_to_arduino(data):
#     """아두이노로 데이터 전송"""
#     arduino.write(data.encode())  # 데이터를 바이트로 변환해 전송
#     time.sleep(0.1)  # 전송 후 약간 대기

# def read_from_arduino():
#     """아두이노로부터 데이터 수신"""
#     if arduino.in_waiting > 0:  # 읽을 데이터가 있는지 확인
#         return arduino.readline().decode('utf-8').strip()  # 데이터를 읽고 디코딩
#     return None



# def main():
#     try:
#         indy10 = MoveGroupPythonInterface(real=True, gripper='Gripper')
        
        # we have to input in degree
        # tau is 6.28
        # 우리는 우선 3가지의 햄버거 만드는 것을 생각
        # 첫번째 불고기버거 (빵->그릇->양배추->그릇->불고기패티->그릇->토마토->그릇->빵->그릇->종누르기->스탠바이)
        # 두번째 새우버거 (빵->그릇->양배추->그릇->새우패티->그릇->토마토->그릇->빵->그릇->종누르기->스탠바이)
        # 세번째 치즈버거 (빵->그릇->양배추->그릇->치즈패티->그릇->토마토->그릇->빵->그릇->종누르기->스탠바이)



stack_info =      { "cv_top"   :      { "Bulgogi Burger": np.array([-90, -25, 115, 0, 0, 0])* DEG2RAD,
                                        "Cheese Burger"  : np.array([-90, -25, 115, 0, 0, 0])* DEG2RAD,   
                                        "Shrimp Burger" : np.array([-90, -25, 115, 0, 0, 0])* DEG2RAD},
                    "cv_bottom":      { "Bulgogi Burger": np.array([-90, -8, 130, 0, -30, 0])* DEG2RAD,
                                        "Cheese Burger"  : np.array([-90, -8, 130, 0, -30, 0])* DEG2RAD,   
                                        "Shrimp Burger" : np.array([-90, -8, 130, 0, -30, 0])* DEG2RAD},
                  
                    "plate_top"   :   { "Bulgogi Burger": np.array([0, -25, 115, 0, 0, 0])* DEG2RAD,
                                        "Cheese Burger"  : np.array([0, -25, 115, 0, 0, 0])* DEG2RAD,   
                                        "Shrimp Burger" : np.array([0, -25, 115, 0, 0, 0])* DEG2RAD},
                    "plate_angle":   { "Bulgogi Burger" : np.array([0, 0, 0, 0, 20, 0])* DEG2RAD,
                                        "Cheese Burger"  : np.array([0, 0, 0, 0, 20, 0])* DEG2RAD,   
                                        "Shrimp Burger" : np.array([0, 0, 0, 0, 20, 0])* DEG2RAD},

                    "patty_top"   :   { "Bulgogi Burger": np.array([90, -25, 115, 0, 0, 0])* DEG2RAD,
                                        "Cheese Burger"  : np.array([90, -25, 115, 0, 0, 0])* DEG2RAD,   
                                        "Shrimp Burger" : np.array([90, -25, 115, 0, 0, 0])* DEG2RAD},                    
                
                    "quantity":       { "Bulgogi Burger" : 1,
                                        "Cheese Burger"   : 1,
                                        "Shrimp Burger"  : 1},


                    "CV_front"    :    {    "rel_xyz"          : [0.13, 0, 0]     ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]      },
                    "CV_back"     :    {    "rel_xyz"          : [-0.13, 0, 0]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]      },
                    
                    #patty_down"    : {     "rel_xyz"           :  [x, y, -0.48] 
                    "patty_down"     : {     "down"              : np.array([90, 22.46, 142.4, 0, -76.8, 0])* DEG2RAD},
                    "patty_md_up"    : {    "rel_xyz"           : [0, 0, 0.05]     ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]      },

                    "patty_md_down"  : {    "rel_xyz"           : [0, 0, -0.05]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]      },

                    "patty_front"    : {    "rel_xyz"           : [-0.12, 0, 0]     ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]      },
                    "patty_back"     : {    "rel_xyz"           : [0.12, 0, 0]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]      },

                    "plate_bottom":   {  "bottom1":   np.array([0, -21.7, 129.7, 0, -18, 0])* DEG2RAD   , 
                                         "bottom2":   np.array([0, -22.35, 128.4, 0, -16, 0])* DEG2RAD  ,
                                         "bottom3":   np.array([0, -23, 127.1, 0, -14.1, 0])* DEG2RAD   ,
                                         "bottom4":   np.array([0, -23.5, 125.7, 0, -12.2, 0])* DEG2RAD ,
                                         "bottom5":   np.array([0, -23.9, 124.3, 0, -10.4, 0])* DEG2RAD },



                    "plate_front" :   {     "rel_xyz"           : [0, 0.12, 0]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]       },
                    "plate_back"  :   {     "rel_xyz"           : [0, -0.12, 0]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]       },
                    "plate_front_lettuce":{ "rel_xyz"           : [0, 0.125, 0]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]       },                    
                    "plate_back_lettuce" :{ "rel_xyz"           : [0, -0.125, 0]    ,
                                            "rel_rpy"           : [0.0, 0.0, 0.0]       },
                    
                    "bell"          :{ "up":     np.array([15, -3.15, 126.3, 0, -35.2, 0])* DEG2RAD   , 
                                       "down":   np.array([15, -2.64, 127.7, 0, -35.2, 0])* DEG2RAD  ,      }                                   
                    }

class HamFeederNode():
    def __init__(self):
        # Subscriber 설정
        self.sub_ham_class = rospy.Subscriber("ham_classifier/ham_info", hamburger_info, self.stack_ham)
        self.sub_object_info = rospy.Subscriber("image_processing/object_info", object_info, self.stack_object)

        # Robot 초기화
        self.indy10 = MoveGroupPythonInterface(real=True)
        iniplace = np.array([0, -25, 115, 0, 0, 0]) * DEG2RAD
        self.indy10.go_to_joint_abs(iniplace)

        # 저장 변수 초기화
        self.current_ham_info = None
        self.current_object_info = None

    def stack_ham(self, msg):
        # hamburger_info 메시지를 받은 경우
        self.current_ham_info = msg
        # object_info도 받았으면 stack 처리
        if self.current_object_info:
            self.process_stack()

    def stack_object(self, msg):
        # object_info 메시지를 받은 경우
        self.current_object_info = msg
        # hamburger_info도 받았으면 stack 처리
        if self.current_ham_info:
            self.process_stack()

    def process_stack(self):
        # 현재의 hamburger_info와 object_info를 처리
        hamburger_info = self.current_ham_info
        object_info = self.current_object_info

        # object_info를 이용한 x, y 좌표
        x = object_info.x_coords
        y = object_info.y_coords

        
        # bread (그릇 상단 -> 컨베이어 상단 -> 컨베이어 하단 -> 재료 앞으로 -> 재료 뒤로 -> 컨베이어 하단 -> 컨베이어 상단 -> 그릇 상단 -> 그릇 하단1 -> 그릇 앞으로 -> 그릇 각도 -> 그릇 뒤로))
        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])
    
        self.indy10.go_to_pose_rel(stack_info["CV_front"]['rel_xyz'], stack_info["CV_front"]['rel_rpy'])
        # self.indy10.go_to_pose_rel(stack_info["CV_back"]['rel_xyz'], stack_info["CV_back"]['rel_rpy'])
    
        #self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])

        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])

        # self.indy10.go_to_joint_abs(stack_info["plate_bottom"]['bottom1'])
        # self.indy10.go_to_pose_rel(stack_info["plate_front"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])

        # self.indy10.go_to_joint_rel(stack_info["plate_angle"][hamburger_info.name])
        # self.indy10.go_to_pose_rel(stack_info["plate_back"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])





        
        # lettuce (그릇 상단 -> 컨베이어 상단 -> 컨베이어 하단 -> 재료 앞으로 -> 재료 뒤로 -> 컨베이어 하단 -> 컨베이어 상단 -> 그릇 상단 -> 그릇 하단2 -> 그릇 앞으로(양상추만) -> 그릇 각도 -> 그릇 뒤로(양상추만)))
        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])          
    
        self.indy10.go_to_pose_rel(stack_info["CV_front"]['rel_xyz'], stack_info["CV_front"]['rel_rpy'])
        # self.indy10.go_to_pose_rel(stack_info["CV_back"]['rel_xyz'], stack_info["CV_back"]['rel_rpy'])       
    
        #self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])

        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])

        # self.indy10.go_to_joint_abs(stack_info["plate_bottom"]['bottom2'])
        # self.indy10.go_to_pose_rel(stack_info["plate_front_lettuce"]['rel_xyz'], stack_info["plate_front_lettuce"]['rel_rpy'])

        # self.indy10.go_to_joint_rel(stack_info["plate_angle"][hamburger_info.name])
        # self.indy10.go_to_pose_rel(stack_info["plate_back_lettuce"]['rel_xyz'], stack_info["plate_front_lettuce"]['rel_rpy'])

        
         
        
        # # patty (그릇 상단 -> 패티 상단 -> 패티 하단 -> 패티 앞으로 -> 패티 뒤로 -> 패티 하단 -> 패티 상단 -> 그릇 상단 -> 그릇 하단3 -> 그릇 앞으로 -> 그릇 각도 -> 그릇 뒤로))
        # self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])
        # self.indy10.go_to_joint_abs(stack_info["patty_top"][hamburger_info.name])

        # self.indy10.go_to_joint_abs(stack_info["patty_down"]['down'])
        # self.indy10.go_to_pose_rel(stack_info["patty_md_up"]['rel_xyz'], stack_info["patty_md_up"]['rel_rpy'])

        # self.indy10.go_to_pose_rel([0, ((x-85)/400)*0.53, 0], [0, 0, 0])
        # self.indy10.go_to_pose_rel(stack_info["patty_md_down"]['rel_xyz'], stack_info["patty_md_down"]['rel_rpy'])


        # self.indy10.go_to_pose_rel(stack_info["patty_front"]['rel_xyz'], stack_info["patty_front"]['rel_rpy'])
        # self.indy10.go_to_pose_rel(stack_info["patty_back"]['rel_xyz'], stack_info["patty_back"]['rel_rpy'])

        # self.indy10.go_to_joint_abs(stack_info["patty_top"][hamburger_info.name])
        
        # self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])

        # self.indy10.go_to_joint_abs(stack_info["plate_bottom"]['bottom3'])
        # self.indy10.go_to_pose_rel(stack_info["plate_front"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])

        # self.indy10.go_to_joint_rel(stack_info["plate_angle"][hamburger_info.name])
        # self.indy10.go_to_pose_rel(stack_info["plate_back"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])
        

        
        
        
        # tomato (그릇 상단 -> 컨베이어 상단 -> 컨베이어 하단 -> 재료 앞으로 -> 재료 뒤로 -> 컨베이어 하단 -> 컨베이어 상단 -> 그릇 상단 -> 그릇 하단4 -> 그릇 앞으로 -> 그릇 각도 -> 그릇 뒤로))
        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])

        self.indy10.go_to_pose_rel(stack_info["CV_front"]['rel_xyz'], stack_info["CV_front"]['rel_rpy'])
        # self.indy10.go_to_pose_rel(stack_info["CV_back"]['rel_xyz'], stack_info["CV_back"]['rel_rpy'])

        #self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])

        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])

        # self.indy10.go_to_joint_abs(stack_info["plate_bottom"]['bottom4'])
        # self.indy10.go_to_pose_rel(stack_info["plate_front"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])

        # self.indy10.go_to_joint_rel(stack_info["plate_angle"][hamburger_info.name])
        # self.indy10.go_to_pose_rel(stack_info["plate_back"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])

        
        
        
        
        
        # bread (그릇 상단 -> 컨베이어 상단 -> 컨베이어 하단 -> 재료 앞으로 -> 재료 뒤로 -> 컨베이어 하단 -> 컨베이어 상단 -> 그릇 상단 -> 그릇 하단5 -> 그릇 앞으로 -> 그릇 각도 -> 그릇 뒤로))
        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])

        self.indy10.go_to_pose_rel(stack_info["CV_front"]['rel_xyz'], stack_info["CV_front"]['rel_rpy'])
        # self.indy10.go_to_pose_rel(stack_info["CV_back"]['rel_xyz'], stack_info["CV_back"]['rel_rpy'])

        
    
        #self.indy10.go_to_joint_abs(stack_info["cv_bottom"][hamburger_info.name])
        self.indy10.go_to_joint_abs(stack_info["cv_top"][hamburger_info.name])

        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])

        # self.indy10.go_to_joint_abs(stack_info["plate_bottom"]['bottom5'])
        # self.indy10.go_to_pose_rel(stack_info["plate_front"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])

        # self.indy10.go_to_joint_rel(stack_info["plate_angle"][hamburger_info.name])
        # self.indy10.go_to_pose_rel(stack_info["plate_back"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])

        
        
        #ring the bell
        #self.indy10.go_to_joint_abs(stack_info["bell"]["bell_top"])
        #self.indy10.go_to_pose_rel(stack_info["bell_down"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])
        self.indy10.go_to_joint_abs(stack_info["bell"]["up"])
        self.indy10.go_to_joint_abs(stack_info["bell"]["down"])
        
        # self.indy10.go_to_pose_rel(stack_info["plate_back"]['rel_xyz'], stack_info["plate_front"]['rel_rpy'])


        self.indy10.go_to_joint_abs(stack_info["plate_top"][hamburger_info.name])
        
        self.current_ham_info = None
        self.current_object_info = None
        
    def run(self):
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행

if __name__ == '__main__':
    try:
        Ham_goooo = HamFeederNode()
        Ham_goooo.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass