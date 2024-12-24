#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError      # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                          # OpenCV 라이브러리
import numpy as np
from google.cloud import vision


from indy_driver.msg import robot_state, hamburger_info


class HamClassifierNode():
    def __init__(self):
        rospy.init_node('ham_classifier', anonymous=True) # 노드 초기화 및 이름 설정
        self.bridge = CvBridge()

        
        self.bAction = False
        # Subscriber
        self.sub_msg = rospy.Subscriber("camera/image_raw", Image, self.action)  # camera/image_raw 토픽에서 Image 메시지 수신
        self.sub_robot = rospy.Subscriber("robot_state", robot_state, self.chk_robot)  # camera/image_raw 토픽에서 Image 메시지 수신
        
        # Publisher
        self.pub_ham_info = rospy.Publisher('ham_classifier/ham_info', hamburger_info, queue_size=10)

        # Message Control Variables
        self.bSend = False
        self.bAction = False
        self.ham_info = hamburger_info()
        self.thresh_cnt = 15
        self.chk_cnt = 0    
        self.ham_id_prev = 0
        self.ham_id_curr = 0
        

    def chk_robot(self,robot_state):
        if robot_state.move == 1:
            self.bAction = False
        else:
            self.bAction = True

    def action(self,data):
        if self.bAction:
            try:
                
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                #Google Vision API 클라이언트 생성
                client = vision.ImageAnnotatorClient()

                # 프레임을 바이너리 데이터로 변환
                _, encoded_image = cv2.imencode('.jpg', cv_image)
                content = encoded_image.tobytes()

                # Vision API에 이미지 전송 및 텍스트 인식
                image = vision.Image(content=content)
                response = client.text_detection(image=image)
                label = response.text_annotations

                if label:
                    label = label[0].description
                else:
                    label = "No text detection"
                 
                if label == "Cheese Burger":
                    self.ham_id_curr = 1      
                elif label == "Bulgogi Burger":
                    self.ham_id_curr = 2
                elif label == "Shrimp Burger":
                    self.ham_id_curr = 3
                else:
                    self.ham_id_curr = 0

                if (self.ham_id_curr == self.ham_id_prev) and self.ham_id_curr != 0:
                    self.chk_cnt += 1
                else:
                    self.chk_cnt = 0

                if self.chk_cnt >= self.thresh_cnt:
                    self.chk_cnt = 0
                    self.bSend = True

                if self.bSend:
                    self.ham_info.name = label
                    self.pub_ham_info.publish(self.ham_info)
                    self.bSend = False

                self.ham_id_prev = self.ham_id_curr

                rospy.loginfo("%s, %d, %d",label, self.ham_id_curr, self.chk_cnt)


                # cv2.imshow("Camera", cv_image)  # 변환된 이미지를 "Camera"라는 이름의 윈도우에 표시
                # cv2.waitKey(1)                  # 1ms 동안 키보드 입력 대기

            except Exception as e:
                print(e)

    def run(self):
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행


if __name__ == '__main__':
    try:
        ham_classifier = HamClassifierNode()       # CameraNode 객체 생성
        ham_classifier.run()                # run 메서드 실행
    except rospy.ROSInterruptException:
        pass