#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image                   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError       # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2                                          # OpenCV 라이브러리

class DisplayNode:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw2",Image,self.callback)  # camera/image_raw 토픽에서 Image 메시지 수신
        
    def callback(self,data):
        try:
            # 수신된 Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Camera", cv_image)  # 변환된 이미지를 "Camera"라는 이름의 윈도우에 표시
        cv2.waitKey(1)                  # 1ms 동안 키보드 입력 대기

    def run(self):
        rospy.init_node('display_node', anonymous=True) # 노드 초기화 및 이름 설정
        rospy.spin()                                    # 노드가 종료될 때까지 계속 실행

if __name__ == '__main__':
    try:
        display = DisplayNode()     # DisplayNode 클래스의 인스턴스 생성
        display.run()               # 노드 실행
    except rospy.ROSInterruptException:
        pass