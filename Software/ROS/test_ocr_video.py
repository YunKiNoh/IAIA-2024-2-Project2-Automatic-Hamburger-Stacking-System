#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import cv2
from google.cloud import vision
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class cameranode:

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)  # 노드 이름 "camera_node"로 초기화
        self.bridge = CvBridge()                # cv_bridge 객체 생성

        # Get camera number from ROS parameter, default to 0
        camera_number = rospy.get_param('~camera_number', 2)
        rospy.loginfo(f"Camera number received: {camera_number}")

        # "camera/image_raw"라는 토픽으로 메시지를 publish할 publisher 객체 생성
        self.image_pub = rospy.Publisher("camera/image_raw",Image,queue_size=1)    
        
        # self.cap = cv2.VideoCapture(camera_number)          # 카메라 연결을 위한 VideoCapture 객체 생성
        self.cap = cv2.VideoCapture(2)


    def main(self):
        # OpenCV를 통해 카메라 접근
        rate = rospy.Rate(30)                           # 루프 실행 주기 : 30hz
        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
            ret, frame = self.cap.read()                # 카메라로부터 이미지를 읽음
            if ret:                                     # 이미지가 정상적으로 읽혀진 경우
                try:
                    # 읽어들인 이미지를 ROS Image 메시지로 변환하여 토픽으로 publish
                    #self.detect_text_from_frame(frame)
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                
                except CvBridgeError as e:
                    print(e)                            # CvBridge 변환 예외 처리
            rate.sleep()                                # 지정된 루프 실행 주기에 따라 대기

if __name__ == "__main__":
    camera = cameranode()
    camera.main()
