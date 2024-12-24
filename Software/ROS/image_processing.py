#!/usr/bin/env python3
# -*- coding:utf-8 -*- 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from indy_driver.msg import object_info, robot_state, hamburger_info

class ImageProcessingNode:

    def __init__(self):
        # CvBridge 객체 생성
        self.bridge = CvBridge()
        
        # 구독자들 설정
        self.sub_image = rospy.Subscriber("camera/image_raw2", Image, self.callback_image)
        self.sub_robot = rospy.Subscriber("robot_state", robot_state, self.chk_robot)
        self.sub_ham = rospy.Subscriber("ham_classifier/ham_info", hamburger_info, self.callback_ham)

        # 상태 변수들
        self.bAction = False  # 로봇 상태에 따른 동작 여부 판단
        self.hamburger_name = None  # 햄버거 정보 저장

        # Object 정보 퍼블리셔 설정
        self.pub_object_info = rospy.Publisher("image_processing/object_info", object_info, queue_size=10)
        self.msg_object_info = object_info()  # 퍼블리셔로 보낼 메시지 객체 생성

        # 객체 정보 배열 (배열로 저장하고 퍼블리시할 것)
        self.object_names = []
        self.x_coords = []
        self.y_coords = []
        self.z_coords = []

    def callback_ham(self, data):
        """햄버거 정보 콜백 함수"""
        # 수신한 햄버거 정보를 저장
        self.hamburger_name = data.name
        rospy.loginfo(f"Received hamburger info: {self.hamburger_name}")
        
        # 로봇이 햄버거를 찾을 준비가 되면, 동작 시작
        self.bAction = True

    def callback_image(self, data):
        """이미지 처리 콜백 함수"""
        if self.bAction:  # 로봇이 햄버거를 찾을 준비가 되었을 때만 동작
            try:
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
                frame = cv.resize(frame, (640, 480))  # 이미지 크기 조정

                roi_points = [(110, 205), (615, 205), (115, 345), (604, 335)]
                roi_points = np.array(roi_points, dtype=np.int32).reshape((-1, 1, 2))

                xmin = min([point[0][0] for point in roi_points])
                ymin = min([point[0][1] for point in roi_points])
                xmax = max([point[0][0] for point in roi_points])
                ymax = max([point[0][1] for point in roi_points])
                roi = frame[ymin:ymax, xmin:xmax]

                # HSV 색상 범위 정의 (Yellow, Brown, Dark Brown)
                h_min_y, h_max_y = 0, 34  # Yellow Hue 범위
                s_min_y, s_max_y = 0, 255
                v_min_y, v_max_y = 100, 240

                h_min_b, h_max_b = 0, 20  # Brown Hue 범위
                s_min_b, s_max_b = 0, 255
                v_min_b, v_max_b = 0, 76

                h_min_bb, h_max_bb = 0, 68  # Dark Brown Hue 범위
                s_min_bb, s_max_bb = 0, 255
                v_min_bb, v_max_bb = 0, 40

                # ROI 영역을 BGR에서 HSV 색공간으로 변환
                hsv_frame = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

                # 색상 범위에 맞는 마스크 생성
                mask_yellow = cv.inRange(hsv_frame, (h_min_y, s_min_y, v_min_y), (h_max_y, s_max_y, v_max_y))
                mask_brown = cv.inRange(hsv_frame, (h_min_b, s_min_b, v_min_b), (h_max_b, s_max_b, v_max_b))
                mask_dark_brown = cv.inRange(hsv_frame, (h_min_bb, s_min_bb, v_min_bb), (h_max_bb, s_max_bb, v_max_bb))

                contours_yellow = []
                contours_brown = []
                contours_dark_brown = []

                # 컨투어 검출
                contours_yellow, _ = cv.findContours(mask_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                contours_brown, _ = cv.findContours(mask_brown, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
                contours_dark_brown, _ = cv.findContours(mask_dark_brown, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                # Calculate centroids and classify burgers
                shrimp_burger_contours = []
                bulgogi_burger_contours = []
                cheese_burger_contours = []


                # # 이미지를 표시하여 디버깅에 활용 (선택적)
                # cv.drawContours(frame, contours_yellow, -1, (0, 0, 255), 3)  # Yellow contours
                # cv.drawContours(frame, contours_brown, -1, (0, 255, 0), 3)  # Brown contours
                # cv.drawContours(frame, contours_dark_brown, -1, (255, 0, 0), 3)  # Dark Brown contours

                # 각 햄버거에 대해 컨투어 처리 및 퍼블리시
                for contour in contours_yellow:
                    area = cv.contourArea(contour)
                    if 3800 < area < 8000:  # Filtering based on area
                        M = cv.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            cheese_burger_contours.append((contour, cx + xmin, cy + ymin))
                            #print(f"CheeseBurger at ({cx + xmin}, {cy + ymin})")

                for contour in contours_brown:
                    area = cv.contourArea(contour)
                    if 3800 < area < 8000:  # Filtering based on area
                        M = cv.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            shrimp_burger_contours.append((contour, cx + xmin, cy + ymin))
                            #print(f"Shrimp Burger at ({cx + xmin}, {cy + ymin})")

                for contour in contours_dark_brown:
                    area = cv.contourArea(contour)
                    if 3800 < area < 8000:  # Filtering based on area
                        M = cv.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            bulgogi_burger_contours.append((contour, cx + xmin, cy + ymin))
                            #print(f"Bulgogi Burger at ({cx + xmin}, {cy + ymin})")

                # Now remove shrimp burger contours too close to bulgogi burger
                for i, shrimp_coord in enumerate(shrimp_burger_contours[:]):  # Copy list for safe iteration
                    for bulgogi_coord in bulgogi_burger_contours:
                        if abs(shrimp_coord[1] - bulgogi_coord[1]) <= 5:  # Compare x-coordinates
                            # Remove the shrimp burger contour from the list
                            shrimp_burger_contours.pop(i)
                            break


                for contour, cx, cy in cheese_burger_contours:    
                    area = cv.contourArea(contour)
                    if 3800 < area < 8000:  # Filtering based on area
                        cv.drawContours(frame, [contour +[xmin, ymin]], -1, (255, 0, 0), 2)
                        M = cv.moments(contour)
                        if M["m00"] != 0:  # Moments 값이 유효할 때만 진행
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            print(f"acheese_burger_contours ({cx})")
                            if self.hamburger_name == "Cheese Burger":
                                self.object_names.append("Cheese Burger")
                                self.x_coords.append(cx)
                                self.y_coords.append(cy)
                                self.z_coords.append(0)

                for contour, cx, cy in shrimp_burger_contours:
                    area = cv.contourArea(contour)
                    if 3800 < area < 8000:  # Filtering based on area
                        cv.drawContours(frame, [contour + [xmin, ymin]], -1, (0, 255, 0), 2)  
                        M = cv.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            print(f"shrimp_burger_contours ({cx})")
                            if self.hamburger_name == "Shrimp Burger":
                                self.object_names.append("Shrimp Burger")
                                self.x_coords.append(cx)
                                self.y_coords.append(cy)
                                self.z_coords.append(0)

                for contour, cx, cy in bulgogi_burger_contours:
                    area = cv.contourArea(contour)
                    if 3800 < area < 8000:  # Filtering based on area
                        cv.drawContours(frame, [contour + [xmin, ymin] ], -1, (0, 0, 255), 2)          
                        M = cv.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            print(f"bulgogi_burger_contours ({cx})")
                            if self.hamburger_name == "Bulgogi Burger":
                                self.object_names.append("Bulgogi Burger")
                                self.x_coords.append(cx)
                                self.y_coords.append(cy)
                                self.z_coords.append(0) 

                # 퍼블리시할 메시지 구성
                if self.object_names:
                    self.msg_object_info.names = self.object_names[-1]  # 가장 최근의 햄버거 이름만 퍼블리시
                    self.msg_object_info.x_coords = self.x_coords[-1]  # 가장 최근의 x 좌표만 퍼블리시
                    self.msg_object_info.y_coords = self.y_coords[-1]  # 가장 최근의 y 좌표만 퍼블리시
                    self.msg_object_info.z_coords = self.z_coords[-1]  # 가장 최근의 z 좌표만 퍼블리시
                    self.pub_object_info.publish(self.msg_object_info)
                    rospy.loginfo(f"Published {self.hamburger_name} at ({self.msg_object_info.x_coords})")

                # 이미지 창 표시 (선택적)
                cv.imshow("Frame", frame)
                cv.imshow("roi", roi)
                cv.waitKey(1)

            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge Error: {e}")



    def chk_robot(self, msg):
    #"""로봇 상태 처리 콜백 함수"""
        if msg.move == 1:
            self.bAction = False
         # 로봇이 움직일 때 3초 후에 bAction을 False로 설정
            #rospy.Timer(rospy.Duration(1), lambda event: setattr(self, 'bAction', False), oneshot=True)

        else:
            self.bAction = True  # 로봇이 멈추면 bAction을 False로 설정

if __name__ == '__main__':
    rospy.init_node('image_processing_node')
    image_processing_node = ImageProcessingNode()
    rospy.spin()



# if __name__ == "__main__":
#     rospy.init_node('image_processing_node', anonymous=True)
#     node = ImageProcessingNode()
#     rospy.spin()
