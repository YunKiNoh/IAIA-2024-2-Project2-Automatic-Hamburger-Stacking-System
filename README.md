# Project2: Robot Operating System for Making Hamburger Model with Deep Learning

- Author: 21800226 Yunki Noh / 21900727 Garam Jin / 21900258 Hyeonho Moon 
- Date: 2024.12.20

본 레포지토리는 Industrial AI and Automation의 Project2[Automatic Hamburger Stacking System with Indy 10]의 수행을 돕기 위해 제작한 가이드라인 자료입니다.

## 1. Hardware Setting
### 1.1. Overall Hardware Setting
본 프로젝트는 크게 네가지 시스템으로 나뉩니다. 첫번째는 Indy10 로봇팔 구동 시스템, 두번째는 OCR을 통한 메뉴 인식 시스템, 세번째는 Arduino 재료 운반 시스템, 마지막으로 Stand Light 환경 속에서 이루어지는 패티 종류 인식 시스템입니다.
<div align="center">
  <img width="940" alt="3D_Schemetic" src="https://github.com/YunKiNoh/24-2_IAIA_Project2-Automatic-Hamburger-Stacking-System/blob/main/image/3D_Schemetic.png" />
  <p style="margin-top: 10px;">Fig 1. 3D Schematic of ROS Hamburger System.</p>
</div>

Indy 10 로봇을 중심으로 하여, 왼쪽에는 재료를 옮기기 위한 아두이노 시스템이, 오른쪽에는 이미지 프로세싱을 통한 패티 인식 시스템이 위치하고, 앞쪽의 왼쪽 부분에서는 OCR을 통한 메뉴 인식 시스템이 위치합니다. 추가적으로 전면 중앙에서 햄버거의 각 재료가 쌓이며, 햄버거가 완성되면 종을 울린 뒤에 Indy10은 대기 상태로 되돌아갑니다. 이때 Indy10 로봇은 제시되어 있는 배치 속에서 햄버거 쌓기를 수행하게 되어 있으므로, 본 프로젝트를 다시 수행하기 위해서는 제시되어 있는 환경을 구축한 뒤에 미세한 위치를 조정하면 됩니다. 

### 1.2. Indy10 Robot Setting

본 프로젝트를 본격적으로 수행하기에 앞서, Indy10 로봇을 사용하기 위해서는 다음과 같은 과정을 수행하면 됩니다.

- 로봇의 전원을 킵니다.
- PC와 태블릿의 WiFi를 IAIA-5G로 연결합니다.
- 로봇의 IP 주소를 확인합니다. [IP: 192.168.0.9]
- Terminal에 아래의 코드를 통해 로봇과 연결한다.
```
roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.9
```

### 1.3. Conveyor Belt Setting

이번 프로젝트에서는 햄버거의 각 재료를 자동으로 Indy10 로봇 앞으로 옮겨놓기 위해서 아두이노를 통해 컨베이어 벨트를 작동시켰습니다. 특히, 초음파 센서를 통해 재료가 로봇팔 앞에 도착했는지 여부에 따라 컨베이어 벨트의 작동을 구분지었고, 추가적으로 스태퍼 모터를 통해 뒤집개가 재료를 잘 옮길 수 있도록 칸막이를 설치하였습니다. 해당 Conveyor Belt 시스템은 총 컨베이어 벨트를 위한 스태퍼 모터1, 칸막이를 위한 스태퍼 모터2, 초음파 센서, 그리고 해당 센서들을 통합 작동시키기 위한 Arduino Uno R3 보드가 사용되었습니다. 각 센서들은 다음과 같이 연결하여 구동시키면 됩니다.
<div align="center">
  <img width="940" alt="3D_Schemetic" src="https://github.com/YunKiNoh/24-2_IAIA_Project2-Automatic-Hamburger-Stacking-System/blob/main/image/ConveyorBelt_Circuit.png" />
  <p style="margin-top: 10px;">Fig 2. ConveyorBelt_Circuit.</p>
</div>

## 2. Software Setting
### 2.1. Overall Software Environment
본 프로젝트는 우분투 환경을 기반으로 하여 ROS Python, OCR(Optimal Character Recognition) Deeplearning, 그리고 Arduino를 활용하였는데, 가장 기본이 되는 프로그램의 종류와 버전은 다음과 같습니다.
- 우분투: Ubuntu 20.04

- 파이썬: Python 3.8

- 아두이노: Arduino 1.8.13

- 딥러닝: OCR Deeplearning Model


### 2.2. Check ROS Python Code

우선 Indy10 로봇은 ROS 환경을 기반으로 파이썬 코드를 통해 데이터를 주고 받으며 동작을 수행합니다. 특히, 이번 프로젝트를 위해서는 총 7개의 파이썬 코드를 구축 및 실행하였습니다.
- Python Source Link: [링크 변경]
- 7가지 파이썬 코드들
0. `catkin_ws/src/indy_driver/src`: 파일 위치로 이동<br>
1. `camera.py`: OCR 수행을 위한 카메라 코드<br>
2. `image_display.py`:<br>
3. `image_processing.py`<br>
4. `test_ocr_video.py`<br>
5. `image_display_ocr.py`<br>
6. `ham_classifier.py`<br>
7. `test_motion.py`<br>

### 2.3. Install 'Aruidno IDE 1.8.13' for 'Ubuntu 20.04'
이번 프로젝트는 Ubuntu 20.04에서 이루어졌기 때문에, 해당 버전에 출시되었던 Arduino IDE 1.8.13을 설치하였습니다.

#### 2.3.1. Download 'Aruidno IDE 1.8.13' for 'Ubuntu 20.04'
우선 안정성을 확보하기 위해서 우분투 20.04 버전에 개발된 Aruidno IDE 1.8.13 파일을 다운로드 해줍니다.
```
$ sudo wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
```

#### 2.3.2. Install 'Aruidno IDE 1.8.13'
그러고 나서 해당 파일을 압축해제 한 뒤에
```
$ tar -xf arduino-1.8.13-linux64.tar.xz
```

해당 폴더로 이동한 뒤 설치를 진행합니다.
```
$ cd arduino-1.8.13
```
```
$ sudo ./install.sh
```

#### 2.3.3. Authorize
처음 아두이노를 설치하면 권한을 부여하여야 포트에 연결할 수 있기 때문에, 다음과 같이 아두이노에 대한 권한을 부여해줍니다.
```
$ cd arduino-1.8.13
```
```
sudo chown 사용자이름 arduino
```

### 2.3. Setting for OCR Utilization
본 프로젝트에서는 주문표를 인식하기 위하여 OCR(Optical character recognition) DeepLearning Model을 사용하였습니다. 이 모델을 사용하기 위해서는 다음과 같이 몇가지 과정을 거쳐야 합니다.
#### 2.3.1. Create google key for use of google API
다음 사이트에 들어가서 구글 API를 사용하기 위한 키를 생성합니다.

[Link: https://blog.naver.com/rhrkdfus/221335351270]

#### 2.3.2. Create json file
그 후에는 API 시스템에서 json file을 생성할 수 있는데, 예시 결과는 다음 링크를 통해 확인할 수 있습니다.

[Link: https://github.com/JinGaram/AIAI/blob/main/clever-obelisk-441505-v9-02af9e940897.json]

#### 2.3.3. Install some file in the ubuntu command prompot
이후에 Ubuntu OS 터미널 창에서 아래를 설치합니다.

```
pip install --upgrade google-api-python-client
```
```
pip install --upgrade google-cloud-vision
```

#### 2.3.4. Register json file as system variable
JSON 계정 키가 정상적으로 다운로드 되었다면 다음과 같이 json 파일을 시스템 변수로 등록해줘야 합니다.
```
setx GOOGLE_APPLICATION_CREDENTIALS (json 파일 위치)\(json 파일 이름).json
```

#### 2.3.5. Using OCR Model
이상으로 Google OCR 연결은 완료 되었다. 다음과 같이 OCR을 사용할 수 있습니다.
```
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
```

만약 에러가 발생할 경우 아래와 같이 시도해보도록 합니다.
- Python Version: 우분투와 윈도우에서 사용하는 Python 버전이 동일한지 확인
```
python3 --version
```
- Library Version: 설치된 패키지 버전이 다른 경우 문제가 발생할 수 있습니다.
```
pip freeze > requirements.txt
```

```
pip install -r requirements.txt
```

## 3. Execute System
이처럼, 하드웨어와 소프트웨어 환경 준비가 완료되면 미리 제공된 7개의 Python 코드들을 실행시켜 햄버거를 자동으로 쌓는 기능을 수행할 수 있습니다. 다음과 같이 각각의 터미널을 통해서 각각의 과정을 차례대로 수행합니다.
- Terminal 1

```
roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.9
```

​로봇 시스템을 구축하려면 Indy 10과 서버 간 연결이 필요합니다. 두 장치 모두 동일한 Wi-Fi 네트워크를 사용해야 하며, 이 예제에서는 IP 주소 192.168.0.9를 사용했습니다. Ubuntu 터미널을 통해 연결한 후, 로봇 팔을 제어할 수 있습니다.



- Terminal 2 

```
rosrun indy_driver camera.py
```

publish: `camera/image_raw2`

​이 노드는 카메라 센서에서 이미지 데이터를 생성합니다. 카메라는 패티 위에 설치되어 이미지 처리를 수행합니다.



- Terminal 3

```
rosrun indy_driver image_display.py
```

subscribe: `camera/image_raw2`

​이 노드는 패티 위에 설치된 카메라에서 데이터를 구독하여 이미지를 윈도우에 표시해 사용자에게 보여줍니다.



- Terminal 4

```
rosrun indy_driver image_processing.py
```

publish: `image_processing/object_info`

subscribe: `camera/image_raw2`, `ham_classifier/ham_info`,  `RobotState_info`

​이 노드는 시스템에서 가장 중요한 노드로 볼 수 있습니다. 이 노드는 먼저 패티를 촬영하는 카메라 데이터를 구독합니다. 또한 로봇 상태(Robot State)에 대한 정보를 구독하며, 사용자 주문 정보를 위한 카메라 입력도 구독합니다. 이 세 가지 데이터를 활용해 하나의 정보를 출력합니다.
알고리즘은 로봇 상태가 0일 때만 작동하며, 로봇이 동작 중일 때는 데이터 처리를 중단합니다. 또한, 첫 번째 카메라에서 얻은 햄버거 정보와 두 번째 카메라에서 얻은 정보가 일치하면`object_info`를 다음 노드로 출력합니다. 정보가 일치하지 않을 경우 데이터를 출력하지 않습니다.
패티 위에 설치된 카메라를 통해 정보를 가져오는 과정에서 이미지 처리를 수행합니다. 이 노드는 세 종류의 햄버거 패티에 대한 BGR 이미지를 처리하며, 관심 영역(ROI) 내에서 이미지 처리를 집중적으로 수행합니다.



- Terminal 5

```
rosrun indy_driver test_ocr_video.py
```

publish: `camera/image_raw`

이 노드는 카메라 센서에서 이미지 데이터를 생성합니다. 사용자의 주문 정보를 캡처하기 위해 주문 스테이션 위에 설치된 카메라를 사용합니다. 정보를 수신한 후 OCR(광학 문자 인식)을 시도합니다. OCR 딥러닝 모델은 Google API에서 제공하는 모델을 활용하며, JSON 파일을 통해 Google Cloud와 통합하여 시스템을 연결합니다.



- Terminal 6

```
rosrun indy_driver image_display_ocr.py
```

subscribe: `camera/image_raw2`

이 노드는 주문 스테이션 위에 설치된 카메라에서 데이터를 구독하고, 이미지를 윈도우에 표시하여 사용자에게 보여줍니다.


- Terminal 7

```
rosrun indy_driver ham_classifier.py
```

publish: `ham_classifier/ham_info`

subscribe: `camera/image_raw`, `Robot State_info`

이 노드는 OCR 카메라에서의 텍스트 입력 데이터와 RobotState_info 데이터를 구독합니다. 알고리즘은 로봇이 동작 중일 때 실행되지 않습니다. 로봇이 정지 상태일 때, 카메라가 특정 햄버거 유형을 15초 이상 감지하면 해당 정보를 다음 노드로 출력합니다.



- Terminal 8 

```
rosrun indy_driver test_motion.py
```

subscribe: `ham_classifier/ham_info`, `image_processing/object_info`

이 마지막 노드는 두 가지 정보를 수신한 후 알고리즘을 실행합니다. 알고리즘은 각 햄버거 유형에 대한 특정 절차를 포함하며, `Image_processing`에서 받은 정보를 기반으로 패티의 위치를 계산합니다. 이 과정에서 패티의 상관 관계식을 사용하여 위치를 결정합니다. 햄버거 제작 과정이 완료되면 알고리즘이 종료되고, 로봇은 대기 위치로 돌아가 정지합니다.
