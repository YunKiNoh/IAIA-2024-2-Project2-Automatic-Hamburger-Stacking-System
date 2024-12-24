# Project2: Robot Operating System for Making Hamburger Model with Deep Learning

- Author: 21800226 Yunki Noh / 21900727 Garam Jin / 21900258 Hyeonho Moon 
- Date: 2024.12.20

본 레포지토리는 Industrial AI and Automation의 Project2[Automatic Hamburger Stacking System with Indy 10]의 수행을 돕기 위해 제작한 가이드라인 자료입니다.

## 1. Hardware
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

## 2. Software
### 2.1. Overall Software Environment
본 프로젝트는 우분투 환경을 기반으로 하여 ROS Python, OCR(Optimal Character Recognition) Deeplearning, 그리고 Arduino를 활용하였는데, 가장 기본이 되는 프로그램의 종류와 버전은 다음과 같습니다.
- 우분투: Ubuntu 20.04

- 파이썬: Python 3.8

- 아두이노: Arduino 1.8.13

- 딥러닝: OCR Deeplearning Model


### 2.2. ROS Python Code

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

### 2.3. 'Aruidno IDE 1.8.13' for 'Ubuntu 20.04'
이번 프로젝트는 Ubuntu 20.04에서 이루어졌기 때문에, 해당 버전에 출시되었던 Arduino IDE 1.8.13을 설치하였습니다.

### 2.3.1. Download 'Aruidno IDE 1.8.13' for 'Ubuntu 20.04'
우선 안정성을 확보하기 위해서 우분투 20.04 버전에 개발된 Aruidno IDE 1.8.13 파일을 다운로드 해줍니다.
```
$ sudo wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
```

### 2.3.2. Install 'Aruidno IDE 1.8.13'
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

### 2.3.3. Authorize
처음 아두이노를 설치하면 권한을 부여하여야 포트에 연결할 수 있기 때문에, 다음과 같이 아두이노에 대한 권한을 부여해줍니다.
```
$ cd arduino-1.8.13
```
```
sudo chown 사용자이름 arduino
```
