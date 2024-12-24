# Project2: Robot Operating System for Making Hamburger Model with Deep Learning

- Author: 21800226 Yunki Noh / 21900727 Garam Jin / 21900258 Hyeonho Moon 
- Date: 2024.12.20

본 레포지토리는 Industrial AI and Automation의 Project2[Automatic Hamburger Stacking System with Indy 10]의 수행을 돕기 위해 제작한 가이드라인 자료입니다.

## Hardware
본 프로젝트는 크게 네가지 시스템으로 나뉩니다. 첫번째는 Indy10 로봇팔 구동 시스템, 두번째는 OCR을 통한 메뉴 인식 시스템, 세번째는 Arduino 재료 운반 시스템, 마지막으로 Stand Light 환경 속에서 이루어지는 패티 종류 인식 시스템입니다.
<div align="center">
  <img width="940" alt="3D_Schemetic" src="https://github.com/YunKiNoh/24-2_IAIA_Project2-Automatic-Hamburger-Stacking-System/blob/main/image/3D_Schemetic.png" />
  <p style="margin-top: 10px;">Fig 1. 3D Schematic of ROS Hamburger System.</p>
</div>


Indy 10 로봇을 중심으로 하여, 왼쪽에는 재료를 옮기기 위한 아두이노 시스템이, 오른쪽에는 이미지 프로세싱을 통한 패티 인식 시스템이 위치하고, 앞쪽의 왼쪽 부분에서는 OCR을 통한 메뉴 인식 시스템이 위치합니다. 추가적으로 전면 중앙에서 햄버거의 각 재료가 쌓이며, 햄버거가 완성되면 종을 울린 뒤에 Indy10은 대기 상태로 되돌아갑니다.

## Software
본 프로젝트는 우분투 환경을 기반으로 하여 ROS Python, OCR(Optimal Character Recognition) Deeplearning, 그리고 Arduino를 활용하였습니다.

### ROS Python Code

우선 Indy10 로봇은 ROS 환경을 기반으로 파이썬 코드를 통해 데이터를 주고 받으며 동작을 수행합니다. 특히, 이번 프로젝트를 위해서는 총 7개의 파이썬 코드를 구축 및 실행하였습니다.

**Python Source Link:** [링크 변경]

7가지 파이썬 코드들

`catkin_ws/src/indy_driver/src`<br>
1. `camera.py`<br>
2. `image_display.py`<br>
3. `image_processing.py`<br>
4. `test_ocr_video.py`<br>
5. `image_display_ocr.py`<br>
6. `ham_classifier.py`<br>
7. `test_motion.py`<br>

### OCR DeepLearning 

### Arduino
이번 프로젝트는 Ubuntu 20.04에서 이루어졌기 때문에, 해당 버전에 출시되었던 Arduino IDE 1.8.13을 설치하였습니다.

#### Download 'Aruidno IDE 1.8.13' for 'Ubuntu 20.04'
우선 안정성을 확보하기 위해서 우분투 20.04 버전에 개발된 Aruidno IDE 1.8.13 파일을 다운로드 해줍니다.
```
$ sudo wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
```

#### Install 'Aruidno IDE 1.8.13'
그러고 나서 해당 파일을 압축해제 한 뒤에, 
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

#### Authorize
처음 아두이노를 설치하면 권한을 부여하여야 포트에 연결할 수 있기 때문에, 다음과 같이 아두이노에 대한 권한을 부여해줍니다.
```
$ cd arduino-1.8.13
```
```
sudo chown 사용자이름 arduino
```
