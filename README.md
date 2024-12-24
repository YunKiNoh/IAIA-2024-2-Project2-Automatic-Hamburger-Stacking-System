# Project2: Robot Operating System for Making Hamburger Model with Deep Learning

- Author: 21800226 Yunki Noh / 21900727 Garam Jin / 21900258 Hyeonho Moon 
- Date: 2024.12.20

본 레포지토리는 Industrial AI and Automation의 Project2[Automatic Hamburger Stacking System with Indy 10]의 수행을 돕기 위해 제작한 가이드라인 자료입니다.

## Hardware
본 프로젝트는 크게 네가지 시스템으로 나뉩니다. 첫번째는 Indy10 로봇팔 구동 시스템, 두번째는 OCR을 통한 메뉴 인식 시스템, 세번째는 Arduino 재료 운반 시스템, 마지막으로 Stand Light 환경 속에서 이루어지는 패티 종류 인식 시스템입니다.
<p align="center">
  <img width="940" alt="3D_Schemetic" src="https://github.com/YunKiNoh/24-2_IAIA_Project2-Automatic-Hamburger-Stacking-System/blob/main/image/3D_Schemetic.png" />
</p>

Indy 10 로봇을 중심으로 하여, 관찰자 시점에서 왼쪽에는 아두이노 시스템, 오른쪽에는 패티 인식 시스템이 위차하고, 앞쪽에는 OCR을 통한 메뉴 인식 시스템이 위치합니다. 

## Software
본 프로젝트는 우분투 환경을 기반으로, ROS Python, OCR(Optimal Character Recognition), Arduino를 활용하였습니다.

### ROS Python Code

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
