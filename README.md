# MIP-Robot_Control_using_ROS

<br>

## Introduction

이 Repository는 한동대학교 2022년 2학기에 진행된 기전융합설계(Mechatronics Integration Projects), <br>
**Simulation Environment Establishment for Robot Arm Control** 에 대한 설치 파일 및 튜토리얼로 구성되었습니다.

<br>

본 프로젝트는 산업 현장의 주요 관심사 중 하나인 **공정 자동화** 를, <br>
로봇 팔을 활용하여 효과적으로 구축하기 위한 방법을 제시하기 위한 목적으로 수행되었습니다. <br>

센서로부터 취득한 데이터를 기반으로 로봇에 명령을 내리는 제어 프로그램을 작성하고, <br>
해당 제어 프로그램을 시뮬레이션 환경에서 먼저 실행시킴으로써 안전한 자동화 공정을 구축하고자 합니다.

매니퓰레이터 모델은 **INDY-10 (Neuromeka)** 과 **UR5e (Universal Robot)** 을 사용하였으며, <br>
Linux 환경에서 ROS의 Rviz, Gazebo, Moveit 패키지를 활용하여 프로그램을 작성하였습니다.

<br>

**본 프로젝트는 한동대학교 김영근 교수님의 지도아래 진행되었습니다.**

<br>

## Requirements

본 프로젝트는, Linux - ubuntu 18.04 환경에서 ROS-melodic을 활용하여 진행했습니다. <br>

**python**과 **C++** 을 활용하여 프로그램을 작성했기에, 해당 언어에 대한 활용 능력이 요구됩니다.


<br>

## Contents
* ### Tutorial: How to use INDY-10
  * [Reference Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Indy10/Indy10_Manual.md)

* ### Tutorial: How to use UR5e
  * [Reference Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/UR5e/UR5e_Maunal.md)

* ### Tutorial: How to use vision sensor
  * [Depth Camera](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Depth_Camera_Manual.md)
  * [USB Camera](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Webcam_Manual.md)
  
* ### Tutorial: Palletizing (Demo)
  * [Reference Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Indy10/Demo_Palletizing.md)

* ### Tutorial: Classification & Pick-Place (Demo)
  * [Reference Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/UR5e/Demo_Classification_Automate_using_QR_Code.md)

* ### TroubleShooting
  * [Reference Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Trouble_Shooting.md)
