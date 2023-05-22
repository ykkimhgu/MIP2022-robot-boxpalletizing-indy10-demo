# Demo: Palletizing

작성자 : 21700469 유진수

본 md 파일은 팔레타이징, 즉 물류 적재 공정 구현에 대해 다루고 있습니다.

데모 구현을 위한 사전 환경 설정은, 아래의 링크를 참조하십시오.

- Indy10 초기 설정 : [Click This Link](https://github.com/chaochao77/ROS_neuromeka_tutorial)
- Depth Camera Setting : [Click This Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Depth_Camera_Manual.md)


<br>

## 공정 개요

팔레타이징이란 팔레트(화물 운반대)위에 박스 등의 물품을 적재하는 작업으로, <br>
물류 산업에서 흔히 볼 수 있는 공정입니다.

무거운 물체를 운반하는 중노동을 사람이 아닌 로봇이 대신함으로써 안전하고, 편리한 공정을 구축하려는 노력이 수행되고 있습니다.

본 프로젝트는, 이러한 팔레타이징 공정을 구현하는 것을 목표로 합니다.

본 공정의 개요를 대략적으로 표현하면 아래와 같습니다. <br>

<br>

![image](https://user-images.githubusercontent.com/84503980/206381741-e6b0427a-5d67-4188-86d7-122b72dffb0a.png)

<br>

비전 센서(카메라 등)에 물체가 인식되면, 정해진 위치로 로봇이 이동하여 박스를 들어올립니다. <br>
이후, 정해진 순서에 따라 순차적으로 박스를 적재합니다.

<br>

## 공정 구현

아래의 코드를 순차적으로 copy & paste 함으로써, 손쉽게 공정을 구현할 수 있습니다.

<br>

### 1. 로봇-노트북 연결
```
# Simulation
roslaunch indy10_gazebo indy10_moveit_gazebo.launch

# Real
roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.6
```

<br>

### 2. 카메라 전원 On(이미지 취득)

```
# Color Frame과 Depth Frame 동기화(with align) -> 얘 실행시킬 것!
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# without align (참고 목적으로 기록하였으나, 실행시키지 않습니다)
roslaunch realsense2_camera rs_camera.launch

# 카메라 포트 확인 코드 (Depth Camera가 잘 연결되었는지 확인하고 싶은 경우에만 실행합니다)
ls -ltr /dev/video*
```

<br>

### 3. 이미지 프로세싱

```
rosrun indy_driver Image_Processing.py
```

#### 3-1. Description about "Image_Processing.py"

해당 코드는, 카메라로부터 이미지를 취득한 뒤 영상처리 기법을 활용하여 로봇에 메세지를 송신하는 코드입니다.


<br>


### 4. 로봇 제어 코드 실행
```
rosrun indy_driver Robot_Controller.py
```

#### 4-1. Description about "Robot_Controller.py"

해당 코드는, 송신받은 메시지를 기반으로 순차적으로 로봇에 동작 명령을 내리는 코드입니다.

정상적인 데모를 구현하기 위해서는, 박스의 사이즈 정보 및 박스가 놓일 위치에 대한 좌표 정보를 코드에 인가해 주어야 합니다.


<br>

## Demo Video

[Click this Link]()




