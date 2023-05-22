# Demo: Classification_Automate using QR Code

작성자 : 21700469 유진수

<br>

본 md 파일은, 딥러닝 모델(QR 코드 리더기)을 활용한 제품의 분류 자동화 공정 구현에 대해 다루고 있습니다.

데모 구현을 위한 사전 환경 설정은, 아래의 링크를 참조하십시오.

- Ubuntu 18.04 설치 : [Click This Link](https://ykkim.gitbook.io/dlip/installation-guide/ubuntu/ubuntu-18.04-installation)
- UR5e 초기 설정 : [Click This Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/UR5e/UR5e_Maunal.md)
- QR 코드 리더기 패키지 다운로드 : [Click This Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/blob/main/Trouble_Shooting.md)
- 컨베이어 벨트 환경 구축 :

<br>

## 공정 개요

물류 산업에서는, QR코드 또는 바코드를 활용하여 물품의 종류를 판별합니다. <br>
또한, 컨베이어 벨트를 활용하여 물품을 이송합니다.

본 프로젝트는 이러한 환경 및 설비로부터 취득한 데이터를 기반으로, 로봇 팔을 제어하는 것을 목적으로 합니다.

해당 목적을 달성하기 위해 설계한 공정의 개요를 대략적으로 표현하면 아래와 같습니다.

<br>

![image](https://user-images.githubusercontent.com/84503980/206390286-cf8e0846-62fd-44e2-9d32-0f4f629fd023.png)

<br>

### **분류 공정 <br>**
컨베이어 벨트를 타고 물품이 운송됩니다. <br>
비전 센서에 4개의 QR 코드가 인식되면 물품이 정상적인 위치에 도달했다고 판단하고, QR 코드의 정보를 로봇에 송신합니다. <br>
이후 수신된 정보를 기반으로, 컨베이어 벨트 좌/우에 놓인 상자에 물품을 분류 적재합니다.

본 프로젝트에서는, A1 & A2 & B1 & B2 라는 정보를 갖는 QR 코드를 생성하였고, <br>
A 상자와 B 상자 안에 물건을 분류 적재하는 방향으로 공정을 설계했습니다. <br>
1과 2를 굳이 나눈 것은, 경우의 수를 늘림으로써 보다 복잡한 공정을 구현하기 위함이었습니다.

---
<br><br>


## 공정 설계

### Why ROS? <br>

로봇 제어 프로그램의 안전성을 검증하기 위해, 시뮬레이션이 가능한 ROS 환경에서 공정을 설계했습니다.

ROS는 Robot Operating System의 약자로, 로봇 소프트웨어를 개발하기 위한 소프트웨어 프레임워크 오픈소스를 말합니다. <br>

노드 간 메시지 교환 방식을 통해 복잡한 프로그램을 잘게 나눠 공동 개발이 가능하며,  <br>
GUI 도구 모음 rqt, 시각화 도구 Rviz, 3차원 시뮬레이터 Gazebo 등을 지원할 뿐 만 아니라, <br>
다양한 프로그래밍 언어(roscpp = c++, rospy = python 등)를 사용할 수 있다는 강점이 있기 때문에 해당 환경을 채택했습니다. <br>

<br>

### 코드 알고리즘 설명 <br>

본 공정의 핵심은 비전 센서로부터 취득한 이미지를 영상처리 기법을 활용하여 정제하고, 이를 통해 로봇에 적절한 명령을 내리는 것입니다.
이번 절에서는 해당 목적을 수행하기 위해 작성한 프로그램에 대해 설명하고자 합니다. 
코드 내 주석을 기입하였으나, 보다 명확한 전달을 위해 중요한 부분에 대한 보충 설명을 기록하였습니다.

<br>

### 1. Image_Processing.py

카메라로부터 취득한 이미지로부터 QR 코드를 읽어내고, 로봇을 제어하는 노드로 메세지를 보내는 프로그램입니다. <br>

<br>

**먼저, main문을 살펴보겠습니다.**

<br>

<p align="center">
      <img src="https://user-images.githubusercontent.com/84503980/206849956-82f5c30f-241f-4317-849e-4317675893a2.png" width="50%" height="50%" alt="text" width="number" />
      </p>
      
<br/>

**getImage**라는 이름의 노드를 생성했습니다. 메세지를 보내기 위해서는, 노드 생성이 우선되어야 하기 때문입니다.

영상처리 기법을 수행하기 위해서는, 먼저 비전 센서로(=카메라)로부터 취득한 이미지 데이터를 받아 오는 과정이 필요합니다. <br>
따라서, 아래와 같은 코드를 사용하였습니다.

```
# Depth Camera 사용 시 아래 코드 실행
rospy.Subscriber("/camera/color/image_raw", Image, imageCallback)

# 웹캠 사용 시 아래 코드 실행
rospy.Subscriber("usb_cam/image_raw", Image, imageCallback)
```

**/camera/color/image_raw**, **usb_cam/image_raw** 부분은 받아 올 토픽의 이름을 의미합니다.
카메라의 경우, 카메라를 실행시킨 뒤 **rqt** 명령어를 실행함으로써 받아 올 토픽의 이름을 확인할 수 있습니다.
사용한 장비에 따라 토픽 이름이 다르게 형성되므로, 유저의 환경에 맞춰 코드를 수정하시면 되겠습니다.

<br>

**다음으로, imageCallback 함수를 살펴보겠습니다.**

이미지를 취득할 때마다, **imageCallback** 함수가 실행됩니다.  <br>
해당 함수는 딥러닝 모델을 활용해 QR 코드를 인식하고, 로봇에 메시지를 송신하는 역할을 합니다. <br>
메세지의 구조는 다음과 같습니다.

<br>

<p align="center">
      <img src="https://user-images.githubusercontent.com/84503980/206851091-ae26ac73-ff08-4861-b380-fa9162ba55f7.png" width="25%" height="25%" alt="text" width="number" />
      </p>
      
<br/>

4개의 부품이 카메라 범위에 인식된 경우 로봇에 메시지를 송신하고, 부품을 분류하는 공정을 설계하였기 때문에 <br>
4개의 부품 각각에 대한 정보를 보내기 위한 변수를 생성하였습니다. <br>
또한, 메세지가 송신된 경우에만 로봇을 동작시키기 위해 flag용 변수를 **check** 라는 이름으로 생성했습니다.

함수 내부, 메세지를 송신하는 부분은 다음과 같습니다.

<br>

<p align="center">
      <img src="https://user-images.githubusercontent.com/84503980/206850899-9dbc7104-6b0f-446c-9cbe-9475a99c280a.png" width="50%" height="50%" alt="text" width="number" />
      </p>
      
<br/>

cnt 변수를 통해 인식된 QR 코드의 개수를 확인합니다. <br>
cnt == 4 일 때, 컨베이어 벨트를 통해 정해진 위치에 물품이 도착했다고 판단, 로봇에 메세지를 송신합니다.

<br><br>

### 2. closed_loop_system.cpp

최초에는 2개의 로봇을 활용하고, 포장 공정까지 수행함으로써 무한히 반복하는 데모를 형성하기 위한 목적으로 **closed_loop_system**이라는 이름을 붙였습니다. <br>
그러나 실제 산업 공정과는 동떨어진, 별개의 이야기이므로 본 시뮬레이션 코드는 오직 **분류** 에 집중했습니다. <br>
무한히 반복되는 데모를 구현하고자 한다면, 현재 코드 상에서 특정 로직을 추가해 주기만 하면 됩니다. <br>
김상현 학우가 구현한 indyDCP 기반 코드를 참고하기 바랍니다.

<br>

이번 절에서는, ur5e에 동작 명령을 내릴 때 주의해야 할 부분에 대해 집중적으로 다루겠습니다.

<br>

**초기 설정**

먼저, 메시지 송-수신을 위한 노드를 형성합니다. 이후, **Image_Processing.py** 로부터 송신된 메시지를 수신합니다. <br>
**msgcallback** 함수를 통해 수신된 값을 로봇의 제어 변수에 할당합니다.

**spin** 부분은, callback 함수를 호출하기 위한 부분입니다.

```
ros::init(argc, argv, "MIP_Closed_loop_system");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe<ur_interface::PartList>("PartList", 100, msgcallback);

ros::AsyncSpinner spinner(1);
spinner.start();
```

<br>

아래 부분은, 로봇을 움직이는 기구학/역기구학 부분을 다루는 **moveit** 패키지를 활용하여 로봇의 환경 설정을 해 주는 부분입니다.

```
/* init moveit*/
moveit::planning_interface::MoveGroupInterface arm("manipulator");
arm.setGoalJointTolerance(0.01);
arm.setMaxAccelerationScalingFactor(1);
arm.setMaxVelocityScalingFactor(1);
```

<br>

**moveit setup assistant** 를 통해, 홈위치 & 영위치를 비롯한 다양한 자세를 설정할 수 있습니다. <br>
아래는, 사전에 설정해 두었던 **stand_by** 라는 이름의 자세로 로봇을 움직이기 위한 명령입니다.

**moveit setup assistant** 사용법이 궁금하다면, 이 [링크]()를 참조하십시오

```
/*move to setted pose*/
arm.setNamedTarget("stand_by");
arm.move();
```

<br>

아래는, 현재 로봇의 조인트 상태 및 자세 정보를 가져오는 부분입니다. <br>
일반적으로 로봇을 제어할 경우 **현재의 상태에서 특정한 동작을 수행해라!** 는 명령을 반복적으로 내리게 됩니다. <br>
따라서 특정 동작 수행 이후에는 현재 로봇의 상태 정보를 반복적으로 갱신해 주는 것이 무엇보다 중요합니다.

```
/*getting joint state*/
vector<double> currentJointState=arm.getCurrentJointValues();

/*get cartesian position and orientation*/
geometry_msgs::Pose movePose;
geometry_msgs::PoseStamped currentPose;
currentPose=arm.getCurrentPose();
cout<<currentPose<<endl;
```

<br><br>

**로봇 제어**

메세지를 수신 받은 경우 **imageServed** 변수에 1을 할당하고, 로봇이 동작하도록 로직을 설계했습니다.
본 프로젝트에서는, **moveit setup assistant** 를 통해 설정한 **stand_by** 상태를 로봇 동작의 시발점으로 하였습니다.

분류할 부품의 크기에 따라, 코드 상으로 입력한 상수의 값을 조절해야 합니다. <br>
해당 부분은 코드 내부에 매우 직관적으로 나타나 있으므로, 설명은 생략하겠습니다. <br> 

부품 1개를 분류할 때 마다 **cnt** 변수의 값을 1씩 증가시켰으며, <br>
4가 된 경우 모든 분류 공정이 완료되었다고 판단, **imageServed** 변수를 0으로 초기화시킴으로써 로봇의 동작을 정지시켰습니다. <br>

시뮬레이션 상으로 그리퍼를 연동시키지 못했기 때문에, 그리퍼를 동작시켜야 할 부분에서는 임의로 로봇의 6번 joint를 회전시켰습니다. <br>
해당 부분은, 추가적인 디벨롭이 필요합니다.

---

<br><br>

## 공정 구현

아래의 코드를 순차적으로 copy & paste 함으로써, 손쉽게 공정을 구현할 수 있습니다.

<br>

### 1. 로봇-노트북 연결
```
# Simulation
roslaunch ur_gazebo ur5e_bringup.launch
roslaunch ur5e_sim_moveit_config move_group.launch

# Real
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.2
roslaunch ur5e_real_moveit_config move_group.launch
```

<br>

### 2. 이미지 취득
```
# Depth Camera 사용 시 아래 코드 실행
roslaunch realsense2_camera rs_camera.launch align_depth:=true

# 웹캠 사용 시 아래 코드 실행
rosrun usb_cam usb_cam_node
```

<br>

### 3. 이미지 프로세싱
```
# 이미지 내부 QR 코드 인식, 메세지 송신
rosrun ur_interface Image_Processing.py
```

<br>

### 4. 로봇 제어
```
rosrun ur_interface closed_loop_system
```


<br>

## Demo Video

[Click this Link]()
