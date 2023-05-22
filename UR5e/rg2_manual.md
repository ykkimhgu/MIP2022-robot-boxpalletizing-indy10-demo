# Gripper Setting


## 홍창민 연구원님 메일 내용

#### 1. Universal robot의 URDF를 수정하여 Onrobot 그리퍼를 추가합니다.

src/universal_robot/ur_description/urdf



#### 2. moveit setup assistant를 수정하여 URDF 추가된 Endeffector를 설정합니다.

- 홍창민 연구원이 준 참고 사이트
https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html



#### 3. 원하시는 Application에 Onrobot 드라이버 시스템을 세팅합니다.

- UR에서 Thread 이용, I/O 신호 이용 or Modbus 이용  
//I/O신호를 이용하거나 Modbus신호를 이용하는 부분에서는 그리퍼의 움직임에 따라서 고르시면 됩니다. Modbus를 이용할 경우, 그리퍼의 넓이를 설정 할 수 있음, I/O 신호를 이용할 경우 그리퍼에 Trigger를 주어서 그리퍼의 넓이 등을 설정할 수 있음

<br>
<br>

## 참고할 만한 자료 (진수 서칭)

https://advrhumanoids.github.io/ROSEndEffectorDocs/install.html#install

https://advrhumanoids.github.io/ROSEndEffectorDocs/usage.html

https://advrhumanoids.github.io/ROSEndEffectorDocs/roseeGazeboPlugins.html#prepare4gazebo

#rg2 모델링....? urdf 여기서 일단 구했음
https://github.com/ekorudiawan/rg2_simulation


## ur5 & rg2 연결...?
https://github.com/AndrejOrsula/ur5_rg2_moveit2_config


http://wiki.ros.org/onrobot


## 오오.......?
https://github.com/Osaka-University-Harada-Laboratory/onrobot

위 링크 참고: 우분투 20.04를 기준으로 하기에, 임의로 melodic용으로 코드를 바꿔봤음. 바꾼 코드는 아래와 같음

```
git clone https://github.com/takuya-ki/onrobot.git --depth 1
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git --depth 1

sudo rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro melodic -y --os=ubuntu:bionic -y

sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers

catkin_make
(build devel src 이외의 폴더가 만들어져있을 텐데, 해당 폴더들을 src 안으로 옮긴 뒤 cmake 하면 오류 없이 잘 된다)

rosdep install onrobot_rg_modbus_tcp

roslaunch onrobot_rg_description disp_rg2_model.launch
```





https://roboticscasual.com/ros-tutorial-how-to-create-a-moveit-config-for-the-ur5-and-a-gripper/
