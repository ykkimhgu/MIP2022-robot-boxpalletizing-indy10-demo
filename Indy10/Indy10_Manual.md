# How to Use Indy10 on Ubuntu 18.04?

작성자 : 21700469 유진수

참조 : https://github.com/chaochao77/ROS_neuromeka_tutorial

<br>

## 1. 환경 설정

로봇 사용을 위한 외부적인 환경 설정은 참조한 링크에 상세히 설명되어 있으므로, <br>
본 문서에서는 software 관련 부분만 작성하였습니다.

<br>

### 1. 작업 공간 생성

아래 코드를 copy & paste 하여 작업 공간을 생성합니다.

```
# Go home directory
cd ~ 


# 작업 공간 생성
mkdir -p indy_ws/src
```

### 2. src 파일 다운로드 & Catkin_make

본 Repository에 업로드 되어 있는, indy10 폴더 내 src 파일 전체를 다운로드합니다.
- [DownLoad Link](https://github.com/Yjinsu/MIP-Robot_Control_using_ROS/tree/main/Indy10/src)

파일 다운로드 이후, **1. 작업 공간 생성** 을 통해 만든 src 폴더 안에 다운로드한 파일 전체를 옮겨줍니다.

파일을 모두 옮긴 뒤에는, 아래의 코드를 copy & paste 합니다.

```
# Install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
catkin_make
```

### 3. 경로 설정
```
# 터미널 실행 시 해당 경로에 접근, 쉘 활성화 (환경설정 초기 1회만 수행)
source devel/setup.bash 

# 현재 경로 확인이 필요하다면?
cat ~/.bashrc

#경로 수정이 필요하다면?
sudo gedit ~/.bashrc 
```


<br>


## 2. 로봇-컴퓨터 간 IP 연결 or Simulation 환경 연결

아래의 코드를 copy & paste 합니다.

```
# 로봇 연결 + 시뮬레이션 실행
roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.6

# 오직 시뮬레이션만 실행
roslaunch indy10_gazebo indy10_moveit_gazebo.launch
```

<br>

## 3. 로봇 제어

로봇 제어를 위한 코드를 실행합니다.

```
# Example
rosrun indy_driver Robot_Controller.py
```
