# How to Use Realsense Depth camera on Ubuntu 18.04?

작성자 : 21700469 유진수

참조 : https://github.com/IntelRealSense/realsense-ros

Pyrealsens2 설치법 = https://jstar0525.tistory.com/97 <br>
pip가 설치 안 되어있다면? https://jjeongil.tistory.com/1274 참고하여 진행하십시오.


<br>

## 1. 환경 설정

<br> 

### ROS Package 설치
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

sudo apt-get install ros-melodic-realsense2*

sudo apt install --reinstall ros-melodic-realsense2-description
```

### Realsense Depth Camera 실행
```
roslaunch realsense2_camera rs_camera.launch

# Depth & Color 프레임 사이즈를 동일하게 조절하여 이미지를 취득하고 싶다면?
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
launch 파일은 opt/ros/melodic/share/realsense_camera_launch 에 저장되어 있습니다.


### 연결된 카메라 확인
```
# 이미지 확인이 필요할 때?
rqt

# 연결된 카메라의 포트가 궁금할 때?
ls -ltr /dev/video*
```

## 2. Camera Calibration

1. 코드 실행
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/color/image_raw
```

위 코드 상에서 인가하는 파라미터는 다음과 같습니다.
- **--size**     -> 체커보드 코너의 개수. 위 코드는, 체커보드 내부에 가로 코너 8개, 세로 코너 6개가 있음을 의미합니다.
- **--square**   -> 체커보드 내부 정사각형의 실측 길이 (meter). 위 코드상으론 10.8cm
- **--image:=**  -> 카메라 토픽. rqt를 통해 사용자의 환경을 확인하고 필요시 변경하면 됩니다.

<br>

2. 코드가 실행된 후? 
- X, Y, Size, Skew 바가 초록색으로 채워질 때까지 체스 보드를 움직입니다.

<br>

3. 계산 결과 저장 및 적용 \
Calibration이 완료되면 SAVE 버튼을 눌러서 Calibration 정보 파일을 저장한 뒤 압축을 해제합니다.
(Calibration 결과는 tmp 폴더 (opt 폴더와 동일한 위치-ros의 상위 폴더) 에 저장되어 있습니다.)
```
cd /tmp
tar -xvzf calibrationdata.tar.gz
mv ost.txt ost.ini
```

<br>

4. 카메라 파라미터 파일 생성
```
rosrun camera_calibration_parsers convert ost.ini camera.yaml
```
- 실행이 안 될 경우 **catkin_make**를 시도합니다.
- **catkin_make** 에서 오류 발생 시, 경로를 확인하십시오 (source /opt/ros/$ROS_DISTRO/setup.bash)

<br>

5. 파라미터 참조
```
mkdir ~/.ros/camera_info
mv camera.yaml ~/.ros/camera_info/
```

