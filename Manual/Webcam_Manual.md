# How to Use Webcam on Ubuntu 18.04?

작성자 : 21700469 유진수
참조 : http://wiki.ros.org/usb_cam

<br>

## 1. 패키지 설치

ROS Melodic에서는, uvc_camera를 사용할 수 없습니다. 지원을 만료했기 때문입니다. \
따라서, 패키지를 설치해 주어야 합니다.

설치 명령은 아래와 같습니다.

```
sudo apt install ros-melodic-usb-cam
```

설치가 끝나면, 업데이트를 수행합니다.

```
audo apt-get update
```

## 2. 카메라 실행

아래의 명령어를 통해, 카메라로부터 이미지를 취득할 수 있습니다.

```
rosrun usb_cam usb_cam_node
```

해당 명령어 실행 전에는, 마스터 노드를 생성시켜 주어야 합니다.

Indy10, UR5e 등을 제어하기 위해 launch 파일을 실행시킨 경우 마스터 노드가 생성되므로 큰 문제가 없으나, \
카메라를 단독적으로 실행시키고 싶다면 아래의 코드를 터미널에 입력하면 됩니다.

```
roscore
```

실행 현황을 살펴보고 싶다면, 아래의 명령어를 입력합니다.

```
// 카메라 화면을 확인하고 싶을 때 
rqt

// 노드 연결 상태를 보고 싶을 때
rqt_graph

// 연결된 카메라의 포트를 확인하고 싶을 때
ls -ltr /dev/video*
```
