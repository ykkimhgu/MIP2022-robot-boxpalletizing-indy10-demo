
1. 실행하기 
roslaunch pointgrey_camera_driver camera.launch camera:=/15384429

rosrun indy_driver video

rosrun indy_driver vidoe_py.py

  1) gazebo (simulation)
	 roslaunch indy10_gazebo indy10_moveit_gazebo.launch
  
  2) Indy 10 (real)
  	 roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.5


2. Files
  1) video.cpp
  	 카메라 칼리브레이션을 통해 평면 이미지 정보를 출력하고 publish.

  2) video_py.py
     평면 이미지 정보를 subscribe.
     평면 이미지 정보를 활용하여 물체의 위치와 목표 위치를 생성하고 publish.

  3) listner.py
     물체 위치와 목표 위치 정보가 잘 생성되고 있는지 확인하고자 만든 listener 노드임.
     굳이 실행하지 않아도 됨.
     rosrun indy_driver listener.py

  4) video_copy.cpp
  	 기존의 video.cpp의 복사본임.

  ++) py파일 실행할 수 없다고 하면, 해당 폴더 위치에서 chmod +x ~~~.py

3. Message
	1) Image_msg = 평면 이미지 정보

	2) CameraDetect = 물체 위치, 목표 위치


4. 문제점
Cameratrg의 on 정보를 cr로 읽어내고 있는데, 기존의 코드에서는 cr을 활용해서 원활하게 동작된 거 같음.
그런데, py로 수정하고 나서는 cr을 활용하지 못해서, 동작이 원활하지 않음.

Cameratrg는 Set HomePose가 완료되면 on 
video.cpp에서 위치 정보를 publish하면 off (Cameratrg.on = 1 일 때만 위치 정보를 publish)


