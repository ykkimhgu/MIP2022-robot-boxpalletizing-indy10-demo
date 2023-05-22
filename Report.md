https://www.youtube.com/watch?v=Hvvd_KvdBXs&list=PL12w7vYWefUz9aRGRQqPWWLH9gTTEovRW&index=2&ab_channel=KIMeLab

# ROS

로봇 소프트웨어를 개발하기 위한 소프트웨어 프레임워크
오픈소스, 메타운영체제.

메타운영체제(Meta-Operating System)이란?
- 윈도우 & 리눅스 & 안드로이드 등의 전통적인 운영체제가 아니라, 전통적인 운영체제 위에서 돌아가는 운영체제.
- 프로그램/라이브러리/툴 등의 집합체.
- 로봇 <-----> 메타 운영체제 <-----> 센서

노드 간 메시지 교환 방식을 통해 복잡한 프로그램을 잘게 나눠 공동 개발이 가능
GUI 도구 모음 rqt, 시각화 도구 Rviz, 3차원 시뮬레이터 Gazebo 지원
다양한 프로그래밍 언어 사용 가능(roscpp = c++, rospy = python 등)

# ROS 용어

**Node**
- 최소 단위의 실행 가능한 프로세서

**Package**
- 하나 이상의 노드, 노드 실행을 위한 정보 등을 묶어 놓은 것.

**Message**
- 노드 간 데이터를 주고받기 위한 도구(=구조)
- integer, floating, point, boolean과 같은 변수 형태
