# 이슈사항 발생 시 해결 방법

참조 : https://github.com/hyKangHGU/ROS_Ubuntu_18_04/blob/main/ROS%20-%20Python3_version_up.md

<br>

### 커스텀 메시지 관련 이슈?

참조 : https://m.blog.naver.com/nswve/222031819104

1. 작업공간 내 msg 폴더 형성하고, 폴더 안에 파일 생성
![msg](https://user-images.githubusercontent.com/84503980/204326203-c27e73e6-8b1f-4a07-9af6-82922e0a683d.png)


2. **CMakeLists.txt** 파일 수정

- ***find_package***에 *message_generation* 추가
- ***add_message_files***에 메세지 파일 추가!
- ***generate_messages***에 *std_msgs* 활성화

3. **Catkin_make**

파이썬과 CPP 두 파일로 메세지 송수신을 하고 싶다면? \
참조 : https://www.theconstructsim.com/subscribe-cpp-to-custom-msg-array-published-python/

<br>

### 패키지를 다운 받았는데, 계속 없다는 에러가 발생할 경우?
- 경로 문제일 가능성이 농후합니다. 아래의 코드를 통해 경로를 확인하고, 필요시 수정하면 됩니다.
```
# 현재 경로 확인
cat ~/.bashrc

# 경로 설정이 안 된 경우
source devel/setup.bash #터미널 실행 시 해당 경로 접근해서 쉘을 킨다는 의미. 경로 설정해두면 편합니다.

# 경로가 본인의 작업 공간과 다른 경우
sudo gedit ~/.bashrc
```

<br>

### em 모듈이 없다는 오류가 발생할 경우?

- 모듈을 다운 받아 주면, 가볍게 해결됩니다.

```
sudo apt install python3-empy
```

<br>

### 스크립트 파일이 실행되지 않는 경우?

- 작업 공간(스크립트 파일이 위치한 경로)에 들어간 뒤, 파일 확장자를 확인하기!

```
ls -al
```

- 파이썬 코드의 경우, +x 파일이 아니면 실행되지 않는다. 확장자는 아래의 코드로 바꿀 수 있습니다.
```
chmod +x filename.py
```

<br>

### QR코드 사용 관련 이슈?

- 아래의 코드를 통해, QR코드를 읽는 모델 **pyzbar**를 다운로드합니다
```
pip install pyzbar
```
- **ImportError: Unable to find zbar shared library** 오류가 발생할 경우?
```
sudo apt install libzbar0
```
