#!/usr/bin/env python
#-*- coding:utf-8 -*- 

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from indy_driver.msg import ObjectInfo

# Window에 출력되는 text 형식 선언
font                   = cv2.FONT_HERSHEY_SIMPLEX
fontScale              = 1
fontColor              = (0,0,255)
lineType               = 2

# 메시지를 보내기 위한 publisher 형성
pub_ObjectInfo = rospy.Publisher("ObjectInfo", ObjectInfo, queue_size = 10)
msg_ObjectInfo = ObjectInfo()

# 퍼블리셔 노드로부터 토픽(Color Image)을 받아들이는 콜백 함수
def color_imageCallback(data):
    try:

        # Color Image를 받아오기
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        # color = np.asanyarray(cv_image)
        color = np.asanyarray(cv_image[0:480, 0:640])

        # 원본 이미지를 보존하기 위해 복사 이미지 형성
        res = color.copy()
        
        # color를 GaussianBlur 처리한 뒤 hsv로 변환
        blur_color = cv2.GaussianBlur(color, (5, 5), 0)
        hsv = cv2.cvtColor(blur_color, cv2.COLOR_BGR2HSV)

        # # 탐지하고자 하는 색깔의 범위를 설정 (BGR이 아닌 HSV로 설정) - 박스 색상
        lower_b = np.array([-10,70,150])
        upper_b = np.array([50,130,210])

        # mask에 탐지된 부분을 띄우며, color에 해당된 부분만 보이게 이미지를 합성
        mask = cv2.inRange(hsv, lower_b, upper_b)
        result = cv2.bitwise_and(color, color, mask=mask)

        # mask에서 Contour를 특정
        (_ , c, _) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        for contour in c:

            pix_cnt = 0

            # 특정 면적 이상의 Contour에 대해서만 진행 (조건: 면적 10000 초과)
            if cv2.contourArea(contour) > 10000:

                # Contour의 최소 면적을 나타내는 사각형을 res에 출력 (회전)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(res, [box], -1, (255, 0, 0), 3)    
                
                # Contour와 외접하는 사각형의 정보를 얻음
                (x, y, w, h) = cv2.boundingRect(contour)
                text_point = (x, y+np.int0(1.2*h))

                # 에러 방지 구문: boundingRect의 범위가 640x480 frame의 경계값과 같은 경우 오류가 발생
                if x + w == 640:
                    w = w - 1
                if y + h == 480:
                    h = h - 1
                
                # Contour에 외접하는 사각형을 res에 출력
                cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3)

                
                ####### 각도 부분은 추후 디벨롭이 필요함 #######
                # 표면의 각도를 측정 (0 ~ 89도)
                angle = np.int0(round(rect[2], 2))
                
                if angle == 90 :
                    angle = 0

                # 표면의 각도를 text로 설정
                text = "Angle: " + str("{0:0d}").format(angle) + "[degree]"

                # res에 출력되는 text2 형식
                cv2.putText(res,
                            text,
                            text_point,
                            font,
                            fontScale,
                            fontColor,
                            lineType)

                # 물체 중심 좌표 & 각도 메세지 송신
                msg_ObjectInfo.check = 1
                msg_ObjectInfo.x = x + w/2
                msg_ObjectInfo.y = y + h/2
                msg_ObjectInfo.angle = angle
                pub_ObjectInfo.publish(msg_ObjectInfo)

                print("Message Served...\n\n")


        # =========================== 이미지 시각화 ===========================

        # cv2.namedWindow("RGB_Orig_Image")
        # cv2.imshow("RGB_Orig_Image", cv_image)

        cv2.namedWindow('RGB_Contour_Image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow("RGB_Contour_Image", res)  

        # cv2.namedWindow('mask', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('mask', mask)

        cv2.namedWindow("Masking_Image")
        cv2.imshow("Masking_Image", result)

        cv2.waitKey(1)


    except CvBridgeError as e:
        print(e)


# 퍼블리셔 노드로부터 토픽(Depth Image)을 받아들이는 콜백 함수
def depth_imageCallback(ros_image):

    # Use cv_bridge() to convert the ROS image to OpenCV format
    bridge = CvBridge()

    try:
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

    except CvBridgeError as e:
        print(e)

    
    #Convert the depth image to a Numpy array
    depth_img = depth_image[0:480, 0:640]
    depth_array = np.array(depth_image, dtype=np.float32)

    # To capture depth information
    depth_info = 0
    depth_size = 0

    for i in range(200, 280) :
        for j in range(280, 360) :

            depth_temp = depth_array[i, j]

            if depth_temp == 0 or depth_temp > 10000 :
                continue

            else :
                depth_size = depth_size + 1
                depth_info = depth_info + depth_temp

    depth_info = depth_info/depth_size

    # 물체 거리 메세지 송신
    msg_ObjectInfo.dist = depth_info
    pub_ObjectInfo.publish(msg_ObjectInfo)


    # ==================== 참고: 가운데 픽셀 거리 데이터만 뽑아내는 코드 =======================

    # # To capture depth information
    # center_idx = np.array(depth_array.shape) / 2
    # # print(depth_array.shape)
    # # print(center_idx.shape)
    # print ('center depth:', depth_array[center_idx[0], center_idx[1]])

    # =================================================================================


    # print("depth information: {:.2f}cm".format(depth_info/10))


    cv2.namedWindow("Depth_Image")
    # cv2.imshow("Depth_Image", depth_image)
    cv2.imshow("Depth_Image", depth_img)

    cv2.waitKey(1)


def main():
    # 노드 초기화. 이름은 listener
    rospy.init_node('getImage', anonymous=True)

    ### 특정 토픽(chatter)를 callback이라는 이름의 함수로 받아들이며, 메시지 타입은 test_msg
    rospy.Subscriber("/camera/color/image_raw", Image, color_imageCallback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_imageCallback)
    
    ### Depth Camera 없을 때, 코드 검증용으로 USB 카메라 사용
    # rospy.Subscriber("usb_cam/image_raw", Image, color_imageCallback)
    
    
    loop = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():

        loop.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()
