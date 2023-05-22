#!/usr/bin/env python
#-*- coding:utf-8 -*- 

import sys
import rospy
import time
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ur_interface.msg import PartList


# Window에 출력되는 text 형식 선언
font                   = cv2.FONT_HERSHEY_SIMPLEX
fontScale              = 1
fontColor              = (0,0,255)
lineType               = 2

rows                   = 4  # 부품 개수
cols                   = 2  # 부품 순서, 정보
qr_list = [[0 for j in range(cols)] for i in range(rows)]

# 메시지를 보내기 위한 publisher 형성
pub_PartList = rospy.Publisher("PartList", PartList, queue_size = 100)
msg_PartList = PartList()

# 퍼블리셔 노드로부터 토픽(Image)을 받아들이는 콜백 함수
def imageCallback(data):
    try:

        # Color Image를 받아오기
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        image = np.asanyarray(cv_image)

        # 이미지를 Gray scale로 변환
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
        # Gray scale로 변환된 QR 이미지를 해독
        decoded = pyzbar.decode(gray)

        # QR 개수를 카운트
        cnt = 0

        for d in decoded :

            # QR 코드 영역 추출
            x, y, w, h = d.rect

            # QR 코드의 타입과 데이터를 읽음
            barcode_data = d.data.decode("utf-8")
            barcode_type = d.type

            # QR 코드의 데이터를 part에 저장
            part = barcode_data
            center_point_x = x + (w / 2)
            center_point_y = y + (h / 2)

            qr_list[cnt][0] = center_point_y
            qr_list[cnt][1] = barcode_data

            # QR 코드 부분에 사각형을 그림
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
        
            # QR 코드의 타입과 데이터를 text에 저장한 뒤에 화면에 나타냄
            text = '%s (%.f, %.f)' % (barcode_data, center_point_x, center_point_y)
            cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    
            # QR 갯수를 업데이트
            cnt += 1
   
            # QR 입력값을 선언
            qr_val = str(0)

            ## QR 4개가 읽힌 경우 입력값을 정리
            if cnt == 4:
                qr_list.sort()
        
                # 메세지 송신
                msg_PartList.check = 1
                msg_PartList.Part0 = qr_list[0][1]
                msg_PartList.Part1 = qr_list[1][1]
                msg_PartList.Part2 = qr_list[2][1]
                msg_PartList.Part3 = qr_list[3][1]

                pub_PartList.publish(msg_PartList)
                cnt = 0
                msg_PartList.check = 0

                print("\n\npublish message....\n")
                print(msg_PartList.Part0, msg_PartList.Part1, msg_PartList.Part2, msg_PartList.Part3)


        # =========================== 이미지 시각화 ===========================

        cv2.namedWindow('Image_Processing', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Image_Processing', image)

        cv2.waitKey(1)


    except CvBridgeError as e:
        print(e)


def main():
    # 노드 초기화. 이름은 listener
    rospy.init_node('getImage', anonymous=True)

    # ============= 특정 토픽(chatter)를 callback이라는 이름의 함수로 받아들입니다 =============
    ### Depth Camera 사용 시 아래 코드 실행
    rospy.Subscriber("/camera/color/image_raw", Image, imageCallback)

    ### 웹캠 사용 시 아래 코드 실행
    # rospy.Subscriber("usb_cam/image_raw", Image, imageCallback)
    # ======================================================================================
    
    loop = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():

        loop.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()