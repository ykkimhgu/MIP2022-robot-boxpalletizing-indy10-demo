#!/usr/bin/env python
#-*- coding:utf-8 -*- 
import rospy
from indy_driver.msg import ObjectInfo, TargetPose, IndyRobotState, GripState, GripCommand 


#  =============================== Variable Initialize ================================
# Object State
flag        = 0
objectDist  = 0
objectAngle = 0
objectX     = 0
objectY     = 0

# Robot State & Grip State
robotState  = 0
gripState   = 0
gripChange  = 0

# ======================================================================================

def readObjectState(_msg):
    global flag, objectDist, objectAngle, objectX, objectY

    flag        = _msg.check
    objectDist  = _msg.dist
    objectAngle = _msg.angle
    objectX     = _msg.x
    objectY     = _msg.y
    

def readRobotState(_msg):
    global robotState
    robotState = _msg.move

def readGripState(_msg):
    global gripState, gripChange
    gripChange = _msg.change
    gripState  = _msg.on 


def main():
    
    rospy.init_node('Controller', anonymous=True)

    pub_TargetPose      = rospy.Publisher("TargetPose", TargetPose, queue_size = 10)
    pub_GripCommand     = rospy.Publisher("GripCommand", GripCommand, queue_size = 10)
    pub_GripState       = rospy.Publisher("GripState", GripState, queue_size = 10)

    sub_ObjectInfo      = rospy.Subscriber("ObjectInfo", ObjectInfo, readObjectState)
    sub_IndyRobotState  = rospy.Subscriber("IndyRobotState", IndyRobotState, readRobotState)
    sub_GripState       = rospy.Subscriber("GripState", GripState, readGripState)

    msg_TargetPose  = TargetPose()
    msg_GripCommand = GripCommand()
    msg_GripState   = GripState()

    msg_GripCommand.DO_Pin0 = 10    # Gripper : DO PIN - 8, 9
    msg_GripCommand.DO_Pin1 = 11    # Vaccum  : DO PIN - 10, 11
    msg_GripCommand.DO_Val0 = 0
    msg_GripCommand.DO_Val1 = 1

    global flag
    cnt = 0
    layer = 0

    box_x_len = 0.075
    box_y_len = 0.1
    box_height = 0.1
    
    loop = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        # If box detected, Palletizing Start!
        if flag == 1 : 

            print("depth information: {:.2f}cm".format(objectDist/10))
            print("angle information: {:.2f}\n\n".format(objectAngle))
            # print("central position: (x = {:.2f}, y = {:.2f})".format(objectX, objectY) )
            rospy.sleep(rospy.Duration(1))


            # =================== Mode에 따른 로봇 동작 ===================    

            # 1. 컨베이어 벨트 위에 놓인 박스 위치(고정된 위치)로 로봇팔 이동

            msg_TargetPose.x        = -0.02091  
            msg_TargetPose.y        = -0.46541  
            msg_TargetPose.z        = 0.66652 
            msg_TargetPose.roll     = -179.87   
            msg_TargetPose.pitch    = 7.97
            msg_TargetPose.yaw      = 88.74     
            pub_TargetPose.publish(msg_TargetPose)
            rospy.sleep(rospy.Duration(1.5))

            # 2. 진공 그리퍼 joint6 회전 (각도에 따른 회전)
            msg_TargetPose.roll     = 179.9     #objectAngle
            msg_TargetPose.pitch    = 0.0
            msg_TargetPose.yaw      = 0.0
            pub_TargetPose.publish(msg_TargetPose)
            rospy.sleep(rospy.Duration(1))


            # 3. 그리퍼 하강시켜 박스에 밀착
            msg_TargetPose.z        = 0.51652
            # msg_TargetPose.z        -= 0.15
            pub_TargetPose.publish(msg_TargetPose)
            
            rospy.sleep(rospy.Duration(0.5))

            # 4. 진공 그리퍼 작동
            msg_GripCommand.DO_Val0 = 1
            msg_GripCommand.DO_Val1 = 0
            pub_GripCommand.publish(msg_GripCommand)
            pub_GripState.publish(msg_GripState)
            rospy.sleep(rospy.Duration(1))

            # 5. 박스 들어올리기
            msg_TargetPose.z        = 0.66652
            # msg_TargetPose.z       += 0.15
            msg_TargetPose.yaw      = 88.74   
            pub_TargetPose.publish(msg_TargetPose)
            rospy.sleep(rospy.Duration(1))

            # 6. Palletizing Sequence에 따라, 목표 위치로 로봇 팔 이동

            if cnt == 0 :
                msg_TargetPose.x        = 0.44087
                msg_TargetPose.y        = 0.17830
                msg_TargetPose.z        = 0.43519 + box_height * layer
                pub_TargetPose.publish(msg_TargetPose)
                rospy.sleep(rospy.Duration(1.5))

            elif cnt == 1 :
                msg_TargetPose.x        = 0.44087 + box_x_len 
                msg_TargetPose.y        = 0.17830
                msg_TargetPose.z        = 0.43519 + box_height * layer
                pub_TargetPose.publish(msg_TargetPose)
                rospy.sleep(rospy.Duration(1.5))

            elif cnt == 2 :
                msg_TargetPose.x        = 0.44087
                msg_TargetPose.y        = 0.17830 + box_y_len
                msg_TargetPose.z        = 0.43519 + box_height * layer
                pub_TargetPose.publish(msg_TargetPose)
                rospy.sleep(rospy.Duration(1.5))

            elif cnt == 3 :
                msg_TargetPose.x        = 0.44087 + box_x_len
                msg_TargetPose.y        = 0.17830 + box_y_len
                msg_TargetPose.z        = 0.43519 + box_height * layer
                pub_TargetPose.publish(msg_TargetPose)
                rospy.sleep(rospy.Duration(1.5))


            # 7. 목표 x,y 좌표 도달한 뒤, 그리퍼 수직으로 아래로 내리기
            # msg_TargetPose.z       = -0.15
            msg_TargetPose.z       = 0.43519 + box_height * layer - 0.15
            pub_TargetPose.publish(msg_TargetPose)
            rospy.sleep(rospy.Duration(1))

            # 8. 목표 위치에 도달했으면, 그리퍼 작동 Off하여 물체 떨어뜨리기
            msg_GripCommand.DO_Val0 = 0
            msg_GripCommand.DO_Val1 = 1
            pub_GripCommand.publish(msg_GripCommand)
            pub_GripState.publish(msg_GripState)
            rospy.sleep(rospy.Duration(1))

            # 9. 박스 떨군 뒤, 그리퍼 다시 수직으로 위로 올리기
            # msg_TargetPose.z       += 0.15
            msg_TargetPose.z       = 0.43519 + box_height * layer
            pub_TargetPose.publish(msg_TargetPose)
            rospy.sleep(rospy.Duration(1))

            # 10. Palletizing Sequence 조정
            cnt = cnt + 1

            if cnt == 4 :
                cnt = 0
                layer = layer + 1
                print("{} layer loading complete\n".format(layer))

            flag = 0
            rospy.sleep(rospy.Duration(1.5))

        loop.sleep()


    rospy.spin()

if __name__ == '__main__':
    main()