#include <ur_interface/PartList.h>
#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <vector>

using namespace std;

int imageServed = 0;
int cnt = 0;
string partList[4][1];

void msgcallback(const ur_interface::PartList::ConstPtr& msg){

    imageServed = msg->check;
    partList[0][0] = msg->Part0;
    partList[1][0] = msg->Part1;
    partList[2][0] = msg->Part2;
    partList[3][0] = msg->Part3;
}

int main(int argc, char**argv)
{
    /*ros init*/
    ros::init(argc, argv, "MIP_Closed_loop_system");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<ur_interface::PartList>("PartList", 100, msgcallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* init moveit*/
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.setGoalJointTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);

    /*move to setted pose*/
    arm.setNamedTarget("stand_by");
    arm.move();

    /*getting joint state*/
    vector<double> currentJointState=arm.getCurrentJointValues();

    /*get cartesian position and orientation*/
    geometry_msgs::Pose movePose;
    geometry_msgs::PoseStamped currentPose;
    currentPose=arm.getCurrentPose();
    cout<<currentPose<<endl;

    // Robot Control ===============================================

    while(ros::ok()){

        currentPose=arm.getCurrentPose();

        // QR 정보가 들어온 경우
        if (imageServed == 1) {

            // 메시지 수신 확인 문구 출력
            if (cnt == 0) {
                printf("\nImage Served\n");
                cout<<"Part0: "<<partList[0][0];
                cout<<"\tPart1: "<<partList[1][0];
                cout<<"\tPart2: "<<partList[2][0];
                cout<<"\tPart3: "<<partList[3][0];
            }

            // ================== 부품 위로 이동 ==================
            movePose.position.x=currentPose.pose.position.x;
            movePose.position.y=currentPose.pose.position.y + 0.02 * (cnt + 1);
            movePose.position.z=currentPose.pose.position.z;
            movePose.orientation=currentPose.pose.orientation;

            arm.setPoseTarget(movePose);
            arm.move();     
            ros::Duration(0.5).sleep();
            currentPose=arm.getCurrentPose();

            // ================== 부품 PICK ==================
            movePose.position.x=currentPose.pose.position.x;
            movePose.position.y=currentPose.pose.position.y;
            movePose.position.z=currentPose.pose.position.z - 0.035;
            movePose.orientation=currentPose.pose.orientation;

            arm.setPoseTarget(movePose);
            arm.move();     
            ros::Duration(0.5).sleep();
            currentPose=arm.getCurrentPose();

            /* =============== 예제코드: JOINT6 각도 회전(60도). 실제로는 그리퍼가 동작하는 부분 =============== */

            // currentJointState=arm.getCurrentJointValues();
            // currentJointState[5]-=3.141592/3;
            // arm.setJointValueTarget(currentJointState);
            // arm.move();
            // ros::Duration(0.5).sleep();

            /* =======================================================================================*/


            // ============== 부품 집은 채 z축 방향으로 올라감 ==============
            movePose.position.x=currentPose.pose.position.x;
            movePose.position.y=currentPose.pose.position.y;
            movePose.position.z+=0.025;
            movePose.orientation=currentPose.pose.orientation;
            arm.setPoseTarget(movePose);
            arm.move();
            ros::Duration(0.5).sleep();
            currentPose=arm.getCurrentPose();


            // ================ QR코드에 따라 상자 위로 이동 ================
            if (partList[cnt][0] == "A1" || partList[cnt][0] == "A2") {

                // 왼쪽 상자 위로 이동
                movePose.position.x=currentPose.pose.position.x - 0.25;
                movePose.position.y=currentPose.pose.position.y + 0.02 * (cnt + 1);
                movePose.position.z=currentPose.pose.position.z;
                movePose.orientation=currentPose.pose.orientation;
            }

            else if (partList[cnt][0] == "B1" || partList[cnt][0] == "B2") {

                // 오른쪽 상자 위로 이동
                movePose.position.x=currentPose.pose.position.x + 0.25;
                movePose.position.y=currentPose.pose.position.y + 0.02 * (cnt + 1);
                movePose.position.z=currentPose.pose.position.z;
                movePose.orientation=currentPose.pose.orientation;
            }

            arm.setPoseTarget(movePose);
            arm.move();
            ros::Duration(0.5).sleep();
            currentPose=arm.getCurrentPose();

            // =============== 상자로 접근 (물건 내려놓기 위함) ===============
            movePose.position.x=currentPose.pose.position.x;
            movePose.position.y=currentPose.pose.position.y;
            movePose.position.z=currentPose.pose.position.z-0.035;
            movePose.orientation=currentPose.pose.orientation;
            arm.setPoseTarget(movePose);
            arm.move();     
            ros::Duration(0.5).sleep();
            currentPose=arm.getCurrentPose();

            /* =============== 예제코드: JOINT6 각도 회전(60도). 실제로는 그리퍼가 동작하는 부분 =============== */

            // currentJointState=arm.getCurrentJointValues();
            
            // currentJointState[5]+=3.141592/3;
            // arm.setJointValueTarget(currentJointState);
            // arm.move();
            // ros::Duration(0.5).sleep();
            // currentPose=arm.getCurrentPose();

            /* =======================================================================================*/
            
            // // =============== 상자 위로 올라감 ===============
            movePose.position.x=currentPose.pose.position.x;
            movePose.position.y=currentPose.pose.position.y;
            movePose.position.z+=0.025;
            //movePose.orientation=currentPose.pose.orientation;
            arm.setPoseTarget(movePose);
            arm.move();
            ros::Duration(0.5).sleep();


            // ================= 초기 상태로 이동 =================
            arm.setNamedTarget("stand_by");
            arm.move();
            ros::Duration(1).sleep();

            currentPose=arm.getCurrentPose();            
            
            cnt = cnt + 1;

            // 4개의 부품을 모두 옮긴 후에는, 새로운 메세지를 받기 전 까지 동작 정지
            if (cnt == 4) {
                cnt = 0;
                imageServed = 0;
            }


        }

        // loop_rate.sleep();
        // ros::spin();
    }

    ros::shutdown();
    return 0;
}
