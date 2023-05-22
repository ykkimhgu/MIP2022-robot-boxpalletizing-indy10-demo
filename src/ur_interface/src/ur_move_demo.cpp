// #include <ros/ros.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

// #include <iostream>
// #include <vector>

// using namespace std;

// int main(int argc, char**argv)
// {
//     /*ros init*/
//     ros::init(argc, argv, "ur_move_demo");
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     /* init moveit*/
//     moveit::planning_interface::MoveGroupInterface arm("manipulator");
//     arm.setGoalJointTolerance(0.01);
//     arm.setMaxAccelerationScalingFactor(1);
//     arm.setMaxVelocityScalingFactor(1);

//     /*move to setted pose*/

//     arm.setNamedTarget("stand_by");
//     arm.move();

//     /*getting joint state*/
//     vector<double> currentJointState=arm.getCurrentJointValues();
//     for(int i=0; i<currentJointState.size(); i++)
//     {
//         cout<<currentJointState[i]<<', ';
//     }
//     cout<<endl<<endl;

//     /*move joint 6 90 degree and move it back*/
//     currentJointState[4]-=3.141592/3;
//     arm.setJointValueTarget(currentJointState);
//     arm.move();

//     currentJointState[4]+=3.141592/3;
//     arm.setJointValueTarget(currentJointState);
//     arm.move();

//     /*get cartesian position and orientation*/
//     geometry_msgs::PoseStamped currentPose;
//     currentPose=arm.getCurrentPose();
//     cout<<currentPose<<endl;

//     /*move position x 5cm z -5cm*/
//     geometry_msgs::Pose movePose;
//     movePose.position.x=currentPose.pose.position.x;
//     movePose.position.y=currentPose.pose.position.y;
//     movePose.position.z=currentPose.pose.position.z-0.05;
//     movePose.orientation=currentPose.pose.orientation;
//     arm.setPoseTarget(movePose);
//     arm.move();

//     /*move orientation pitch 30 degree*/
//     currentPose=arm.getCurrentPose();
//     movePose.position=currentPose.pose.position;

//     tf2::Quaternion q_orig, q_rot, q_new;
//     double r=0, p=3.141592/6, y=0;
//     tf2::convert(currentPose.pose.orientation, q_orig);
//     q_rot.setRPY(r, p, y);

//     q_new=q_orig*q_rot;
//     q_new.normalize();

//     tf2::convert(q_new, movePose.orientation);

//     arm.setPoseTarget(movePose);
//     arm.move();
    
//     return 0;
// }


#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char**argv)
{
    /*ros init*/
    ros::init(argc, argv, "ur_move_demo");
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
    for(int i=0; i<currentJointState.size(); i++)
    {
        cout<<currentJointState[i]<<', ';
    }
    cout<<endl<<endl;

    /*move joint 6 90 degree and move it back*/
    currentJointState[5]-=3.141592/3;
    arm.setJointValueTarget(currentJointState);
    arm.move();

    currentJointState[5]+=3.141592/3;
    arm.setJointValueTarget(currentJointState);
    arm.move();

    /*get cartesian position and orientation*/
    geometry_msgs::PoseStamped currentPose;
    currentPose=arm.getCurrentPose();
    cout<<currentPose<<endl;

    /*move position x 5cm z -5cm*/
    geometry_msgs::Pose movePose;
    movePose.position.x=currentPose.pose.position.x;
    movePose.position.y=currentPose.pose.position.y;
    movePose.position.z=currentPose.pose.position.z-0.05;
    movePose.orientation=currentPose.pose.orientation;
    arm.setPoseTarget(movePose);
    arm.move();


    /*move position x 5cm z -5cm*/
    movePose.position.x=0.1;
    movePose.position.y=0.5;
    movePose.position.z+=0.05;
    arm.setPoseTarget(movePose);
    arm.move();



    return 0;
}
