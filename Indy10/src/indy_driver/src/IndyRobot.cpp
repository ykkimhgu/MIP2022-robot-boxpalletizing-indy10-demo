#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <stdio.h>
#include <indy_driver/TargetPose.h>
#include <indy_driver/IndyRobotState.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>

#define ABS_COMP_2NUMS(n1,n2)       double (fabs(n1 - n2))
#define COMP_2NUMS(num, threshold)  int (num > threshold ? 1 : 0)

double targetPose[6] = { 0.0, 0.4, 0.818, 179.9, 0.0, 0.0 };

void readTargetPose(const indy_driver::TargetPose::ConstPtr& _msg){
    targetPose[0] = _msg -> x;
    targetPose[1] = _msg -> y;
    targetPose[2] = _msg -> z;
    targetPose[3] = _msg -> roll;
    targetPose[4] = _msg -> pitch;
    targetPose[5] = _msg -> yaw;
    // printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", targetPose[0], targetPose[1], targetPose[2], targetPose[3], targetPose[4], targetPose[5]);
}

int checkMovingState(geometry_msgs::Pose _msg_TargetPose, geometry_msgs::PoseStamped _cPose, float _threshold){
    int state;
    if (COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.position.x, _cPose.pose.position.x), _threshold) ||
        COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.position.y, _cPose.pose.position.y), _threshold) ||
        COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.position.z, _cPose.pose.position.z), _threshold) ||
        COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.orientation.x, _cPose.pose.orientation.x), _threshold) ||
        COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.orientation.y, _cPose.pose.orientation.y), _threshold) ||
        COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.orientation.z, _cPose.pose.orientation.z), _threshold) ||
        COMP_2NUMS(ABS_COMP_2NUMS(_msg_TargetPose.orientation.w, _cPose.pose.orientation.w), _threshold)    )
        state = 1;
    else
        state = 0;

    // printf("%.2f %.2f\n", _msg_TargetPose.position.x, _cPose.pose.position.x);
    // printf("%.2f %.2f\n", _msg_TargetPose.position.y, _cPose.pose.position.y);
    // printf("%.2f %.2f\n", _msg_TargetPose.position.z, _cPose.pose.position.z);
    // printf("%.2f %.2f\n", _msg_TargetPose.orientation.x, _cPose.pose.orientation.x);
    // printf("%.2f %.2f\n", _msg_TargetPose.orientation.y, _cPose.pose.orientation.y);
    // printf("%.2f %.2f\n", _msg_TargetPose.orientation.z, _cPose.pose.orientation.z);
    // printf("%.2f %.2f\n", _msg_TargetPose.orientation.w, _cPose.pose.orientation.w);
    
    return state;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "IndyRobot");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int go_target = 0;

    // Initialising and defining the planning group for move_base
    static const std::string PLANNING_GROUP = "indy10";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // Subscriber & Publisher
    ros::Subscriber sub_TargetPose  = n.subscribe<indy_driver::TargetPose>("TargetPose", 10, readTargetPose);
    // ros::Publisher  pub_PoseStamped = n.advertise<geometry_msgs::PoseStamped>("CurrentPose", 10);
    ros::Publisher pub_IndyRobotState  = n.advertise<indy_driver::IndyRobotState>("IndyRobotState", 10);

    // Message
    geometry_msgs::PoseStamped  cPose;
    geometry_msgs::Pose         msg_TargetPose;
    indy_driver::IndyRobotState msg_IndyRobotState;

    // Transform euler coordinate to quaternion coordinate
    tf::Quaternion quat_pose;

    ros::Rate loop_rate(10);

    while (ros::ok()){

        quat_pose.setRPY( targetPose[3] * M_PI/180, targetPose[4] * M_PI/180, targetPose[5] * M_PI/180 );
        quat_pose = quat_pose.normalize();
        
        msg_TargetPose.position.x      = targetPose[0];
        msg_TargetPose.position.y      = targetPose[1];
        msg_TargetPose.position.z      = targetPose[2];
        msg_TargetPose.orientation.x   = quat_pose.x();
        msg_TargetPose.orientation.y   = quat_pose.y();
        msg_TargetPose.orientation.z   = quat_pose.z();
        msg_TargetPose.orientation.w   = quat_pose.w();

        // check
        cPose = move_group.getCurrentPose();
        msg_IndyRobotState.move = checkMovingState(msg_TargetPose, cPose, 0.01);
        
        if (msg_IndyRobotState.move == 1){
            pub_IndyRobotState.publish(msg_IndyRobotState);
            
            move_group.setPoseTarget(msg_TargetPose);
                    
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;

            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);  
            if (success){
                move_group.move();
                success = 0;
            }
            else
                printf("failed!\n");
        }

        msg_IndyRobotState.move = 0;
        pub_IndyRobotState.publish(msg_IndyRobotState);
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}