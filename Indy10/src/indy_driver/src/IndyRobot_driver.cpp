#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <indy_driver/GripState.h>
#include <indy_driver/GripCommand.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "SocketHandler/IndyDCPSocket.h"
#include "TrajectoryDownloader/JointTrajectoryDownloader.h"

#define JOINT_DOF 6
#define RAD2DEG M_PI/180

int  DO_Pin0 = 10; 		// Gripper: DO PIN -  8, 9 
int  DO_Pin1 = 11; 		// Vaccum : DO PIN - 10, 11 
char DO_Val0 = 0;
char DO_Val1 = 0;

char gripChange = 0;
char gripOn 	= 0;

void readGripCommand(const indy_driver::GripCommand::ConstPtr& _msg){
    DO_Pin0 = _msg -> DO_Pin0;
    DO_Pin1 = _msg -> DO_Pin1;
    DO_Val0 = _msg -> DO_Val0;
    DO_Val1 = _msg -> DO_Val1;
}

void readGripState(const indy_driver::GripState::ConstPtr& _msg){
    gripChange  = _msg -> change;
    gripOn 		= _msg -> on;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "IndyRobot_driver");
	ros::NodeHandle n;

	ros::AsyncSpinner spinner(1);
    spinner.start();

	std::string robotName, ip;
	int port;
	
	// override IP/port with ROS params, if available
	ros::param::param<std::string>("robot_name", robotName, "");
	ros::param::param<std::string>("robot_ip_address", ip, SERVER_IP);
	ros::param::param<int>("~port", port, SERVER_PORT);

	// check for valid parameter values
	if (robotName.empty()) {
		ROS_ERROR("No valid robot's name found.  Please set ROS 'robot_name' param");
		return false;
	}
	if (ip.empty())	{
		ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
		return false;
	}
	if (port <= 0) {
		ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
		return false;
	}

	IndyDCPSocket indySocket;
	indySocket.init(robotName, ip, port);
	
	JointTrajectoryDownloader jmotionInterface(indySocket, JOINT_DOF);
	jmotionInterface.init();

	ros::Publisher  pub_JointState 	= n.advertise<sensor_msgs::JointState>("joint_states", 10);
	ros::Publisher  pub_GripState 	= n.advertise<indy_driver::GripState>("GripState", 10);
	ros::Subscriber sub_GripCommand = n.subscribe<indy_driver::GripCommand>("GripCommand", 10, readGripCommand);
	ros::Subscriber sub_GripState 	= n.subscribe<indy_driver::GripState>("GripState", 10, readGripState);
	
	sensor_msgs::JointState joint_state;
	indy_driver::GripState 	msg_GripState;

	double q[JOINT_DOF];

	ros::Rate loop_rate(10);
	while (ros::ok()) {

		if (indySocket.isWorking())	{
			Data data, gripData_D0, gripData_D1;
			unsigned int len;

			if (gripChange == 1){
				memcpy(gripData_D0.byte, &DO_Pin0, sizeof(int));
				memcpy(gripData_D0.byte + sizeof(int), &DO_Val0, sizeof(char));
				memcpy(gripData_D1.byte, &DO_Pin1, sizeof(int));
				memcpy(gripData_D1.byte + sizeof(int), &DO_Val1, sizeof(char));

				indySocket.sendCommand(402, gripData_D0, 5);
				indySocket.getFeedback(402, gripData_D0, len);
				indySocket.sendCommand(402, gripData_D1, 5);
				indySocket.getFeedback(402, gripData_D1, len);

				if 		(DO_Val0 == 0 && DO_Val1 == 1)	msg_GripState.on = 0;
				else if (DO_Val0 == 0 && DO_Val1 == 1)	msg_GripState.on = 1;

				msg_GripState.change = 0;
				pub_GripState.publish(msg_GripState);
				printf("= = = = = = = = G R I P = = = = = = = =\n");
			}

			indySocket.sendCommand(320, data, 0);
			indySocket.getFeedback(320, data, len);

			for (int i = 0; i < JOINT_DOF; i++)	{
				q[i] = data.double6dArr[i];
				}
		}

		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(JOINT_DOF);
		joint_state.position.resize(JOINT_DOF);

		joint_state.name[0] = "joint0";
		joint_state.position[0] = q[0] * RAD2DEG;
		joint_state.name[1] = "joint1";
		joint_state.position[1] = q[1] * RAD2DEG;
		joint_state.name[2] = "joint2";
		joint_state.position[2] = q[2] * RAD2DEG;
		joint_state.name[3] = "joint3";
		joint_state.position[3] = q[3] * RAD2DEG;
		joint_state.name[4] = "joint4";
		joint_state.position[4] = q[4] * RAD2DEG;
		joint_state.name[5] = "joint5";
		joint_state.position[5] = q[5] * RAD2DEG;

		pub_JointState.publish(joint_state);
		loop_rate.sleep();
	}
	indySocket.stop();

	return 0;
}