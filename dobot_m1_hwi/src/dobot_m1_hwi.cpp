/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include <cmath>
#include <iostream>
#include <algorithm>
#include <sstream>
#include "dobot_m1_hwi/dobot_m1_hwi.h"
#include "nkg_demo_msgs/XYZ.h"
#include "moveit/macros/console_colors.h"

using namespace hardware_interface;

namespace dobot_m1_hwi
{
DobotM1HWI::DobotM1HWI(ros::NodeHandle& nh) : nh_(nh), _first(true){
	init();
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	nh_.param("generic_hw_control_loop/loop_hz", loop_hz_, 30.0);
	ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	non_realtime_loop_ = nh_.createTimer(update_freq, &DobotM1HWI::update, this);

	_xyz_client.reset(new ros::ServiceClient());
	*_xyz_client = nh_.serviceClient<nkg_demo_msgs::XYZ>("get_xyz");
	_xyz_client->waitForExistence(ros::Duration(5.0));

	_alarm_sub = nh_.subscribe("get_alarm", 5, &DobotM1HWI::alarmCB, this);
}

DobotM1HWI::~DobotM1HWI() {
	SetQueuedCmdStopExec();
	DisconnectDobot();
}

void DobotM1HWI::init() {
	// Get joint names
	nh_.getParam("hardware_interface/joints", joint_names_);
	if (joint_names_.empty()){
		ROS_ERROR_STREAM_NAMED("init", "No joints found on 'hardware_interface/joints'");
	}
	num_joints_ = joint_names_.size();

	// Resize vectors
	joint_position_.resize(num_joints_);
	joint_velocity_.resize(num_joints_);
	joint_effort_.resize(num_joints_);
	joint_position_command_.resize(num_joints_);
	joint_velocity_command_.resize(num_joints_);
	joint_effort_command_.resize(num_joints_);

	// record command
	_last_cmd.resize(num_joints_);

	// Initialize Controller 
	for (int i = 0; i < num_joints_; ++i) {
		// Create joint state interface
		JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		joint_state_interface_.registerHandle(jointStateHandle);

		// Create position joint interface
		JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
		position_joint_interface_.registerHandle(jointPositionHandle);

		// Create effort joint interface
		JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
		effort_joint_interface_.registerHandle(jointEffortHandle);

		// Create velocity joint interface
		JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
		velocity_joint_interface_.registerHandle(jointVelocityHandle);
	}

	registerInterface(&joint_state_interface_);
	registerInterface(&position_joint_interface_);
	registerInterface(&effort_joint_interface_);
	registerInterface(&velocity_joint_interface_);
}

bool DobotM1HWI::connectM1(const std::string& port){
	int result = ConnectDobot(port.c_str(), 115200,NULL,NULL);
	if (result == DobotConnect_NoError){
		if(ClearAllAlarmsState() != DobotCommunicate_NoError)
			ROS_ERROR("Clearing alarm fails!");
		else if(SetQueuedCmdClear() != DobotCommunicate_NoError)
			ROS_ERROR("Clearing queue fails!");
		else if (SetQueuedCmdStartExec() != DobotCommunicate_NoError)
			ROS_ERROR("Starting queue fails!");
		else if (!settingParams())
			ROS_ERROR("Setting params fails!");
		else{
/*
			// Initial position
			uint64_t useless_idx;
			PTPCmd cmd;
			cmd.ptpMode = PTPMode::PTPMOVJANGLEMode; 
			cmd.x = 0;
			cmd.y = 0;
			cmd.z = 125;
			cmd.r = 0;
			int result = SetPTPCmd(&cmd, false, &useless_idx);

			if (result != DobotCommunicate_NoError)
				ROS_WARN("Initialization fails!");
*/
			ROS_INFO(MOVEIT_CONSOLE_COLOR_CYAN "DobotM1 connects" MOVEIT_CONSOLE_COLOR_RESET);
			return true;
		}
	}
	else if (result == DobotConnect_NotFound)
		ROS_ERROR("DobotM1 not found!");
	else if (result == DobotConnect_Occupied)
		ROS_ERROR("DobotM1 occupied!");
	else
		ROS_ERROR("Unknown connection issue!");
	return false;
}

bool DobotM1HWI::settingParams(){
	uint64_t useless_idx;
	bool queued = false;

	// setting PTP ratio
	PTPCommonParams ptp_common_params;
	ptp_common_params.velocityRatio = 50;
	ptp_common_params.accelerationRatio = 50;
	if (SetPTPCommonParams(&ptp_common_params, queued, &useless_idx) != DobotCommunicate_NoError)
		return false;

	// setting PTP velocity & acceleration values
	PTPJointParams joint_params;
	joint_params.velocity[0] = 300;
	joint_params.velocity[1] = 300;
	joint_params.velocity[2] = 300;
	joint_params.velocity[3] = 300;
	joint_params.acceleration[0] = 30000;
	joint_params.acceleration[1] = 30000;
	joint_params.acceleration[2] = 30000;
	joint_params.acceleration[3] = 30000;
	if (SetPTPJointParams(&joint_params, queued, &useless_idx) != DobotCommunicate_NoError)
		return false;

	// setting CP params
	CPParams cp_params;
	cp_params.planAcc = 300;
	cp_params.junctionVel = 300;
	cp_params.period = 1.0/loop_hz_;
	cp_params.realTimeTrack = 1;
	if (SetCPParams(&cp_params, queued, &useless_idx) != DobotCommunicate_NoError)
		return false;

	return true;
}

void DobotM1HWI::update(const ros::TimerEvent& e) {
	elapsed_time_ = ros::Duration(e.current_real - e.last_real);
	read();
	controller_manager_->update(ros::Time::now(), elapsed_time_);
	write(elapsed_time_);
}

void DobotM1HWI::read() {
	Pose pose;
	if (GetPose(&pose) == DobotCommunicate_NoError){
		joint_position_[0]=(pose.jointAngle[2]/1000);
		joint_position_[1]=(pose.jointAngle[0]*M_PI/180);
		joint_position_[2]=(pose.jointAngle[1]*M_PI/180);
		joint_position_[3]=(pose.jointAngle[3]*M_PI/180);
//		ROS_ERROR("X: %f, Y: %f, Z: %f, R: %f", pose.x, pose.y, pose.z, pose.r);
//		ROS_ERROR("Jz: %f, J1: %f, J2: %f, Jr: %f", pose.jointAngle[2], pose.jointAngle[0], pose.jointAngle[1], pose.jointAngle[3]);
		if (_first){	// get initial arm orientation
			_ori = (joint_position_[2] > 0);
			uint64_t useless_idx;
			if (_ori)
				SetArmOrientation(ArmOrientation::RightyArmOrientation,false,&useless_idx);
			else
				SetArmOrientation(ArmOrientation::LeftyArmOrientation,false,&useless_idx);
			_first = false;
		}
	}
	else
		ROS_WARN("GetPose timeout!");
}

void DobotM1HWI::write(const ros::Duration& elapsed_time) {
	bool same = std::equal(_last_cmd.begin(), _last_cmd.end(), joint_position_command_.begin());
	if (!same){
		uint64_t idx;
		bool queued = true;

		// check left-rigth arm switching
		bool change = _ori ^ (joint_position_command_[2] > 0);
		if (change){	// TODO synchronization with moveit
			ROS_INFO(MOVEIT_CONSOLE_COLOR_CYAN "Orientation changed" MOVEIT_CONSOLE_COLOR_RESET);

			PTPCmd cmd;
			cmd.ptpMode = PTPMode::PTPMOVJANGLEMode; 
			cmd.x =  joint_position_command_[1]*180/M_PI;
			cmd.y =  joint_position_command_[2]*180/M_PI;
			cmd.z =  joint_position_command_[0]*1000;
			cmd.r =  joint_position_command_[3]*180/M_PI;
			int result = SetPTPCmd(&cmd, queued, &idx);
			if (result == DobotCommunicate_BufferFull)
				ROS_WARN("Buffer full!");
			else if (result == DobotCommunicate_Timeout)
				ROS_WARN("PTPCmd timeout!");
			else{
				_last_cmd.assign(joint_position_command_.begin(), joint_position_command_.end());
				_ori = !_ori;
				if (_ori)
					SetArmOrientation(ArmOrientation::RightyArmOrientation, queued, &idx);
				else
					SetArmOrientation(ArmOrientation::LeftyArmOrientation, queued, &idx);
			}
		}
		else{
			nkg_demo_msgs::XYZ::Request  req;
			nkg_demo_msgs::XYZ::Response res;
			req.joints.assign(joint_position_command_.begin(), joint_position_command_.begin()+4);
			if(_xyz_client->call(req,res)){			
				CPCmd cmd;
				cmd.cpMode = CPMode::CPAbsoluteMode;
				cmd.x = res.cartesian[1]*1000.0 - 100;
				cmd.y = -res.cartesian[0]*1000.0;
				cmd.z = res.cartesian[2]*1000.0 - 47;
//				ROS_WARN("%f, %f, %f", res.cartesian[0],res.cartesian[1],res.cartesian[2]);
				int result = SetCPCmd(&cmd, queued, &idx);

				if (result == DobotCommunicate_BufferFull)
					ROS_WARN("Buffer full!");
				else if (result == DobotCommunicate_Timeout)
					ROS_WARN("CPCmd timeout!");
				else
					_last_cmd.assign(joint_position_command_.begin(), joint_position_command_.end());
			}
			else
				ROS_ERROR("getXYZ fails!");
		}
/*/
		PTPCmd cmd;
		cmd.ptpMode = PTPMode::PTPMOVJANGLEMode; 
		cmd.x =  joint_position_command_[1]*180/M_PI;
		cmd.y =  joint_position_command_[2]*180/M_PI;
		cmd.z =  joint_position_command_[0]*1000;
		cmd.r =  joint_position_command_[3]*180/M_PI;
		int result = SetPTPCmd(&cmd, queued, &idx);
		if (result == DobotCommunicate_BufferFull)
			ROS_WARN("Buffer full!");
		else if (result == DobotCommunicate_Timeout)
			ROS_WARN("PTPCmd timeout!");
		else
			_last_cmd.assign(joint_position_command_.begin(), joint_position_command_.end());
*/
	}
}

void DobotM1HWI::alarmCB(const std_msgs::Bool& flag){
	if (flag.data){
		uint32_t len, max_len = 32;
		uint8_t alarms[max_len];

		if (GetAlarmsState(alarms, &len, max_len) != DobotCommunicate_NoError)
			ROS_ERROR("Get alarm fails!");
		else
			ROS_WARN("Get alarm succeds!");
		
		uint64_t exec_idx = 0;
		GetQueuedCmdCurrentIndex(&exec_idx);
		ROS_ERROR("Execute to %lu", exec_idx);

		int idx = 0;
		bool has = false;
		for (int i=0; i<len; ++i){
			if (alarms[i] != 0){
				switch(alarms[i]){
					case 1  :				break;
					case 2  :	idx += 1;	break;
					case 4  :	idx += 2;	break;
					case 8  :	idx += 3;	break;
					case 16 :	idx += 4;	break;
					case 32 :	idx += 5;	break;
					case 64 :	idx += 6;	break;
					case 128:	idx += 7;	break;
					default: ROS_ERROR("More than 1 alarm!"); break;
				}
				has = true;
				break; // break for loop
			}
			else
				idx += 8;
		}
		if (!has){
			ROS_INFO(MOVEIT_CONSOLE_COLOR_CYAN "No alarms" MOVEIT_CONSOLE_COLOR_RESET);
			return;
		}
		std::stringstream ss;
		ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << idx;
		ROS_ERROR("Alarm: %s", ss.str().c_str());
	}
}

} // namespace dobot_m1_hwi
