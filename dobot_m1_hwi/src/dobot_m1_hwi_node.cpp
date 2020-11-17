/*********************************************************************
 *	Software License Agreement (BSD License)
 *
 *	All rights reserved.
 *
 *	Author: 	Yu-Wen Chen
 *	Date: 		2020/09
 *	Version: 	1.0.0
 *********************************************************************/
#include "dobot_m1_hwi/dobot_m1_hwi.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dobot_m1_hwi");
    ros::CallbackQueue ros_queue;
    ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
	ros::AsyncSpinner spinner(0, &ros_queue);
	spinner.start();

	std::string port("/dev/ttyUSB0");
    dobot_m1_hwi::DobotM1HWI rhi(nh);
	if (!rhi.connectM1(port))
		return -1;

	ros::waitForShutdown();
    return 0;
}
