#ifndef NKG_DOBOT_M1_HWI_H
#define NKG_DOBOT_M1_HWI_H

#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h>

#include "dobot_m1_hwi/dobot_m1_hardware.h"
#include "DobotDll/DobotDll.h"

using namespace hardware_interface;

namespace dobot_m1_hwi
{
class DobotM1HWI: public dobot_m1_hwi::DobotM1HW
{
	public:
		DobotM1HWI(ros::NodeHandle& nh);
		~DobotM1HWI();
		bool connectM1(const std::string&);
		void init();
		void update(const ros::TimerEvent& e);
		void read();
		void write(const ros::Duration& elapsed_time);
		void alarmCB(const std_msgs::Bool&);

	protected:
		ros::NodeHandle nh_;
		ros::Timer non_realtime_loop_;
		ros::Duration control_period_;
		ros::Duration elapsed_time_;
		double loop_hz_;
		boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

	private:
		bool settingParams();
		std::vector<double> _last_cmd;
		std::unique_ptr<ros::ServiceClient> _xyz_client;
		ros::Subscriber _alarm_sub;
		bool _ori, _first;	// dobot arm orientation 1: right, 0: left
};

}	// namespace

#endif
