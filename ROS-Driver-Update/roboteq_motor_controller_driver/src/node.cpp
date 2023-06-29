#include "roboteq_motor_controller_driver_node.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <roboteq_motor_controller_driver/querylist.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver_0");
	
	if(argc < 1) {
		std::runtime_error("roboteq_motor_controller_driver node should be called with an arg specifying the node id (0, 1, 2, 3).");
	}

	int id = std::atoi(argv[1]);

	RoboteqDriver driver(id);
	ros::waitForShutdown();

	return 0;
}
