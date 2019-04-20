/*
* @file main.cpp
* @brief Runs control algorithm to travel to a specified GPS waypoint	 
*
*/

#include <ros/ros.h>

int main(int argc char** argv) {
	ros::init(argc, argv, "flight_path_node");
	ros::NodeHandle nh;

	ros::Rate rate(10.0);
	while(ros::ok() && current_state.connected) {

	}

}
