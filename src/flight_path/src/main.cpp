/*
* @file main.cpp
* @brief Runs control algorithm to travel to a specified GPS waypoint	 
*
*/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#define FLIGHT_ALTITUDE 1.5f;

mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_position;
static bool global_position_received = false;

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	global_position = *msg;	
	global_position_received = true;
	//ROS_INFO("Global position: [%.8f, %.8f, %.4f]", msg->latitude, msg->longitude, msg->altitude);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "flight_path_node");
	ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
	ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, globalPositionCallback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher global_target_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", 10);

	ros::Publisher move_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


	ros::Publisher mav_att_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 100);
	ros::Publisher mav_thr_pub = nh.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 100);
	//ros::Subscriber mav_imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 100, mav_imu_data_callback);

	ros::Rate rate(20.0);
	// Wait for the flight controller to connect
	while(ros::ok() && current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	// Wait to get first global position message
	/*while(ros::ok() && !global_position_received) {
		ROS_INFO("Waiting for GPS signal...");
		ros::spinOnce();
		rate.sleep();
	}*/

	// Initialize waypoint to target to current position
	mavros_msgs::GlobalPositionTarget global_waypoint_target;
	global_waypoint_target.latitude = global_position.latitude;
	global_waypoint_target.longitude = global_position.longitude;
	global_waypoint_target.altitude = global_position.altitude;

	geometry_msgs::TwistStamped move_msg;
	move_msg.twist.linear.z = 1.0f;
	
	// Send message to initialize
	for(unsigned int i = 0; i < 20; i++) {
		global_position.header.stamp = ros::Time::now();
		global_target_pub.publish(global_waypoint_target);
		ros::spinOnce();
		rate.sleep();
	}

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	ros::Time last_request = ros::Time::now();

    // Change to offboard mode and arm
	ROS_INFO("Attempting to set proper mode and arm");
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
			ROS_INFO("%s", current_state.mode.c_str());
            if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
				break;
            } else {
				ROS_INFO_THROTTLE(1, "Mode not yet changed");
			}
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
	while(ros::ok()) {
		if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
			if(arming_client.call(arm_cmd) &&
				arm_cmd.response.success){
				ROS_INFO("Vehicle armed");
				break;
			}
			else {
				ROS_INFO("Not yet armed");
			}
			last_request = ros::Time::now();
		}
		ros::spinOnce();
		rate.sleep();
	}

	
	//move_pub.publish(move_msg);
	/*std_msgs::Float64 cmd_thr;
	geometry_msgs::PoseStamped cmd_att;
	cmd_att.pose.position.x = 0.0f;
	cmd_att.pose.position.y = 0.0f;
	cmd_att.pose.position.z = 0.0f;
	tf::Quaternion mav_orientation = tf::createQuaternionFromRPY(0.5235, 0.5235, 0.0);
	cmd_att.pose.orientation.x = mav_orientation.x();
	cmd_att.pose.orientation.y = mav_orientation.y();
	cmd_att.pose.orientation.z = mav_orientation.z();
	cmd_att.pose.orientation.w = mav_orientation.w();
	cmd_thr.data = 0.9f;
	for(int i = 0; i < 500; i++) {
		cmd_att.header.stamp = ros::Time::now();
		cmd_att.header.seq = i;
		mav_att_pub.publish(cmd_att);
		mav_thr_pub.publish(cmd_thr);
		rate.sleep();
	}
	ROS_INFO("Finished sending throttle/attitude commands");*/
	
	ROS_INFO("Ready to target waypoints");
	
	global_waypoint_target.altitude = global_position.altitude + FLIGHT_ALTITUDE;
	
	geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Go to the first waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");


    // Go to the second waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 1;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // Send setpoints for 10 seconds
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("second way point finished!");

	// Land the drone
	mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    while (!(land_client.call(land_cmd) && land_cmd.response.success)){

      ROS_INFO("Land Drone");
      ros::spinOnce();
      rate.sleep();
    }
	

	// Loop until program is killed
	while(ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
