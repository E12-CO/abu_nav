// Area 1->3 navigator for Autonomous robot in the ABU R0b0c0n 2O24 contest.
// Coded by TinLethax from Robot Club Engineering KMITL.

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

/* Linux C low level stuffs */
#include <linux/gpio.h>

#include <malloc.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>

#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "vl53l1_platform.h"

/* end Linux C Low level stuffs */

// ROS2 
#include <rclcpp/rclcpp.hpp>
// Geometry mssage
#include <geometry_msgs/msg/twist.hpp>
// Nav message
#include <nav_msgs/msg/odometry.hpp>
// std message
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

// tf2 lib
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

// Define Feedback Loop time 
#define LOOP_TIME_MIL   100 // 100 millisec
#define LOOP_TIME_SEC	LOOP_TIME_MIL/1000 // Loop time in second

enum TOF_SELECT{
		TOF_FRONT = 0,
		TOF_LEFT,
		TOF_RIGHT
};

int gpio_fd;
struct gpiohandle_request rq;
struct gpiohandle_data tof_sel;

class abu_nav : public rclcpp::Node{
	public:
	
	// For Twist message
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistout;
	geometry_msgs::msg::Twist twist;
	
	// For robot odometry (rotation check)
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
	
	// For status flag
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPub;
	std_msgs::msg::String statusOut;
	
	// For team mode 
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr TeamMode;

	// For ToF measurement publishing
	rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr front_tof_pub;
	rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr left_tof_pub;
	rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr right_tof_pub;
	
	// Wall timer instant
	rclcpp::TimerBase::SharedPtr runner_;
	
	// VL53L1X stuffs
	int bus_id;// I2C Bus number
	// XSHUT GPIOs
	int front_tof_en_gpio, // XSHUT pin of front senser
		left_tof_en_gpio,  // XSHUT pin of left sensor
		right_tof_en_gpio; // XSHUT pin of right sensor
	 
	//int current_tof; 	   // Keep track of currently in-use ToF
	
	std::string twistout_topic;// ROS2 arg to select the twist topic name.
	
	int team, play_mode;
	// team :
	// team == 0 -> Stop mode
	// team == 1 -> Red team
	// team == 2 -> Blue team
	// play_mode :
	// play_mode == 0 -> Stop mode
	// play_mode == 1 -> Start mode
	// play_mode == 2 -> Retry mode
	enum TEAM_TYPE{
		TEAM_STOP=0,	
		TEAM_RED,
		TEAM_BLUE
	};
	
	enum PLAY_MODE{
		PLAY_STOP=0,
		PLAY_START,
		PLAY_RETRY
	};
	
	// Internal Finite State Machine
	int abu_fsm = 0;
	int abu_prev_fsm = -1;
	enum ABU_FSM{
		ABU_FSM_IDLE = 0,// Idle state
		ABU_FSM_SEL_TEAM,// Team select decision 
		ABU_FSM_SEL_PLAYMODE,// Play mode decision
		ABU_FSM_A1_A2,// Blind walk from Area 1 to Area 2
		ABU_FSM_A2_A2,// Slow down after entered Area 2, prepare to dock with the corner.
		ABU_FSM_A2_RED,// from Red team side Area 2 corner to slope 
		ABU_FSM_A2_BLU,// from Blue team side Area 2 corner to slope
		ABU_FSM_A2_A3_SLOPE,// Enter the slope from Area 2 to Area 3
		ABU_FSM_A3_A3,// Area 3 rotate-to-ball
		ABU_FSM_STOP = 255
	};
	
	
	// store team and playmode string received from callback
	std::string teammode_str;
	int got_teammode_flag;
	
	// Motion related stuffs
	// From Area 1 to Area2
	unsigned long blindwalk_period;// How long the Robot move to get from A1 to A2
	float blindwalk_velocity;// Velocity that the Robot use
	
	// From Area 2 to Area 2 corner
	float maxvel_A2A2;// Maximum walking speed for appracing Area 2 corner
	float minvel_A2A2;// Mininum walking speed before braking.
	float Kp_A2A2;// Proportional gain (Kp) for approaching Area 2 corner and brake	

	// From RED Area 2 leaving corner
	float normalvel_A2RED;// normal runnign speed 
	float slowvel_A2RED;// slowed down speed for entering area 3
	unsigned long Kt_A2RED;// braking time after front sensor detects openning
	
	// From BLUE Area 2 leaving corner
	float normalvel_A2BLU;// normal runnign speed 
	float slowvel_A2BLU;// slowed down speed for entering area 3
	unsigned long Kt_A2BLU;// braking time after front sensor detects openning
	
	// From Area 2 -> Slope to Area 3
	unsigned long blindwalk_period_2;// How long the Robot move to get from A2 to A3 over the slope
	float blindwalk_velocity_2;// Velocity that the Robot use
	
	unsigned long Blind_walk_ticks;// Keep track of time that robot is moving.

	float x_dist, y_dist;
	float x_dist_prev, y_dist_prev;
	
	double current_az;// current angular z pose from odometry message
	double before_rotate_az;
	
	// Laser measurement storage
	uint16_t Front_distance;
	uint16_t Left_distance;
	uint16_t Right_distance;
	
	uint8_t first_range[3] = {1};
	VL53L1X_Result_t Results;

        unsigned long A2Delay;
	
	abu_nav() : Node("ABUNavigator"){
		RCLCPP_INFO(this->get_logger(), "Starting ABU Navigation node");

		// Time-of-Flight sensor params
		declare_parameter("i2c_bus_number", 0);
		get_parameter("i2c_bus_number", bus_id);
		declare_parameter("TOF_front_EN", 0);
		get_parameter("TOF_front_EN", front_tof_en_gpio);
		declare_parameter("TOF_left_EN", 0);
		get_parameter("TOF_left_EN", left_tof_en_gpio);
		declare_parameter("TOF_right_EN", 0);
		get_parameter("TOF_right_EN", right_tof_en_gpio);
		
		// ROS2 topic params
		declare_parameter("twist_output_topic", "/cmd_vel");
		get_parameter("twist_output_topic", twistout_topic);
		
		// Motion related params
		// From Area 1 to Area 2, distance of 4.8 meters
		declare_parameter("motion_A1A2_period", 96);// In the multiple of 100ms
		get_parameter("motion_A1A2_period", blindwalk_period);
		declare_parameter("motion_A1A2_velocity", 0.5);
		get_parameter("motion_A1A2_velocity", blindwalk_velocity);
		
		// From Area 2 to Area 3, distance of  3.0 meters
		declare_parameter("motion_A2A3_period", 60);// In the multiple of 100ms
		get_parameter("motion_A2A3_period", blindwalk_period_2);
		declare_parameter("motion_A2A3_velocity", 0.5);
		get_parameter("motion_A2A3_velocity", blindwalk_velocity_2);
		
		declare_parameter("motion_A2A2_max_velocity", 0.7);
		get_parameter("motion_A2A2_max_velocity", maxvel_A2A2);
		declare_parameter("motion_A2A2_min_velocity", 0.2);
		get_parameter("motion_A2A2_min_velocity", minvel_A2A2);
		declare_parameter("motion_A2A2_Kp", 0.0004);
		get_parameter("motion_A2A2_Kp", Kp_A2A2);
		
		declare_parameter("motion_A2RED_normal_velocity", 0.3);
		get_parameter("motion_A2RED_normal_velocity", normalvel_A2RED);
		declare_parameter("motion_A2RED_slow_velocity", 0.3);
		get_parameter("motion_A2RED_slow_velocity", slowvel_A2RED);
		declare_parameter("motion_A2RED_Kt", 20);
		get_parameter("motion_A2RED_Kt", Kt_A2RED);
		
		declare_parameter("motion_A2BLU_normal_velocity", 0.3);
		get_parameter("motion_A2BLU_normal_velocity", normalvel_A2BLU);
		declare_parameter("motion_A2BLU_slow_velocity", 0.3);
		get_parameter("motion_A2BLU_slow_velocity", slowvel_A2BLU);
		declare_parameter("motion_A2BLU_Kt", 20);
		get_parameter("motion_A2BLU_Kt", Kt_A2BLU);

		// Initialize all GPIOs
		if(initGPIOs(front_tof_en_gpio, left_tof_en_gpio, right_tof_en_gpio) < 0){
			RCLCPP_ERROR(this->get_logger(), "Initializing GPIOs error! Quiting...");
			exit(1);
		}
		ToF_select(255);
		usleep(200000);//wait 200ms for sensor to reset
		// Initialize the VL53l1X sensors
		// 0 - TOF_FRONT -> Front sensor
		// 1 - TOF_LEFT -> Left sensor
		// 2 - TOF_RIGHT -> Right senser

		ToF_init();

		TeamMode = create_subscription<std_msgs::msg::String>(
			"/teammode",
			10,
			std::bind(
				&abu_nav::TeamModeCallback, 
				this,
				std::placeholders::_1)
			);
			
		subOdometry = create_subscription<nav_msgs::msg::Odometry>(
			"/odom",
			10,
			std::bind(
				&abu_nav::OdomCallback,
				this,
				std::placeholders::_1)
			);
		
		twistout = create_publisher<geometry_msgs::msg::Twist>(
			twistout_topic, 
			10);
			
		statusPub = create_publisher<std_msgs::msg::String>(
			"/abu_nav_stat",
			10);
			
		front_tof_pub = create_publisher<std_msgs::msg::UInt16>(
			"/front_tof",
			10);
		left_tof_pub = create_publisher<std_msgs::msg::UInt16>(
			"/left_tof",
			10);
		right_tof_pub = create_publisher<std_msgs::msg::UInt16>(
			"/right_tof",
			10);
			
		runner_ = this->create_wall_timer(
			std::chrono::milliseconds(LOOP_TIME_MIL),
			std::bind(&abu_nav::abu_runner, this)
			);	
			
		RCLCPP_INFO(this->get_logger(),"Robot Club KMITL : Starting ABU navigator node...");
	}
	
	// Receive command for Team selection and Start/Retry Mode
	void TeamModeCallback(const std_msgs::msg::String::SharedPtr team_mode){
		teammode_str = team_mode->data;
		got_teammode_flag = 1;
	}
	
	// Get Angular position from mecanum controller
	void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
		tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
		
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		x_dist = odom_msg->pose.pose.position.x;
		y_dist = odom_msg->pose.pose.position.y;
		current_az = yaw;
	}
	
	void abu_runner(){
		
		// For emergency

		if(got_teammode_flag == 1){
			abu_fsm = ABU_FSM_IDLE;
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twistout->publish(twist);
		}

		if(abu_prev_fsm != abu_fsm){// Do once when state was changed
			abu_prev_fsm = abu_fsm;
			switch(abu_prev_fsm){
			case ABU_FSM_IDLE:
				RCLCPP_INFO(this->get_logger(), "[FSM]Idle state");
			break;

			case ABU_FSM_SEL_TEAM:
				RCLCPP_INFO(this->get_logger(), "[FSM]Selecting team");
			break;
			
			case ABU_FSM_SEL_PLAYMODE:
				RCLCPP_INFO(this->get_logger(), "[FSM]Select Play mode");
			break;

			case ABU_FSM_A1_A2:
				RCLCPP_INFO(this->get_logger(), "[FSM]Blind walk from Area 1 to Area 2");
			break;

			case ABU_FSM_A2_A2:
				RCLCPP_INFO(this->get_logger(), "[FSM]Approaching Area 2 corner");
			break;

			case ABU_FSM_A2_RED:
				RCLCPP_INFO(this->get_logger(), "[FSM]leaving Area 2 Team red");
			break;

			case ABU_FSM_A2_BLU:
				RCLCPP_INFO(this->get_logger(), "[FSM]leaving Area 2 Team blue");
			break;

			case ABU_FSM_A2_A3_SLOPE:
				RCLCPP_INFO(this->get_logger(), "[FSM]From Area 2 to Area 3 slope");
			break;

			case ABU_FSM_A3_A3:
				RCLCPP_INFO(this->get_logger(), "[FSM]Area 3 rotation to Silo/Ball");
			break;

			case ABU_FSM_STOP:
				RCLCPP_INFO(this->get_logger(), "[FSM]STOP MODE");
			break;
			}
			
		}
		
		// Get Distance before the FSM task
		ToF_getAllDistance();
		
		switch(abu_fsm){
		case ABU_FSM_IDLE:// Idle state, Wait for new message
		{
			if(got_teammode_flag == 1){
				got_teammode_flag = 0;
				RCLCPP_INFO(this->get_logger(),"Got a message : %s", teammode_str.c_str());
				abu_fsm = ABU_FSM_SEL_TEAM;
			}
		}
		break;

		case ABU_FSM_SEL_TEAM:// parse team 
		{
			size_t pos = teammode_str.find(",");
			std::string parse_team = teammode_str.substr(0, pos);

			RCLCPP_INFO(this->get_logger(),"Received %s", parse_team.c_str());
			
			if((parse_team == "Red") || (parse_team == "red") || (parse_team == "RED")){
				team = TEAM_RED;
			}else if((parse_team == "Blue") || (parse_team == "blue") || (parse_team == "Blu") || 
					(parse_team == "blu") || (parse_team == "BLUE") || (parse_team == "BLU")){
				team = TEAM_BLUE;
			}else{
				team = TEAM_STOP;
			}

			RCLCPP_INFO(
				this->get_logger(),
				"Team %s", 
				(team == TEAM_STOP) ? "STOP" : ((team == TEAM_RED) ? "Red" : ((team == TEAM_BLUE) ? "Blue" : "STOP"))
				);
			
			teammode_str.erase(0, pos+1);// Delete the team, left with play mode
			
			abu_fsm = ABU_FSM_SEL_PLAYMODE;
			
			if(team == TEAM_STOP)
				abu_fsm = ABU_FSM_STOP;
			
		}
		break;
		
		case ABU_FSM_SEL_PLAYMODE:// parse playmode
		{
			play_mode = PLAY_STOP;

			if((teammode_str == "START") || (teammode_str == "Start") || (teammode_str == "start")){
				play_mode = PLAY_START;
			}else if((teammode_str == "RETRY") || (teammode_str == "Retry") || (teammode_str == "retry")){
				play_mode = PLAY_RETRY;
			}else{
				play_mode = PLAY_STOP;
			}
			
			RCLCPP_INFO(
				this->get_logger(),
				"Play mode %s", 
				(play_mode == PLAY_STOP) ? "STOP" : ((play_mode == PLAY_START) ? "Start" : ((play_mode == PLAY_RETRY) ? "Retry" : "STOP"))
				);
			
			if(play_mode == PLAY_RETRY){// Retry play mode don't do A1_A2 sequence
				abu_fsm = ABU_FSM_A2_A2;
			}else{
				abu_fsm = ABU_FSM_A1_A2;
			}

			if(play_mode == PLAY_STOP)
				abu_fsm = ABU_FSM_STOP;
		}
		break;
		
		case ABU_FSM_A1_A2:// Leave Area 1 and enter Area 2
		{
			Blind_walk_ticks++;

			if(Blind_walk_ticks > blindwalk_period){// Blind walk done
				Blind_walk_ticks = 0;// Reset to 0
				abu_fsm = ABU_FSM_A2_A2;
				// stop robot
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{
				twist.linear.x = -blindwalk_velocity;
				// A little bit of y component to make sure that the robot sticks to the wall
				if((Blind_walk_ticks * blindwalk_velocity * 100) < 2100){
					twist.linear.y = (team == TEAM_RED) ? -0.1 : ((team == TEAM_BLUE) ? 0.1 : 0.0);
				}else{// Stop commanding Y velocity before entering Slope.
					twist.linear.y = 0.0;
				}
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A2_A2:// Approaching the corner of Area 2 and brake
		{
			float calc_vel;
			
			RCLCPP_INFO(this->get_logger(), "Front Distance :%d", Front_distance);
			// Detect front sensor
			if(Left_distance > 1500)// If measurement is more than 1500mm (1.5m)
				Left_distance = 1500;// Cap it at 1.5 meters
			
			calc_vel = Front_distance * Kp_A2A2;// Calculate the velocity with the Kp
			
			// Speed Min Max hold
			if(calc_vel < minvel_A2A2)
					calc_vel = minvel_A2A2;
			if(calc_vel > maxvel_A2A2)
				calc_vel = maxvel_A2A2;
			
			if(Left_distance < 10){// 10mm (1.0cm) brake distance
				switch(team){
					case TEAM_RED:
						y_dist_prev = y_dist;
						abu_fsm = ABU_FSM_A2_RED;
						break;
					case TEAM_BLUE:
						y_dist_prev = y_dist;
						abu_fsm = ABU_FSM_A2_BLU;
						break;
					default:
						abu_fsm = ABU_FSM_IDLE;
						break;
				}
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{				
				twist.linear.x = -calc_vel;
				// A little bit of y component to make sure that the robot sticks to the wall
				//twist.linear.y = (team == TEAM_RED) ? -0.1 : ((team == TEAM_BLUE) ? 0.1 : 0.0);
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A2_RED:// Red team leaving Area 2 corner
		{
			RCLCPP_INFO(this->get_logger(), "Front Distance :%d", Front_distance);


			if(Left_distance > 150)
				A2Delay++;

			if(A2Delay > Kt_A2RED){// Apporaching 3.950 m
				A2Delay = 0;
				abu_fsm = ABU_FSM_A2_A3_SLOPE;
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{
				twist.linear.y = (Front_distance < 150) ? normalvel_A2RED : slowvel_A2RED;// Move from Right to Left
				if(abs(abs(y_dist) - abs(y_dist_prev)) < 2.5){// Stick to the wall
					twist.linear.x = -0.05;
				}else{// Stop commanding X velocity before entering Slope to Area 3.
					twist.linear.x = 0.0;
				}
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A2_BLU:// Blue team leaving Area 2 corner 
		{
			RCLCPP_INFO(this->get_logger(), "Front Distance :%d", Front_distance);
			
			if(Right_distance > 150)
				A2Delay++;

			if(A2Delay > Kt_A2BLU){
				A2Delay = 0;
				abu_fsm = ABU_FSM_A2_A3_SLOPE;
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{
				twist.linear.y = (Front_distance < 150) ? (-normalvel_A2BLU) : (-slowvel_A2BLU);// Move from Right to Left
				if(abs(abs(y_dist) - abs(y_dist_prev)) < 2.5){				
					twist.linear.x = -0.05;
				}else{// Stop commanding X velocity before entering Slope to Area 3.
					twist.linear.x = 0.0;
				}
			}
			twistout->publish(twist);
			
		}
		break;
		
		case ABU_FSM_A2_A3_SLOPE:// Leave Area 2 by entering the slope to Area 3
		{
			Blind_walk_ticks++;
			// TODO : velocity smoothing
			if(Blind_walk_ticks > blindwalk_period_2){// Blind walk done
				Blind_walk_ticks = 0;// Reset to 0
				// Stop Robot
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
				
				// Stamp current robot az
				before_rotate_az = current_az;
				
				abu_fsm = ABU_FSM_A3_A3;
			}else{
				twist.linear.x = -blindwalk_velocity_2;	
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A3_A3:// Rotate in-place 90 degree
		{
			Blind_walk_ticks++;// Reuse for time out.
			
			// Stop robot rotation after reached 90 degree rotation OR timed out after 5 seconds
			if((std::abs(current_az - before_rotate_az) > 1.57) || (Blind_walk_ticks > 50)){
				Blind_walk_ticks = 0;
				twist.angular.z = 0.0;
				abu_fsm = ABU_FSM_STOP;
			}else{
				twist.angular.z = (team == TEAM_RED) ? -0.3925 : ((team == TEAM_BLUE) ? 0.3925 : 0.0);// 0.3925 rad/s
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_STOP:// Stop mode
		{
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twist.angular.z = 0.0;
			twistout->publish(twist);
			statusOut.data = "DONE";
			statusPub->publish(statusOut);
			
			team = TEAM_STOP;
			play_mode = PLAY_STOP;
			abu_fsm = ABU_FSM_IDLE;
		}
		break;
		
		default:
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twistout->publish(twist);
			team = TEAM_STOP;
			play_mode = PLAY_STOP;
			abu_fsm = ABU_FSM_IDLE;
		break;
		}
		
		// Publish ToF data
		ToF_publishDistance();
	}
	
	int8_t initGPIOs(int f_tof_gpio, int l_tof_gpio, int r_tof_gpio){
		gpio_fd = open("/dev/gpiochip0", O_RDONLY);
		if(gpio_fd < 0){
			RCLCPP_ERROR(
				this->get_logger(),
				"Error! Can't open GPIO chip for GPIO control!"
			);
			close(gpio_fd);
			return -1;
		}
		
		rq.lineoffsets[0] 	= f_tof_gpio;
		rq.lineoffsets[1] 	= l_tof_gpio;
		rq.lineoffsets[2] 	= r_tof_gpio;
		rq.lines			= 3;
		rq.flags			= GPIOHANDLE_REQUEST_OUTPUT;
		
		int ret = ioctl(gpio_fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
		close(gpio_fd);
		if(ret == -1){
			RCLCPP_ERROR(
				this->get_logger(),
				"Error! Can't set line handle of IOCTL!"
			);
			return -1;
		}
	
		return 0;
	}
	
	void ToF_select(uint8_t ToF){
		int ret;
		switch(ToF){
		case TOF_FRONT:
			tof_sel.values[0] = 1;
			tof_sel.values[1] = 1;
			tof_sel.values[2] = 1;
		break;
		
		case TOF_LEFT:
			tof_sel.values[0] = 0;
			tof_sel.values[1] = 1;
			tof_sel.values[2] = 1;
		break;
		
		case TOF_RIGHT:
			tof_sel.values[0] = 0;
			tof_sel.values[1] = 0;
			tof_sel.values[2] = 1;
		break;

		case 255:// Case for reset all sensors
			tof_sel.values[0] = 0;
			tof_sel.values[1] = 0;
			tof_sel.values[2] = 0;
		break;

		default:
			tof_sel.values[0] = 0;
			tof_sel.values[1] = 0;
			tof_sel.values[2] = 0;
			RCLCPP_ERROR(
				this->get_logger(),
				"Error! Selecting the out off range sensor number"
			);
		}
		
		ret = ioctl(rq.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &tof_sel);
		if(ret == -1){
			RCLCPP_ERROR(
				this->get_logger(),
				"Error! Can't set line value of IOCTL!"
			);
			return;
		}
	}

	void ToF_init(){
		for(uint8_t i=2; i != 255; i--){
			switch(i){
			case TOF_FRONT:
				break;
				ToF_select(TOF_FRONT);
				usleep(20000);// 20ms delay for sensor startup
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_FRONT, bus_id, 0x29);
				if(VL53L1X_SensorInit(TOF_FRONT) < 0){
					RCLCPP_ERROR(this->get_logger(), "TOF_FRONT Init Error");
					exit(1);
				}
				VL53L1X_SetDistanceMode(TOF_FRONT, 2); /* 1=short, 2=long */
				VL53L1X_SetTimingBudgetInMs(TOF_FRONT, 90);
				VL53L1X_SetInterMeasurementInMs(TOF_FRONT, 52);
				VL53L1X_StartRanging(TOF_FRONT);
				RCLCPP_INFO(this->get_logger(), "Front ToF configured");
			break;
			
			case TOF_LEFT:
//				break;
				ToF_select(TOF_LEFT);
				usleep(20000);// 20ms delay for sensor startup
				if(VL53L1X_SetI2CAddress(TOF_FRONT, (0x2A << 1)) < 0){
					RCLCPP_ERROR(this->get_logger(), "Set TOF_LEFT I2C address Error");
					exit(1);
				}
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_LEFT, bus_id, 0x2A);
				VL53L1X_SensorInit(TOF_LEFT);
				VL53L1X_SetDistanceMode(TOF_LEFT, 2); /* 1=short, 2=long */
				VL53L1X_SetTimingBudgetInMs(TOF_LEFT, 90);
				VL53L1X_SetInterMeasurementInMs(TOF_LEFT, 52);
				VL53L1X_StartRanging(TOF_LEFT);
				RCLCPP_INFO(this->get_logger(), "Left ToF configured");
			break;
			
			case TOF_RIGHT:
//				break;
				ToF_select(TOF_RIGHT);
				usleep(20000);// 20ms delay for sensor startup
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_FRONT, bus_id, 0x29);
				if(VL53L1X_SetI2CAddress(TOF_FRONT, (0x2B << 1)) < 0){
					RCLCPP_ERROR(this->get_logger(), "Set TOF_RIGHT I2C address Error");
					exit(1);
				}
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_RIGHT, bus_id, 0x2B);
				VL53L1X_SensorInit(TOF_RIGHT);
				VL53L1X_SetDistanceMode(TOF_RIGHT, 2); /* 1=short, 2=long */
				VL53L1X_SetTimingBudgetInMs(TOF_RIGHT, 90);
				VL53L1X_SetInterMeasurementInMs(TOF_RIGHT, 52);
				VL53L1X_StartRanging(TOF_RIGHT);
				RCLCPP_INFO(this->get_logger(), "Right ToF configured");
			break;
			}
		}
		
	}
	
	uint16_t ToF_getDistance(uint8_t ToF){
		VL53L1X_GetResult(ToF, &Results);
		VL53L1X_ClearInterrupt(ToF);
//		if (first_range[ToF]) {
			/* very first measurement shall be ignored
			 * thus requires twice call
			 */
			VL53L1X_ClearInterrupt(ToF);
			first_range[ToF] = 0;
//		}
		//if(Results.Distance > 4400)
		//	return 4400;
		return Results.Distance;
	}
	
	void ToF_getAllDistance(){
//		Front_distance = ToF_getDistance(TOF_FRONT);
		Left_distance = ToF_getDistance(TOF_LEFT);
		Right_distance = ToF_getDistance(TOF_RIGHT);
	}
	
	void ToF_publishDistance(){
		std_msgs::msg::UInt16 tof_data;

		tof_data.data = Front_distance;
		front_tof_pub->publish(tof_data);

		tof_data.data = Left_distance;
		left_tof_pub->publish(tof_data);

		tof_data.data = Right_distance;
		right_tof_pub->publish(tof_data);
	}
	
};

int main (int argc, char **argv){
	rclcpp::init(argc, argv);
	auto nav_rbc {std::make_shared<abu_nav>()};
	rclcpp::spin(nav_rbc);
	
	// Exit cleanup.
	close(rq.fd);
	VL53L1X_UltraLite_Linux_I2C_Close(TOF_FRONT);
	VL53L1X_UltraLite_Linux_I2C_Close(TOF_LEFT);
	VL53L1X_UltraLite_Linux_I2C_Close(TOF_RIGHT);
	
	rclcpp::shutdown();
}
