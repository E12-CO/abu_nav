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
// Geometry lib
#include <geometry_msgs/msg/twist.hpp>
// std message
#include <std_msgs/msg/string.hpp>

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
	
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr TeamMode;

	rclcpp::TimerBase::SharedPtr runner_;
	
	// VL53L1X stuffs
	int bus_id;// I2C Bus number
	
	
	
	int front_tof_en_gpio, // XSHUT pin of front senser
		left_tof_en_gpio,  // XSHUT pin of left sensor
		right_tof_en_gpio; // XSHUT pin of right sensor
	 
	int current_tof; 	   // Keep track of currently in-use ToF
	
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
	enum ABU_FSM{
		ABU_FSM_IDLE = 0,// Idle state
		ABU_FSM_SEL_TEAM,// Team select decision 
		ABU_FSM_SEL_PLAYMODE,// Play mode decision
		ABU_FSM_A1_A2,// Blind walk from Area 1 to Area 2
		ABU_FSM_A2_A2,// Slow down after entered Area 2, prepare to dock with the corner.
		ABU_FSM_A2_RED,// from Red team side Area 2 corner to slope 
		ABU_FSM_A2_BLU,// from Blue team side Area 2 corner to slope
		ABU_FSM_A2_A3_SLOPE,// Enter the slope from Area 2 to Area 3
		ABU_FSM_A3_A3,// For other stuffs on area 3
		ABU_FSM_STOP = 255
	};
	
	
	// store team and playmode string received from callback
	std::string teammode_str;
	int got_teammode_flag;
	
	// Motion related stuffs
	// From Area 1 to Area2
	unsigned long blindwalk_period;// How long the Robot move to get from A1 to A2
	float blindwalk_velocity;// Velocity that the Robot use
	
	float Kp_A2A2;// Proportional gain (Kp) for approaching Area 2 corner and brake
	float Kp_A2RED;// Proportional gain (Kp) for leaving RED Area 2 corner
	float Kp_A2BLU;// Proportional gain (Kp) for leaving BLUE Area 2 corner
	
	// From Area 2 -> Slope to Area 3
	unsigned long blindwalk_period_2;// How long the Robot move to get from A2 to A3 over the slope
	float blindwalk_velocity_2;// Velocity that the Robot use
	
	unsigned long Blind_walk_ticks;// Keep track of time that robot is moving.
	
	// Laser measurement storage
	uint16_t Front_distance;
	uint16_t Left_distance;
	uint16_t Right_distance;
	
	uint8_t first_range = 1;
	VL53L1X_Result_t Results;
	
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
		// From Area 1 to Area 2
		declare_parameter("motion_A1A2_period", 70);// In the multiple of 100ms
		get_parameter("motion_A1A2_period", blindwalk_period);
		declare_parameter("motion_A1A2_velocity", 0.5);
		get_parameter("motion_A1A2_velocity", blindwalk_velocity);
		
		// From Area 2 to Area 3
		declare_parameter("motion_A2A3_period", 42);// In the multiple of 100ms
		get_parameter("motion_A2A3_period", blindwalk_period_2);
		declare_parameter("motion_A2A3_velocity", 0.5);
		get_parameter("motion_A2A3_velocity", blindwalk_velocity_2);
		
		declare_parameter("motion_A2A2_Kp", 0.0001);
		get_parameter("motion_A2A2_Kp", Kp_A2A2);
		
		declare_parameter("motion_A2RED_Kp", 0.00011);
		get_parameter("motion_A2RED_Kp", Kp_A2RED);
		declare_parameter("motion_A2BLU_Kp", 0.00011);
		get_parameter("motion_A2BLU_Kp", Kp_A2BLU);

		// Initialize all GPIOs
		if(initGPIOs(front_tof_en_gpio, left_tof_en_gpio, right_tof_en_gpio) < 0){
			RCLCPP_ERROR(this->get_logger(), "Initializing GPIOs error! Quiting...");
			exit(1);
		}
		
		// Initialize the VL53l1X sensors
		// 0 - TOF_FRONT -> Front sensor
		// 1 - TOF_LEFT -> Left sensor
		// 2 - TOF_RIGHT -> Right senser
		for(uint8_t i=0; i < 3; i++){
			switch(i){
			case TOF_FRONT:
				ToF_select(TOF_FRONT);
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_FRONT, bus_id, 0x29);
				usleep(20000);// 20ms delay
				if(VL53L1X_SensorInit(TOF_FRONT) < 0){
					RCLCPP_ERROR(this->get_logger(), "TOF_FRONT Init Error");
					exit(1);
				}
				VL53L1X_SetDistanceMode(TOF_FRONT, 2); /* 1=short, 2=long */
				VL53L1X_SetTimingBudgetInMs(TOF_FRONT, 50);
				VL53L1X_SetInterMeasurementInMs(TOF_FRONT, 52);
				VL53L1X_StartRanging(TOF_FRONT);
				RCLCPP_INFO(this->get_logger(), "Front ToF configured");
			break;
			
			case TOF_LEFT:
				ToF_select(TOF_LEFT);
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_LEFT, bus_id, 0x29);
				if(VL53L1X_SetI2CAddress(TOF_FRONT, 0x2A) < 0){
					RCLCPP_ERROR(this->get_logger(), "Set TOF_LEFT I2C address Error");
					exit(1);
				}
				VL53L1X_UltraLite_Linux_I2C_change_device(TOF_LEFT, 0x2A);
				usleep(20000);// 20ms delay
				VL53L1X_SensorInit(TOF_LEFT);
				VL53L1X_SetDistanceMode(TOF_LEFT, 2); /* 1=short, 2=long */
				VL53L1X_SetTimingBudgetInMs(TOF_LEFT, 50);
				VL53L1X_SetInterMeasurementInMs(TOF_LEFT, 52);
				VL53L1X_StartRanging(TOF_LEFT);
				RCLCPP_INFO(this->get_logger(), "Left ToF configured");
			break;
			
			case TOF_RIGHT:
				ToF_select(TOF_RIGHT);
				VL53L1X_UltraLite_Linux_I2C_Init(TOF_RIGHT, bus_id, 0x29);
				if(VL53L1X_SetI2CAddress(TOF_FRONT, 0x2B) < 0){
					RCLCPP_ERROR(this->get_logger(), "Set TOF_RIGHT I2C address Error");
					exit(1);
				}
				VL53L1X_UltraLite_Linux_I2C_change_device(TOF_RIGHT, 0x2B);
				usleep(20000);// 20ms delay
				VL53L1X_SensorInit(TOF_RIGHT);
				VL53L1X_SetDistanceMode(TOF_RIGHT, 2); /* 1=short, 2=long */
				VL53L1X_SetTimingBudgetInMs(TOF_RIGHT, 50);
				VL53L1X_SetInterMeasurementInMs(TOF_RIGHT, 52);
				VL53L1X_StartRanging(TOF_RIGHT);
				RCLCPP_INFO(this->get_logger(), "Right ToF configured");
			break;
			}
		}
		
		TeamMode = create_subscription<std_msgs::msg::String>(
			"/teammode",
			10,
			std::bind(
				&abu_nav::TeamModeCallback, 
				this,
				std::placeholders::_1)
			);
		
		twistout = create_publisher<geometry_msgs::msg::Twist>(
			twistout_topic, 
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
	
	
	void abu_runner(){
		
		// For emergency
		if(got_teammode_flag == 1)
			abu_fsm = ABU_FSM_STOP;
		
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
			
			if((parse_team == "Red") || (parse_team == "red") || (parse_team == "RED")){
				team = TEAM_RED;
			}else if((parse_team == "Blue") || (parse_team == "blue") || (parse_team == "Blu") || (parse_team == "blu") || (parse_team == "BLUE") || (parse_team == "BLU")){
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
				switch(team){
				case TEAM_RED:
					abu_fsm = ABU_FSM_A2_RED;
					break;
				case TEAM_BLUE:
					abu_fsm = ABU_FSM_A2_BLU;
					break;
				default:
					abu_fsm = ABU_FSM_IDLE;
					break;
				}
			}else{
				abu_fsm = ABU_FSM_A1_A2;
			}
		}
		break;
		
		case ABU_FSM_A1_A2:// Leave Area 1 and enter Area 2
		{
			Blind_walk_ticks++;
			// TODO : velocity smoothing
			if(Blind_walk_ticks > blindwalk_period){// Blind walk done
				Blind_walk_ticks = 0;// Reset to 0
				abu_fsm = ABU_FSM_A2_A2;
				// stop robot
				twist.linear.x = 0.0;
				//twist.linear.y = 0.0;
			}else{
				twist.linear.x = blindwalk_velocity;
				// A little bit of y component to make sure that the robot sticks to the wall
				twist.linear.y = (team == TEAM_RED) ? 0.1 : ((team == TEAM_BLUE) ? -0.1 : 0.0);
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A2_A2:// Approaching the corner of Area 2 and brake
		{
			float calc_vel;
			
			Front_distance = ToF_getDistance(TOF_FRONT);
			
			// Detect front sensor
			if(Front_distance > 3000)// If measurement is more than 3000mm (3m)
				Front_distance = 3000;// Cap it at 3 meters
			
			calc_vel = Front_distance * Kp_A2A2;// Calculate the velocity with the Kp
			
			if(Front_distance < 50){// 50mm (5cm) brake distance
				switch(team){
					case TEAM_RED:
						abu_fsm = ABU_FSM_A2_RED;
						break;
					case TEAM_BLUE:
						abu_fsm = ABU_FSM_A2_BLU;
						break;
					default:
						abu_fsm = ABU_FSM_IDLE;
						break;
				}
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{
				twist.linear.x = calc_vel;
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A2_RED:// Red team leaving Area 2 corner
		{
			float calc_vel;
			
			Left_distance = ToF_getDistance(TOF_LEFT);
			
			// Detect Left sensor
			if(Left_distance > 4285)// If measurement is more than 3000mm (3m)
				Left_distance = 4285;// Cap it at 3 meters
			
			calc_vel = (4285-Left_distance) * Kp_A2RED;// Calculate the velocity with the Kp
			
			if(calc_vel < 0.1){// Small velocity
				abu_fsm = ABU_FSM_A2_A3_SLOPE;
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{
				twist.linear.y = -calc_vel;// Move from Left to right
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A2_BLU:// Blue team leaving Area 2 corner 
		{
			float calc_vel;
			
			Right_distance = ToF_getDistance(TOF_RIGHT);
			
			// Detect Right sensor
			if(Right_distance > 4285)// If measurement is more than 3000mm (3m)
				Right_distance = 4285;// Cap it at 3 meters
			
			calc_vel = (4285-Right_distance) * Kp_A2BLU;// Calculate the velocity with the Kp
			
			if(calc_vel < 0.1){// Small velocity
				abu_fsm = ABU_FSM_A2_A3_SLOPE;
				twist.linear.x = 0.0;
				twist.linear.y = 0.0;
			}else{
				twist.linear.y = calc_vel;// Move from Right to Left
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
				abu_fsm = ABU_FSM_A3_A3;
			}else{
				twist.linear.x = blindwalk_velocity_2;	
			}
			twistout->publish(twist);
		}
		break;
		
		case ABU_FSM_A3_A3:// Done
		{
			abu_fsm = ABU_FSM_STOP;
		}
		break;
		
		case ABU_FSM_STOP:// Stop mode
		{
			twist.linear.x = 0.0;
			twist.linear.y = 0.0;
			twistout->publish(twist);
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
			tof_sel.values[1] = 0;
			tof_sel.values[2] = 0;
		break;
		
		case TOF_LEFT:
			tof_sel.values[0] = 1;
			tof_sel.values[1] = 1;
			tof_sel.values[2] = 0;
		break;
		
		case TOF_RIGHT:
			tof_sel.values[0] = 1;
			tof_sel.values[1] = 1;
			tof_sel.values[2] = 1;
		break;
		
		default:
			RCLCPP_ERROR(
				this->get_logger(),
				"Error! Selecting the out off range sensor number"
			);
			return;
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
	
	uint16_t ToF_getDistance(uint8_t ToF){
		VL53L1X_GetResult(ToF, &Results);
		VL53L1X_ClearInterrupt(ToF);
		if (first_range) {
			/* very first measurement shall be ignored
			 * thus requires twice call
			 */
			VL53L1X_ClearInterrupt(ToF);
			first_range = 0;
		}
		return Results.Distance;
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
