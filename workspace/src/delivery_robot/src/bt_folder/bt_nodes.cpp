#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <queue>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

std::mutex battery_mutex;
std::mutex position_mutex;
std::mutex twist_mutex;

geometry_msgs::msg::Twist velocity;
geometry_msgs::msg::Point current_position;
geometry_msgs::msg::Quaternion quaternion_data;
float batteryState = 0.0;

struct DeliveryInfo {
	std::string food_id;
	double x;
	double y;
};

std::queue<DeliveryInfo> deliveries_list;

//This function asks the user to confirm that the food is on the robot

BT::NodeStatus IsFoodOnRobot() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food is on the Robot: ";
	
	while (std::stoi(number) < 1) {
		std::cin >> number;
		std::cout << std::endl;
	}

	return BT::NodeStatus::SUCCESS;
}

//This function checks if the robot's battery has more than 10% of power

BT::NodeStatus BatteryStatus() {
	battery_mutex.lock();
	std::cout << "Battery: " << batteryState << std::endl;
	if (batteryState <= 0.10) {
		battery_mutex.unlock();
		return BT::NodeStatus::FAILURE;
	}
	battery_mutex.unlock();

	return BT::NodeStatus::SUCCESS;
}

//This function is responsible for confirming that the food has been picked up from the robot

BT::NodeStatus IsFoodTaken() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food has been picked up: ";
	
	while (std::stoi(number) < 1) {
		std::cin >> number;
		std::cout << std::endl;
	}

	return BT::NodeStatus::SUCCESS;
}

//This function verifies if the robot is on the kitchen

BT::NodeStatus IsRobotOnKitchen() {
	if ((current_position.x >= -0.5 && current_position.x <= 0.5) && (current_position.y >= -0.5 && current_position.y <= 0.5)){
		return BT::NodeStatus::SUCCESS;
	}

	return BT::NodeStatus::FAILURE;
}

//This node is resposible for getting user input about the delivery

class RegisterDeliveryInfo : public BT::SyncActionNode {
	public:
		RegisterDeliveryInfo(const std::string& name)
		       : BT::SyncActionNode(name, {}) {}

		BT::NodeStatus tick() override {
			std::string food_id = "";
			std::string x = "0.0";
			std::string y = "0.0";
			std::string deliveries_num = "0";
			
			std::cout << "Type the number of food items that you want to deliver: " << std::endl;

			while (std::stoi(deliveries_num) <= 0) {
				std::cin >> deliveries_num;
			}

			for (int i = 0; i<std::stoi(deliveries_num); i++) {
				std::cout << "Type the food identification of your desire: ";
				std::cin >> food_id;

				std::cout << std::endl << "Now type the x coordinate of the room: ";
				std::cin >> x;

				std::cout << std::endl << "Now type the y coordinate of the room: ";
				std::cin >> y;
				std::cout << std::endl << std::endl;

				DeliveryInfo info = {food_id, std::stod(x), std::stod(y)};

				deliveries_list.push(info);
			}

			return BT::NodeStatus::SUCCESS;
		}
};

//This node is responsible for the movement of the robot using a PID controller

class GoToPatientRoom : public BT::StatefulActionNode {
	private:
		double targetX = 0.0;
		double targetY = 0.0;
		double current_linear_error_ = 0.0;
		double current_angular_error_ = 0.0;
		double linear_error_sum_ = 0.0;
		double angular_error_sum_ = 0.0;
		const double KP_ = 0.3;
		const double KI_ = 0.1;

		void calculate_error() {
			position_mutex.lock();
			double linear_error = sqrt((pow((this -> targetX - current_position.x), 2.0)) + pow((this -> targetY - current_position.y), 2.0));
			
			double target_angle = atan2(this -> targetY - current_position.y, this -> targetX - current_position.x);
			
			double current_angle = atan2(2.0*(quaternion_data.y*quaternion_data.x + 
					quaternion_data.w*quaternion_data.z), 1.0 - 2.0*(pow(quaternion_data.z, 2.0) +
					pow(quaternion_data.y, 2.0)));
			position_mutex.unlock();

			double angular_error = target_angle - current_angle;
			
			this -> current_linear_error_ = linear_error;
			this -> current_angular_error_ = angular_error;
			this -> linear_error_sum_ += linear_error;
			this -> angular_error_sum_ += angular_error;
		}

		double calculate_linear_velocity() {
			return (KP_*current_linear_error_) + (KI_*linear_error_sum_);
		}

		double calculate_angular_velocity() {
			return (KP_*current_angular_error_) + (KI_*angular_error_sum_);
		}
	
	public:
		GoToPatientRoom(const std::string& name)
			: BT::StatefulActionNode(name, {}) {}

		BT::NodeStatus onStart() override {
			DeliveryInfo data = deliveries_list.front();
			this -> targetX = data.x;
			this -> targetY = data.y;

			std::cout << "Initializing delivery process..." << std::endl;

			return BT::NodeStatus::RUNNING;
		}

		BT::NodeStatus onRunning() override {
			this -> calculate_error();
			
			if (this -> current_angular_error_ > 0.3) {
				twist_mutex.lock();
				velocity.angular.z = this -> calculate_angular_velocity();
				velocity.linear.x = 0.0;
				twist_mutex.unlock();

				std::cout << "Changing velocity" << std::endl;
			}else if (this -> current_linear_error_ > 0.1) {
				twist_mutex.lock();
				velocity.angular.z = 0.0;
				velocity.linear.x = this -> calculate_linear_velocity();
				twist_mutex.unlock();

				std::cout << "Changing velocity" << std::endl;
			}else{
				twist_mutex.lock();
				velocity.linear.x = 0.0;
				velocity.linear.z = 0.0;
				twist_mutex.unlock();
				
				std::cout << "Changing velocity" << std::endl;
				return BT::NodeStatus::SUCCESS;
			}

			return BT::NodeStatus::RUNNING;
		}

		void onHalted() override {
			std::cout << "The battery percentage has reached 10% or less. The delivery has been interrupted!" << std::endl; 
		}
};

//This node is responsible for displaying information about the delivery

class DisplayFoodInfo : public BT::SyncActionNode {
	public:
		DisplayFoodInfo(const std::string& name) 
			: BT::SyncActionNode(name, {}) {}

		BT::NodeStatus tick() override {
			DeliveryInfo data = deliveries_list.front();
			std::cout << "The food has arrived! " << std::endl << "Take the food with this id: " << data.food_id << std::endl;
			
			return BT::NodeStatus::SUCCESS;
		}
};

//This node updates data to set the robot for the next delivery, if there is any

class UpdateDeliveryInfo : public BT::SyncActionNode {
	public:
		UpdateDeliveryInfo(const std::string& name)
			: BT::SyncActionNode(name, {}) {}

		BT::NodeStatus tick() override {
			if (deliveries_list.empty()) {
				return BT::NodeStatus::SUCCESS;
			}

			deliveries_list.pop();
			
			return BT::NodeStatus::FAILURE;
		}
};

//This node is responsible for making the robot go back to the kitchen

class GoBackToKitchen : public BT::StatefulActionNode {
	private:
		double targetX = 0.0;
		double targetY = 0.0;
		double current_linear_error_ = 0.0;
		double current_angular_error_ = 0.0;
		double linear_error_sum_ = 0.0;
		double angular_error_sum_ = 0.0;
		const double KP_ = 0.3;
		const double KI_ = 0.1;

		void calculate_error() {
			position_mutex.lock();
			double linear_error = sqrt((pow((this -> targetX - current_position.x), 2.0)) + pow((this -> targetY - current_position.y),2.0));
			
			double target_angle = atan2(this -> targetY - current_position.y, this -> targetX - current_position.x);
			
			double current_angle = atan2(2.0*(quaternion_data.y*quaternion_data.x + 
					quaternion_data.w*quaternion_data.z), 1.0 - 2.0*(pow(quaternion_data.z, 2.0) +
					pow(quaternion_data.y, 2.0)));
			position_mutex.unlock();

			double angular_error = target_angle - current_angle;
			
			this -> current_linear_error_ = linear_error;
			this -> current_angular_error_ = angular_error;
			this -> linear_error_sum_ += linear_error;
			this -> angular_error_sum_ += angular_error;
		}

		double calculate_linear_velocity() {
			return (KP_*current_linear_error_) + (KI_*linear_error_sum_);
		}

		double calculate_angular_velocity() {
			return (KP_*current_angular_error_) + (KI_*angular_error_sum_);
		}
	
	public:
		GoBackToKitchen(const std::string& name)
			: BT::StatefulActionNode(name, {}) {}

		BT::NodeStatus onStart() override {
			std::cout << "Going back to kitchen..." << std::endl;

			return BT::NodeStatus::RUNNING;
		}

		BT::NodeStatus onRunning() override {
			this -> calculate_error();
			
			if (this -> current_angular_error_ > 0.3) {
				twist_mutex.lock();
				velocity.angular.z = this -> calculate_angular_velocity();
				velocity.linear.x = 0.0;
				twist_mutex.unlock();
			}else if (this -> current_linear_error_ > 0.1) {
				twist_mutex.lock();
				velocity.angular.z = 0.0;
				velocity.linear.x = this -> calculate_linear_velocity();
				twist_mutex.unlock();
			}else{
				twist_mutex.lock();
				velocity.linear.x = 0.0;
				velocity.linear.z = 0.0;
				twist_mutex.unlock();

				return BT::NodeStatus::SUCCESS;
			}

			return BT::NodeStatus::RUNNING;
		}

		void onHalted() override {
			std::cout << "The battery precentage reached 10% or less. The delivery has been interrupted!" << std::endl;
		}

};
