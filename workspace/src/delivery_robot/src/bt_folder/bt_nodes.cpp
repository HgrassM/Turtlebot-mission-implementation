#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <queue>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/Twist.hpp"
#include "geometry_msgs/msg/Pose.hpp"
#include "geometry_msgs/msg/Point.hpp"
#include "geometry_msgs/msg/Quaternion.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "sensor_msgs/msg/BatteryState.hpp"

std::mutex global_mutex;

geometry_msg::msg::Twist velocity;
geometry_msg::msg::Point current_position = NULL;
geometry_msg::msg::Quaternion quaternion_data = NULL;
sensor_msgs::msg::BatteryState batteryState = NULL;

std::queue<DeliveryInfo> deliveries_list;

struct DeliveryInfo {
	std::string food_id;
	double x;
	double y;
};

namespace BT {
	template <> inline DeliveryInfo convertFromString(StringView) {
		auto parts = BT::splitString(str, ';');

		if (parts.size() != 3) {
			throw BT::RunTimeError("Invalid input!");
		}else{
			DeliveryInfo output;
			output.food_id = convertFromString<std::String>(parts[0]);
			output.x = convertFromString<double>(parts[1]);
			output.y = convertFromString<double>(parts[2]);
			return output;
		}	
	}
}

//This function asks the user to confirm that the food is on the robot

BT::NodeStatus IsFoodOnRobot() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food is on the Robot: ";
	
	while (std::stoi(number) < 1) {
		std::cin >> number;
		std::cout << endl;
	}

	return BT::NodeStatus::SUCCESS;
}

//This function checks if the robot's battery has more than 10% of power

BT::NodeStatus BatteryStatus() {
	global_mutex.lock();
	float percentage = batteryState.percentage;
	global_mutex.unlock();

	if (percentage <= 0.10) {
		return BT::NodeStatus::FAILURE;
	}

	return BT::NodeStatus::SUCCESS;
}

//This function is responsible for confirming that the food has been picked up from the robot

BT::NodeStatus IsFoodTaken() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food has been picked up: ";
	
	while (std::stoi(number) < 1) {
		std::cin >> number;
		std::cout << endl;
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
		RegisterDeliveryInfo(const std::string& name, const BT::NodeConfiguration& config)
		       : BT::SyncActionNode(name, config) {}

		static BT::PortsList providedPorts() {
			return {BT::OutputPort<DeliveryInfo>("delivery_info_output")};
		}

		BT::NodeStatus tick() override {
			std::string food_id = "";
			std::string x = 0.0;
			std::string y = 0.0;
			std::string deliveries_num = "0";
			
			std::cout << "Type the number of food items that you want to deliver: " << endl;

			while (std::stoi(deliveries_num) <= 0) {
				std::cin >> deliveries_num;
			}

			for (int i = 0; i<deliveries_num; i++) {
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
			
			BT::setOutput<DeliveryInfo>("delivery_info_output", deliveries_list.front());

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
			double linear_error = sqrt(((this -> targetX - current_position.x)**2) + (this -> targetY - current_position.y)**2);
			
			double target_angle = atan2(this -> targetY - current_position.y, this -> targetX - current_position.x);
			
			global_mutex.lock();
			double current_angle = atan2(2.0*(quaternion_data.y*quaternion_data.x + 
					quaternion_data.w*quaternion_data.z), 1.0 - 2.0*(quaternion_data.z**2 +
					quaternion_data.y**2));
			global_mutex.unlock();

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
		GoToPatientRoom(const std::string& name, const BT::NodeConfiguration& config)
			: BT::StatefulActionNode(name, config) {}
		
		BT::PortsList providedPorts() {
			return {BT::InputPort<DeliveryInfo>("target_info")}
		}

		BT::NodeStatus onStart() override {
			auto res = BT::getInput<DeliveryInfo>("target_info");
			
			if (!res) {
				throw RuntimeError("Error reading inputPort: ", res.error());
			}
			
			DeliveryInfo data = res.value();
			this -> targetX = data.x;
			this -> targetY = data.y;

			std::cout << "Initializing delivery process..." << std::endl;

			return BT::NodeStatus::RUNNING;
		}

		BT::NodeStatus onRunning() override {
			this -> calculate_error();
			
			if (this -> current_angular_error_ > 0.3) {
				velocity.angular.z = this -> calculate_angular_velocity();
				velocity.linear.x = 0.0;
			}else if (this -> current_linear_error_ > 0.1) {
				velocity.angular.z = 0.0;
				velocity.linear.x = this -> calculate_linear_velocity();
			}else{
				velocity.linear.x = 0.0;
				velocity.linear.z = 0.0;
				
				return BT::NodeStatus::SUCCESS;
			}

			return BT::NodeStatus::RUNNING;
		}
};

//This node is responsible for displaying information about the delivery

class DisplayFoodInfo : BT::SyncActionNode {
	public:
		DisplayFoodInfo(const std::string& name, const BT::NodeConfiguration& config) 
			: BT::SyncActionNode(name, config) {}

		BT::PortsList providedPorts() {
			return {BT::InputPort<DeliveryInfo>("info_for_display")};
		}

		BT::NodeStatus tick() override {
			auto res = BT::getInput<DeliveryInfo>("info_for_display");

			if (!res) {
				throw RuntimeError("Error reading inputPort: ", res.error());
			}

			DeliveryInfo data = res.value();
			std::cout << "The food has arrived! " << std::endl << "Take the food with this id: " << data.food_id << std::endl;
			
			return BT::NodeStatus::SUCCESS;
		}
};

//This node updates data to set the robot for the next delivery, if there is any

class UpdateDeliveryInfo : BT::SyncActionNode {
	public:
		UpdateDeliveryInfo(const std::string& name, BT::NodeConfiguration& config)
			: BT::SyncActionNode(name, config) {}
		
		BT::PortsList providedPorts() {
			return {BT::OutputPort<DeliveryInfo>("update_info")};
		}

		BT::NodeStatus tick() override {
			if (deliveries_list.empty()) {
				return BT::NodeStatus::SUCESS;
			}

			deliveries_list.pop();
			BT::setOutput<DeliveryInfo>("update_info", deliveries_list.front());
			
			return BT::NodeStatus::FAILURE;
		}
};

//This node is responsible for making the robot go back to the kitchen

class GoBackToKitchen : BT::StatefulActionNode {
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
			double linear_error = sqrt(((this -> targetX - current_position.x)**2) + (this -> targetY - current_position.y)**2);
			
			double target_angle = atan2(this -> targetY - current_position.y, this -> targetX - current_position.x);
			
			global_mutex.lock();
			double current_angle = atan2(2.0*(quaternion_data.y*quaternion_data.x + 
					quaternion_data.w*quaternion_data.z), 1.0 - 2.0*(quaternion_data.z**2 +
					quaternion_data.y**2));
			global_mutex.unlock();

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
				velocity.angular.z = this -> calculate_angular_velocity();
				velocity.linear.x = 0.0;
			}else if (this -> current_linear_error_ > 0.1) {
				velocity.angular.z = 0.0;
				velocity.linear.x = this -> calculate_linear_velocity();
			}else{
				velocity.linear.x = 0.0;
				velocity.linear.z = 0.0;
				
				return BT::NodeStatus::SUCCESS;
			}

			return BT::NodeStatus::RUNNING;
		}

};
