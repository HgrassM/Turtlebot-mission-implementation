#include <iostream>
#include <string>
#include <mutex>
#include <cmath>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/Pose.hpp"
#include "geometry_msgs/msg/Point.hpp"
#include "geometry_msgs/msg/Quaternion.hpp"
#include "nav_msgs/msg/Odometry.hpp"
#include "sensor_msgs/msg/BatteryState.hpp"

std::mutex global_mutex;
double targetX = 0.0;
double targetY = 0.0;
double target_angular = 0.0;
geometry_msg::msg::Point current_position = NULL;
geometry_msg::msg::Quaternion quaternion_data = NULL;
sensor_msgs::msg::BatteryState batteryState = NULL;

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
			output.y = convertFromString<double>(parts[1]);
			output.y = convertFromString<double>(parts[2]);
			return output;
		}	
	}
}

BT::NodeStatus IsFoodOnRobot() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food is on the Robot: ";
	
	while (stoi(number) < 1) {
		std::cin >> number;
		std::cout << endl;
	}

	return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BatteryStatus() {
	global_mutex.lock();
	float percentage = batteryState.percentage;
	global_mutex.unlock();

	if (percentage <= 0.10) {
		return BT::NodeStatus::FAILURE;
	}

	return BT::NodeStatus::SUCCESS;
}

class RegisterDeliveryInfo : public BT::SyncActionNode {
	public:
		RegisterDeliveryInfo(const std::string& name, const BT::NodeConfiguration& config)
		       : BT::SyncActionNode(name, config) {}

		static BT::PortsList providedPorts() {
			return {BT::OutputPort<DeliveryInfo>("delivery_info_output")};
		}

		BT::NodeStatus tick() override {
			std::string food_id = "";
			double x = 0.0;
			double y = 0.0;

			std::cout << "Type the food identification of your desire: ";
			std::cin >> food_id;

			std::cout << std::endl << "Now type the x coordinate of the room: ";
			std::cin >> x;

			std::cout << std::endl << "Now type the y coordinate of the room: ";
			std::cin >> y;
			std::cout << std::endl << std::endl;
			
			DeliveryInfo info = {food_id, x, y};
			BT::setOutput<DeliveryInfo>("delivery_info_output", info);

			return BT::NodeStatus::SUCCESS;
		}
};

class GetRoomInfo : public BT::SyncActionNode {
	public:
		GetRoomInfo(const std::string& name, const BT::NodeConfiguration config)
			: BT::SyncActionNode(name, config) {}

		static BT::PosrtsList providedPorts() {
			return {BT::InputPort<DeliveryInfo>("delivery_info_input")};
		}

		BT::NodeStatus tick() override {
			auto res = BT::getInput<DeliveryInfo>("delivery_info_input");

			if (!res) {
				throw RuntimeError("Error reading inputPort: ", res.error());
			}

			DeliveryInfo data = res.value();
			targetX = data.x;
			targetY = data.y;			
		}
};

class GoToPatientRoom : public BT::SyncActionNode {
	private:
		double current_linear_error_ = 0.0;
		double current_angular_error_ = 0.0;
		double linear_error_sum_ = 0.0;
		double angular_error_sum_ = 0.0;
		const double KP_ = 0.3;
		const double KI_ = 0.1;

		void calculate_error() {
			double linear_error = sqrt(((targetX - current_position.x)**2) + (targetY - current_position.y)**2);
			
			double target_angle = atan2(targetY - current_position.y, targetX - current_position.x);
			
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
		GoToPatientRoom(const std::string& name)
			: BT::SyncActionNode(name, {}) {}
		
		BT::NodeStatus tick() override {
			//TODO terminar de implementar o PID	
		}
};
