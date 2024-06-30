#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Odometry.hpp" 
#include "bt_folder/bt_nodes.cpp"

class ExecutionNode : public rclcpp::Node {
	private:
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;

		void odom_callback(const nav_msgs::msg::Odometry& odom_msg) {
			position_mutex.lock();
			current_position = odom_msg.pose.pose.position;
			quaternion_data = odom_msg.pose.pose.orientation;
			position_mutex.unlock();
		}

		void battery_callback(const sensor_msgs::msg::BatteryState& battery_msg) {
			battery_mutex.lock();
			batteryState = battery_msg.percentage;
			battery_mutex.unlock();
		}

	public:
		ExecutionNode() : Node("execution_node"), count_(0) {
			odom_subscriber_ = this -> create_subscription<nav_msgs::msg::Odometry>(
					"odom", std::bind(&ExecutionNode::odom_callback, this, _1));
			battery_subscriber_ = this -> create_subscription<sensor_msgs::msg::BatteryState>(
					"battery_state", std::bind(&ExecutionNode::battery_callback, this, _1));
		}
};
