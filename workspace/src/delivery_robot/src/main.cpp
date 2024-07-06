#include <chrono>
#include <thread>
#include <stack>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "bt_folder/bt_nodes.cpp"

using namespace std::chrono_literals;

void tick_tree(BT::BehaviorTreeFactory tree_factory) {
	auto tree = tree_factory.createTreeFromFile("src/delivery_robot/src/bt_folder/deliveryTree.xml");
			
	while (true) {
		tree.tickRootWhileRunning();
	}
}

class ExecutionNode : public rclcpp::Node {
	private:
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		size_t count_;
		std::stack<std::thread> thread_stack_;

		void twist_callback() {
			twist_mutex.lock();
			twist_publisher_ -> publish(velocity);
			twist_mutex.unlock();
		}

		void odom_callback(const nav_msgs::msg::Odometry& odom_msg) const{
			position_mutex.lock();
			current_position = odom_msg.pose.pose.position;
			quaternion_data = odom_msg.pose.pose.orientation;
			position_mutex.unlock();
		}

		void battery_callback(const sensor_msgs::msg::BatteryState& battery_msg) const{
			battery_mutex.lock();
			batteryState = battery_msg.percentage;
			battery_mutex.unlock();
		}

		void laser_callback(const sensor_msgs::msg::LaserScan& laser_msg) const{
			laser_mutex.lock();
			laser_data = laser_msg;
			laser_mutex.unlock();
		}	

	public:
		ExecutionNode() : Node("execution_node"), count_(0) {
			
			//Initializing variables
			odom_subscriber_ = rclcpp::Node::create_subscription<nav_msgs::msg::Odometry>(
					"odom", 10, std::bind(&ExecutionNode::odom_callback, this, std::placeholders::_1));
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
			battery_subscriber_ = rclcpp::Node::create_subscription<sensor_msgs::msg::BatteryState>(
					"battery_state", qos, std::bind(&ExecutionNode::battery_callback, this, std::placeholders::_1));
			
			laser_subscriber_ = rclcpp::Node::create_subscription<sensor_msgs::msg::LaserScan>(
					"scan", qos, std::bind(&ExecutionNode::laser_callback, this, std::placeholders::_1));
			twist_publisher_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			timer_ = this -> create_wall_timer(100ms, std::bind(&ExecutionNode::twist_callback, this));
			
			//Creating behavior tree
			BT::BehaviorTreeFactory factory;

			factory.registerNodeType<RegisterDeliveryInfo>("RegisterDeliveryInfo");
			factory.registerNodeType<GoToPath>("GoToPath");
			factory.registerNodeType<DisplayFoodInfo>("DisplayFoodInfo");
			factory.registerNodeType<UpdateDeliveryInfo>("UpdateDeliveryInfo");
			factory.registerNodeType<CalculatePath>("CalculatePath");

			factory.registerSimpleCondition("IsFoodOnRobot", std::bind(IsFoodOnRobot));
			factory.registerSimpleCondition("BatteryStatus", std::bind(BatteryStatus));
			factory.registerSimpleCondition("IsFoodTaken", std::bind(IsFoodTaken));
			factory.registerSimpleCondition("IsRobotOnKitchen", std::bind(IsRobotOnKitchen));
			factory.registerSimpleCondition("IsThereObstacle", std::bind(IsThereObstacle));		
			
			//Storing the rooms information on the hash map
			std::tuple<double,double> coord_1(-2.5,-3.0);
			std::tuple<double,double> coord_2(2.0,-7.0);
			std::tuple<double,double> coord_3(13.5,-12.0);
			std::tuple<double,double> coord_4(-14,13.0);

			rooms_data["UTI-1"] = {"Pedro", "Joana", 34, true, coord_1};
			rooms_data["UTI-2"] = {"Joao", "NULL", 27, false, coord_2};
			rooms_data["UTI-3"] = {"Cleberson", "NULL", 52, false, coord_3};
			rooms_data["UTI-4"] = {"Carlos", "Maria", 74, true, coord_4};
			
			//Initializing behavior tree thread
			thread_stack_.push(std::thread(tick_tree, factory));
		}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ExecutionNode>());
	rclcpp::shutdown();
	return 0;
}
