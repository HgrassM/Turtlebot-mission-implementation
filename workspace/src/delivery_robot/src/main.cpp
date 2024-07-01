#include <chrono>
#include <thread>
#include <stack>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp" 
#include "bt_folder/bt_nodes.cpp"

using namespace std::chrono_literals;

static const char *xml_tree = R"(<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="3"
      main_tree_to_execute="FoodDelivery">
  <BehaviorTree ID="FoodDelivery">
    <Fallback>
      <ForceFailure>
        <Sequence>
          <RegisterDeliveryInfo/>
          <IsFoodOnRobot/>
          <ReactiveSequence>
            <ReactiveSequence>
              <BatteryStatus/>
              <GoToPatientRoom/>
            </ReactiveSequence>
            <DisplayFoodInfo/>
            <IsFoodTaken/>
            <UpdateDeliveryInfo/>
          </ReactiveSequence>
        </Sequence>
      </ForceFailure>
      <Fallback>
        <IsRobotOnKitchen/>
        <ReactiveSequence>
          <BatteryStatus/>
          <GoBackToKitchen/>
        </ReactiveSequence>
      </Fallback>
    </Fallback>
  </BehaviorTree>

  <!-- Description of Node Models (used by Groot) -->
  <TreeNodesModel>
    <Condition ID="BatteryStatus"
               editable="true"/>
    <Action ID="DisplayFoodInfo"
            editable="true"/>
    <Action ID="GoBackToKitchen"
            editable="true"/>
    <Action ID="GoToPatientRoom"
            editable="true"/>
    <Condition ID="IsFoodOnRobot"
               editable="true"/>
    <Condition ID="IsFoodTaken"
               editable="true"/>
    <Condition ID="IsRobotOnKitchen"
               editable="true"/>
    <Action ID="RegisterDeliveryInfo"
            editable="true"/>
    <Action ID="UpdateDeliveryInfo"
            editable="true"/>
  </TreeNodesModel>

</root>)";

BT::BehaviorTreeFactory tree_factory;

void tick_tree(BT::BehaviorTreeFactory tree_factory) {
			auto tree = tree_factory.createTreeFromText(xml_tree);

			tree.tickRootWhileRunning();
}

class ExecutionNode : public rclcpp::Node {
	private:
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
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
			//std::cout << "Battery: " << battery_msg.percentage << std::endl;
			batteryState = battery_msg.percentage;
			battery_mutex.unlock();
		}	

	public:
		ExecutionNode() : Node("execution_node"), count_(0) {
			odom_subscriber_ = rclcpp::Node::create_subscription<nav_msgs::msg::Odometry>(
					"odom", 10, std::bind(&ExecutionNode::odom_callback, this, std::placeholders::_1));
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
			battery_subscriber_ = rclcpp::Node::create_subscription<sensor_msgs::msg::BatteryState>(
					"battery_state", qos, std::bind(&ExecutionNode::battery_callback, this, std::placeholders::_1));
			twist_publisher_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			timer_ = this -> create_wall_timer(100ms, std::bind(&ExecutionNode::twist_callback, this));

			BT::BehaviorTreeFactory factory;

			factory.registerNodeType<RegisterDeliveryInfo>("RegisterDeliveryInfo");
			factory.registerNodeType<GoToPatientRoom>("GoToPatientRoom");
			factory.registerNodeType<DisplayFoodInfo>("DisplayFoodInfo");
			factory.registerNodeType<UpdateDeliveryInfo>("UpdateDeliveryInfo");
			factory.registerNodeType<GoBackToKitchen>("GoBackToKitchen");

			factory.registerSimpleCondition("IsFoodOnRobot", std::bind(IsFoodOnRobot));
			factory.registerSimpleCondition("BatteryStatus", std::bind(BatteryStatus));
			factory.registerSimpleCondition("IsFoodTaken", std::bind(IsFoodTaken));
			factory.registerSimpleCondition("IsRobotOnKitchen", std::bind(IsRobotOnKitchen));
			
			thread_stack_.push(std::thread(tick_tree, factory));
		}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ExecutionNode>());
	rclcpp::shutdown();
	return 0;
}
