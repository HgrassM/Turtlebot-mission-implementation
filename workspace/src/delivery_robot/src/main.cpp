#include <chrono>

#include "rclcpp/rclcpp.hpp"
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
          <RegisterDeliveryInfo delivery_info_output="{delivery_Info}"/>
          <IsFoodOnRobot/>
          <ReactiveSequence>
            <ReactiveSequence>
              <BatteryStatus/>
              <GoToPatientRoom/>
            </ReactiveSequence>
            <DisplayFoodInfo info_for_display="{delivery_info}"/>
            <IsFoodTaken/>
            <UpdateDeliveryInfo update_info="{delivery_info}"/>
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
            editable="true">
      <input_port name="info_for_display"/>
    </Action>
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
            editable="true">
      <output_port name="delivery_info_output"/>
    </Action>
    <Action ID="UpdateDeliveryInfo"
            editable="true">
      <output_port name="update_info"/>
    </Action>
  </TreeNodesModel>

</root>)";

class ExecutionNode : public rclcpp::Node {
	private:
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
		size_t count_;

		void twist_callback() {
			twist_mutex.lock();
			twist_publisher_ -> publish(velocity);
			twist_mutex.unlock();
		}

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
			odom_subscriber_ = rclcpp::Node::create_subscription<nav_msgs::msg::Odometry>(
					"odom", 10, std::bind(&ExecutionNode::odom_callback, this, std::placeholders::_1));
			battery_subscriber_ = rclcpp::Node::create_subscription<sensor_msgs::msg::BatteryState>(
					"battery_state", 10, std::bind(&ExecutionNode::battery_callback, this, std::placeholders::_1));
			twist_publisher_ = rclcpp::Node::create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			timer_ = rclcpp::Node::create_wall_timer(500ms, std::bind(&ExecutionNode::twist_callback, this));

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

			auto behavior_tree = factory.createTreeFromText(xml_tree);

			behavior_tree.tickRootWhileRunning();
		}
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ExecutionNode>());
	rclcpp::shutdown();
	return 0;
}
