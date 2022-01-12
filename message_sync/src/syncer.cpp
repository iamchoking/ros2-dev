// Refrence: https://answers.ros.org/question/366440/ros-2-message_filters-timesynchronizer-minimal-example-does-not-reach-callback-function/
#include <chrono>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>    
#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class SyncerNode : public rclcpp::Node {
 public:
  SyncerNode() : Node("syncer") {
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    this->declare_parameter<std::string>("topic1_name","hw_cpp");
    this->declare_parameter<std::string>("topic2_name","hw_py");

    this->get_parameter("topic1_name",topic1_name);
    this->get_parameter("topic2_name",topic2_name);

    subscriber_temp1_.subscribe(this, topic1_name, rmw_qos_profile);
    subscriber_temp2_.subscribe(this, topic2_name, rmw_qos_profile);

    // // Uncomment this to verify that the messages indeed reach the
    // subscriber_temp1_.registerCallback(
    //     std::bind(&SyncerNode::Tmp1Callback, this, std::placeholders::_1));
    // subscriber_temp2_.registerCallback(
    //     std::bind(&SyncerNode::Tmp2Callback, this, std::placeholders::_1));

    temp_sync_ = std::make_shared<message_filters::TimeSynchronizer<std_msgs::msg::String, std_msgs::msg::String>>(subscriber_temp1_, subscriber_temp2_, 10);
    temp_sync_->registerCallback(std::bind(&SyncerNode::TempSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

 private:
  // For veryfing the single subscriber instances: Uncomment line 26-28.
  void Tmp1Callback(const std_msgs::msg::String::ConstSharedPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "[1] Data: %s",(msg->data).c_str());
  }

  // For veryfing the single subscriber instances: Uncomment line 29-31.
  void Tmp2Callback(const std_msgs::msg::String::ConstSharedPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "[2] Data: %s",(msg->data).c_str());
  }

  // This callback is never being called.
  void TempSyncCallback(
      const std_msgs::msg::String::ConstSharedPtr& msg_1,
      const std_msgs::msg::String::ConstSharedPtr& msg_2) {
    RCLCPP_INFO(this->get_logger(),
                "Received messages:\n%s\n%s",(msg_1->data).c_str(),(msg_2->data).c_str());
  }

  message_filters::Subscriber<std_msgs::msg::String> subscriber_temp1_;
  message_filters::Subscriber<std_msgs::msg::String> subscriber_temp2_;
  std::shared_ptr<message_filters::TimeSynchronizer<std_msgs::msg::String, std_msgs::msg::String>> temp_sync_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::string topic1_name;
  std::string topic2_name;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncerNode>());
  rclcpp::shutdown();
  return 0;
}