#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "common_nodes.hpp"


typedef enum enum_node_type {
    NO_PUB_SUB = 0,
    ONE_PUB_ONE_SUB = 1,
    TWO_PUB = 2
}NodeType;

const int CREATE_NODE_TIMES = 10;
const int DEFAULT_NODE_TYPE = NodeType::NO_PUB_SUB;

namespace ros_app {

  class Node_No_Pub_Sub : public rclcpp::Node {
    public:
      Node_No_Pub_Sub(const std::string& node_name, int count) 
      : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false)) {
        count_ = count;
      } 

      ~Node_No_Pub_Sub() {
        // std::cout << "NO_PUB_SUB: " << count_ << std::endl;
      }
    private:
      int count_;
  };

class Node_One_Pub_One_Sub : public rclcpp::Node
{
public:
  explicit Node_One_Pub_One_Sub(
    const std::string & node_name, const std::string &topic, int count)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false))
  {
    count_ = count;

    using namespace std::chrono_literals;
    subsciber_ = this->create_subscription<builtin_interfaces::msg::Time>(
      topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&Node_One_Pub_One_Sub::count_sub_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<builtin_interfaces::msg::Time>(
      topic,
      rclcpp::SystemDefaultsQoS());
    auto topictimer_callback =
      [&]() -> void {
        timestamp_ = this->get_clock()->now();
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "pub: Current timestamp is : " <<
            std::to_string(timestamp_.sec) <<
            " second, " <<
            std::to_string(timestamp_.nanosec) <<
            " nanosecond.");
        publisher_->publish(timestamp_);
      };
    timer_ = this->create_wall_timer(1s, topictimer_callback);
  }


  ~Node_One_Pub_One_Sub() {
    // std::cout << "ONE_PUB_ONE_SUB: " << count_ << std::endl;
  }


private:
  builtin_interfaces::msg::Time timestamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subsciber_;
  int count_;

  void count_sub_callback(const std::shared_ptr<builtin_interfaces::msg::Time> msg)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Sub: Current timestamp is : " <<
        std::to_string(msg->sec) <<
        " second, " <<
        std::to_string(msg->nanosec) <<
        " nanosecond.");
  }
};

class Node_Two_Pub : public rclcpp::Node
{
public:
  explicit Node_Two_Pub(
    const std::string & node_name, const std::string &topic, int count)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(false))
  {
    count_ = count;

    using namespace std::chrono_literals;
    publisher1_ = this->create_publisher<builtin_interfaces::msg::Time>(
      topic,
      // rclcpp::SystemDefaultsQoS());
      rclcpp::QoS(0).keep_all().transient_local().reliable());
    publisher2_ = this->create_publisher<builtin_interfaces::msg::Time>(
      topic,
      // rclcpp::SystemDefaultsQoS());
      rclcpp::QoS(0).keep_all().transient_local().reliable());   
    auto topictimer_callback =
      [&]() -> void {
        timestamp_ = this->get_clock()->now();
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "pub: Current timestamp is : " <<
            std::to_string(timestamp_.sec) <<
            " seconds, " <<
            std::to_string(timestamp_.nanosec) <<
            " nanoseconds.");
        publisher1_->publish(timestamp_);
        publisher2_->publish(timestamp_);
      };
    timer_ = this->create_wall_timer(500ms, topictimer_callback);
  }

  ~Node_Two_Pub() {
            // std::cout << "TWO_PUB: " << count_ << std::endl;
  }

private:
  int count_;
  builtin_interfaces::msg::Time timestamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher1_, publisher2_;
};

} // namespace ros_app

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // 创建节点和销毁节点 
  // 1s 创建节点 10/100/1000
  // 创建节点类型 1.没有pub和sub 2.一个pub和sub 3.两个pub, 发布者主题和rosapp1有关
  
  int times = argc > 1 ? std::stoi(argv[1]) : CREATE_NODE_TIMES;
  int node_type = argc > 2 ? std::stoi(argv[2]) : DEFAULT_NODE_TYPE;

  auto curTime = std::chrono::steady_clock::now();
  for(int i = 0; i < times; ++i) {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(1000/times));
    if(node_type == NodeType::NO_PUB_SUB) {
      auto node = std::make_shared<ros_app::Node_No_Pub_Sub>
        ("node" + std::to_string(i), i);
    } else if (node_type == NodeType::ONE_PUB_ONE_SUB) {
      auto node = std::make_shared<ros_app::Node_One_Pub_One_Sub>(
        "node" + std::to_string(i), "time", i);
    } else if (node_type == NodeType::TWO_PUB) {
      auto node = std::make_shared<ros_app::Node_Two_Pub>(
        "node" + std::to_string(i), "topic1_1", i);
    }
  }
  auto curTime2 = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::milli> spendTime = (curTime2-curTime);
  std::cout<< "speed: "<< spendTime.count()<< "ms"<< std::endl;
  std::cout << "rate: " << spendTime.count() / times << "counts per ms" << std::endl;
  rclcpp::shutdown();
  return 0;
}
