// Copyright (c) 2024 JeffreySharp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS_EVALUATE_SOLARIS__COMMON_NODES_HPP_
#define ROS_EVALUATE_SOLARIS__COMMON_NODES_HPP_

#include "rclcpp/rclcpp.hpp"

#include "dummy_load.hpp"
#include "utils.hpp"
#include "config.hpp"

#include <iostream>
#include <thread>
#include <memory>
#include <functional>
#include <string>
#include "std_msgs/msg/string.hpp"

// BeginNode MidNode EndNode 用于multi_chain的测量

/*
* Begin Node
* 一条chain的开始节点 存在timer_ pub_
* 用于开启一条chian的实例
*/
class BeginNode : public rclcpp::Node {
  public:
    explicit BeginNode(
      // 节点名称
      const std::string &node_name, 
      // 将要发布的话题的名称
      const std::string &output_topic, 
      // 第chaind_idx条链条 上的 第node_idx个节点
      const uint32_t chain_idx, const uint32_t node_idx,
      // 执行时间execute_time和period
      const int execute_time, const int period,
      // 是否使用进程内通信
      const bool use_intra_process_comms = true) 
      : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(use_intra_process_comms)),
      chain_idx_(chain_idx), node_idx_(node_idx), execute_time_(execute_time)
      {
        this->count_ = 0;
        // keep last 10
        pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period), 
          std::bind(&BeginNode::on_timer, this));
      }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_pub() {
    return pub_;
  }

  rclcpp::TimerBase::SharedPtr get_timer() {
    return timer_;
  }

  ~BeginNode() {
      // std::cout << "beginnode destory\n";
  }

  protected:
    void on_timer() {
      auto msg = std::make_unique<std_msgs::msg::String>();
      std::string data = std::to_string(++count_);
      msg->data = data;
      
      chain_start_print(count_, chain_idx_, node_idx_);
      
      dummy_load_ms(execute_time_);
      pub_->publish(std::move(msg));

      // chain_end_print(count_, chain_idx_, node_idx_);
    }


    // 当前是第几个实例
    size_t count_;
    // 第几条chain的第几个node
    uint32_t chain_idx_, node_idx_;
    // 模拟执行时间
    int execute_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

/*
* Mid Node
* 一条chain的中间节点 存在sub_ pub_
* 用于接收发布在话题上的消息 然后将结果发送到下一个话题上去
*/
class MidNode : public rclcpp::Node {
  public:
  explicit MidNode(
    const std::string &node_name,
    const std::string &input_topic,
    const std::string &output_topic,
    const uint32_t chain_idx, const uint32_t node_idx,
    const uint32_t execute_time,
    const bool use_intra_process_comms = true) :
    Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(use_intra_process_comms)),
        chain_idx_(chain_idx), node_idx_(node_idx), execute_time_(execute_time)
    {
      // 创建订阅对象
      pub_ = this->create_publisher<std_msgs::msg::String>(output_topic, 10);
      auto callback =
        [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
          {
            this->on_listen(msg);
          };
      sub_ = this->create_subscription<std_msgs::msg::String>(input_topic, 10, callback);
    }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr get_sub() {
    return sub_;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_pub() {
    return pub_;
  }

  ~MidNode() {
    // std::cout << "midnode destory\n";
  }

  protected:
    void on_listen(std_msgs::msg::String::ConstSharedPtr msg) {
      auto output_msg = std::make_unique<std_msgs::msg::String>();
      output_msg->data = msg->data;

      size_t count = std::stoul(msg->data);
      // chain_start_print(count, chain_idx_, node_idx_);
      dummy_load_ms(execute_time_);
      pub_->publish(std::move(output_msg));
      // chain_end_print(count, chain_idx_, node_idx_);
    }


  // 第几条chain的第几个node
  uint32_t chain_idx_, node_idx_;
  // 模拟执行时间
  int execute_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

/*
* End Node
* 一条chain的结束节点 存在sub_
* 负责接受到最后的消息，判断最后的执行时间
*/
class EndNode : public rclcpp::Node {
  public:
  explicit EndNode(
    const std::string &node_name,
    const std::string &input_topic,
    const uint32_t chain_idx, const uint32_t node_idx,
    const uint32_t execute_time, 
    const bool use_intra_process_comms = true) :
    Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(use_intra_process_comms)),
    chain_idx_(chain_idx), node_idx_(node_idx), execute_time_(execute_time)
     {
      auto callback =
        [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
          {
            this->on_listen(msg);
          };
      sub_ = this->create_subscription<std_msgs::msg::String>(input_topic, 10, callback);
    }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr get_sub() {
    return sub_;
  }

  ~EndNode() {
      // std::cout << "endnode destory\n";
  }

  protected:

  void on_listen(std_msgs::msg::String::ConstSharedPtr msg) {
    auto output_msg = std::make_unique<std_msgs::msg::String>();
    output_msg->data = msg->data;

    // chain_start_print(std::stoi(output_msg->data), chain_idx_, node_idx_);
    dummy_load_ms(execute_time_);
    chain_end_print(std::stoul(output_msg->data), chain_idx_, node_idx_);
  }

  // 第几条chain的第几个node
  uint32_t chain_idx_, node_idx_;
  // 模拟执行时间
  int execute_time_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

// 1-N N-1
class Chain{
  public:
  std::shared_ptr<BeginNode> begin_node;
  std::vector<std::shared_ptr<MidNode> > mid_nodes;
  std::shared_ptr<EndNode> end_node;
  ~Chain() {
    mid_nodes.clear();
      std::cout << "chain destory\n";
  }
};

#endif // !ROS2_PERFORM_TEST__COMMON_NODES_HPP_
