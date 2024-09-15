// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <time.h>

#include "exe_config.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace solaris_test
{
class PubNodeComponent : public rclcpp::Node
{
public:
  PubNodeComponent(const rclcpp::NodeOptions & options) : Node("PubNodeComponent", options), count_(0)
  {
    pub_ = create_publisher<std_msgs::msg::String>("PubNodeComponent_output_topic", 10);

    timer_ = create_wall_timer(C1_Period_MS, std::bind(&PubNodeComponent::on_timer, this));
  }

protected:
  void on_timer(){
    auto msg = std::make_unique<std_msgs::msg::String>();
    std::string data = std::to_string(++count_);
    data.append(C1_Msg_Size - data.size(), ' ');
    msg->data = data;

    uint64_t cur_time = get_clocktime();

    printf("|PubNodeComponent|-->published msg(size) : %ld(%ld) at %lu.\n", std::stol(msg->data), msg->data.size(), cur_time);

    dummy_load_ms(C1_Exec_MS);

    pub_->publish(std::move(msg));
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

}  // namespace rclcpp_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(solaris_test::PubNodeComponent)
