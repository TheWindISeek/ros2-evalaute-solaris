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
class SubNodeComponent : public rclcpp::Node
{
public:
  SubNodeComponent(const rclcpp::NodeOptions & options) : Node("SubNodeComponent", options)
  {
      auto callback =
        [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
          {
            this->on_listen(msg);
          };

      sub_ = create_subscription<std_msgs::msg::String>("PubNodeComponent_output_topic", 10, callback);
  }

protected:
  void on_listen(std_msgs::msg::String::ConstSharedPtr msg)
  {
      auto output_msg = std::make_unique<std_msgs::msg::String>();
      output_msg->data = msg->data;

      dummy_load_ms(C2_Exec_MS);

      uint64_t cur_time = get_clocktime();

      printf("|SubNodeComponent|-->received  msg : %ld at %lu.\n", std::stol(msg->data), cur_time);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace rclcpp_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(solaris_test::SubNodeComponent)
