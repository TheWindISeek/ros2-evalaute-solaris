#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "common_nodes.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node1 = std::make_shared<BeginNode>
    ("chain1_node1", "topic1_1", 1, 1, 10, 100, false);
  auto node2 = std::make_shared<MidNode>
    ("chain1_node2", "topic1_1", "topic1_2", 1, 2, 10, false);
  auto node3 = std::make_shared<EndNode>
    ("chain1_node3", "topic1_2", 1, 3, 10, false);
  
  rclcpp::executors::SingleThreadedExecutor executor_;

  executor_.add_node(node1);
  executor_.add_node(node2);
  executor_.add_node(node3);
  
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
