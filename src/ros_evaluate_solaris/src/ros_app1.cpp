#include <memory>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "common_nodes.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  const char* record_file = argc > 1 ? argv[1] : "record/RECORD";

  FILE* file = freopen(record_file, "w", stdout);  
  int times = 30;
  bool use_intra_process_comms = false;
  // 创建稳定的任务链 false=>采取异步通信的方式
  // auto node1 = std::make_shared<BeginNode>
  //   ("chain1_node1", "/topic1_1", 1, 1, times, 100, false);
  // auto node2 = std::make_shared<MidNode>
  //   ("chain1_node2", "/topic1_1", "/topic1_2", 1, 2, times, false);
  // auto node3 = std::make_shared<EndNode>
  //   ("chain1_node3", "/topic1_2", 1, 3, times, false);
  
      // chain1 short 91 2 2 1 3 3
    auto c_1_1 = std::make_shared<BeginNode>("chain_1_1", "/topic_1", 1, 1, 2*times, 91, use_intra_process_comms);
    auto c_1_2 = std::make_shared<MidNode>("chain_1_2", "/topic_1", "/topic_2", 1, 2, 2*times, use_intra_process_comms);
    auto c_1_3 = std::make_shared<MidNode>("chain_1_3", "/topic_2", "/topic_3", 1, 3, 1*times, use_intra_process_comms);
    auto c_1_4 = std::make_shared<MidNode>("chain_1_4", "/topic_3", "/topic_4", 1, 4, 3*times, use_intra_process_comms);
    auto c_1_5 = std::make_shared<EndNode>("chain_1_5", "/topic_4", 1, 5, 3*times, use_intra_process_comms);


    // chain2 long 507 32 41 38 48 49
    auto c_2_1 = std::make_shared<BeginNode>("chain_2_1", "/topic_5", 2, 1, 32*times, 507, use_intra_process_comms);
    auto c_2_2 = std::make_shared<MidNode>("chain_2_2", "/topic_5", "/topic_6", 2, 2, 41*times, use_intra_process_comms);
    auto c_2_3 = std::make_shared<MidNode>("chain_2_3", "/topic_6", "/topic_7", 2, 3, 38*times, use_intra_process_comms);
    auto c_2_4 = std::make_shared<MidNode>("chain_2_4", "/topic_7", "/topic_8", 2, 4, 48*times, use_intra_process_comms);
    auto c_2_5 = std::make_shared<EndNode>("chain_2_5", "/topic_8", 2, 5, 49*times, use_intra_process_comms);

    // chain3 long 438 46 27 27 27 44
    auto c_3_1 = std::make_shared<BeginNode>("chain_3_1", "/topic_9", 3, 1, 46*times, 438, use_intra_process_comms);
    auto c_3_2 = std::make_shared<MidNode>("chain_3_2", "/topic_9", "/topic_10", 3, 2, 27*times, use_intra_process_comms);
    auto c_3_3 = std::make_shared<MidNode>("chain_3_3", "/topic_10", "/topic_11", 3, 3, 27*times, use_intra_process_comms);
    auto c_3_4 = std::make_shared<MidNode>("chain_3_4", "/topic_11", "/topic_12", 3, 4, 27*times, use_intra_process_comms);
    auto c_3_5 = std::make_shared<EndNode>("chain_3_5", "/topic_12", 3, 5, 44*times, use_intra_process_comms);

    // chain4 long 314 50 33 23 20 25
    auto c_4_1 = std::make_shared<BeginNode>("chain_4_1", "/topic_13", 4, 1, 50*times, 314, use_intra_process_comms);
    auto c_4_2 = std::make_shared<MidNode>("chain_4_2", "/topic_13", "/topic_14", 4, 2, 33*times, use_intra_process_comms);
    auto c_4_3 = std::make_shared<MidNode>("chain_4_3", "/topic_14", "/topic_15", 4, 3, 23*times, use_intra_process_comms);
    auto c_4_4 = std::make_shared<MidNode>("chain_4_4", "/topic_15", "/topic_16", 4, 4, 20*times, use_intra_process_comms);
    auto c_4_5 = std::make_shared<EndNode>("chain_4_5", "/topic_16", 4, 5, 25*times, use_intra_process_comms);

  rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(c_1_1);
    executor.add_node(c_1_2);
    executor.add_node(c_1_3);
    executor.add_node(c_1_4);
    executor.add_node(c_1_5);

    executor.add_node(c_2_1);
    executor.add_node(c_2_2);
    executor.add_node(c_2_3);
    executor.add_node(c_2_4);
    executor.add_node(c_2_5);

    executor.add_node(c_3_1);
    executor.add_node(c_3_2);
    executor.add_node(c_3_3);
    executor.add_node(c_3_4);
    executor.add_node(c_3_5);

    executor.add_node(c_4_1);
    executor.add_node(c_4_2);
    executor.add_node(c_4_3);
    executor.add_node(c_4_4);
    executor.add_node(c_4_5);
  
  executor.spin();
  fclose(file);
  rclcpp::shutdown();
  return 0;
}
