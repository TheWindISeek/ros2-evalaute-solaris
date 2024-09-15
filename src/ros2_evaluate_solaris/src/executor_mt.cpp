#include <chrono>
#include <memory>
#include <vector>
#include <random>
#include <sys/time.h>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"

//消息节点
#include "std_msgs/msg/int32.hpp"

//配置文件
#include "conf.hpp"

#include "common_nodes.hpp"

// 说明： 传输的数据是int类型的大小

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // const char* record_file = argc > 1 ? argv[1] : "record/RECORD";
    // FILE* file = freopen(record_file, "w", stdout);     
    
    dummy_load_calibration();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2,0);
    executor.set_core_start_push(2);
    executor.set_core_start_push(4);
    // executor.set_core_start_push(16);
    // executor.set_core_start_push(17);




    // chain1 short 91 2 2 1 3 3
    auto c_1_1 = std::make_shared<BeginNode>("chain_1_1", "/topic_1", 1, 1, 2, 91);
    auto c_1_2 = std::make_shared<MidNode>("chain_1_2", "/topic_1", "/topic_2", 1, 2, 2);
    auto c_1_3 = std::make_shared<MidNode>("chain_1_3", "/topic_2", "/topic_3", 1, 3, 1);
    auto c_1_4 = std::make_shared<MidNode>("chain_1_4", "/topic_3", "/topic_4", 1, 4, 3);
    auto c_1_5 = std::make_shared<EndNode>("chain_1_5", "/topic_4", 1, 5, 3);


    // chain2 long 507 32 41 38 48 49
    auto c_2_1 = std::make_shared<BeginNode>("chain_2_1", "/topic_5", 2, 1, 32, 507);
    auto c_2_2 = std::make_shared<MidNode>("chain_2_2", "/topic_5", "/topic_6", 2, 2, 41);
    auto c_2_3 = std::make_shared<MidNode>("chain_2_3", "/topic_6", "/topic_7", 2, 3, 38);
    auto c_2_4 = std::make_shared<MidNode>("chain_2_4", "/topic_7", "/topic_8", 2, 4, 48);
    auto c_2_5 = std::make_shared<EndNode>("chain_2_5", "/topic_8", 2, 5, 49);

    // chain3 long 438 46 27 27 27 44
    auto c_3_1 = std::make_shared<BeginNode>("chain_3_1", "/topic_9", 3, 1, 46, 438);
    auto c_3_2 = std::make_shared<MidNode>("chain_3_2", "/topic_9", "/topic_10", 3, 2, 27);
    auto c_3_3 = std::make_shared<MidNode>("chain_3_3", "/topic_10", "/topic_11", 3, 3, 27);
    auto c_3_4 = std::make_shared<MidNode>("chain_3_4", "/topic_11", "/topic_12", 3, 4, 27);
    auto c_3_5 = std::make_shared<EndNode>("chain_3_5", "/topic_12", 3, 5, 44);

    // chain4 long 314 50 33 23 20 25
    auto c_4_1 = std::make_shared<BeginNode>("chain_4_1", "/topic_13", 4, 1, 50, 314);
    auto c_4_2 = std::make_shared<MidNode>("chain_4_2", "/topic_13", "/topic_14", 4, 2, 33);
    auto c_4_3 = std::make_shared<MidNode>("chain_4_3", "/topic_14", "/topic_15", 4, 3, 23);
    auto c_4_4 = std::make_shared<MidNode>("chain_4_4", "/topic_15", "/topic_16", 4, 4, 20);
    auto c_4_5 = std::make_shared<EndNode>("chain_4_5", "/topic_16", 4, 5, 25);

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
    
    //ros2 默认的配置
    #if(CFG_EXECUTOR == ROS2_MULTI)

    #endif

    // 绑定线程的配置
    #if(CFG_EXECUTOR == BIND_MULTI)
      #if(CFG_STRATEGY == 0)
        // 设置调度策略
        executor.set_scheduling_strategy(1);

        //chain-1
        executor.set_thread_affinity(c_01_1->timer_, 0);
        executor.set_thread_affinity(c_01_2->sub_, 1);
        executor.set_thread_affinity(c_01_3->sub_, 2);
        executor.set_thread_affinity(c_01_4->sub_, 0);

        //chain-2
        executor.set_thread_affinity(c_02_1->timer_, 0);
        executor.set_thread_affinity(c_02_2->sub_, 1);
        executor.set_thread_affinity(c_02_3->sub_, 2);
        executor.set_thread_affinity(c_02_4->sub_, 0);


        //chain-3
        executor.set_thread_affinity(c_03_1->timer_, 0);
        executor.set_thread_affinity(c_03_2->sub_, 1);
        executor.set_thread_affinity(c_03_3->sub_, 2);
        executor.set_thread_affinity(c_03_4->sub_, 0);
      #endif

      #if(CFG_STRATEGY == 2)
        // 设置调度策略
        executor.set_scheduling_strategy(2);

        //chain-1
        // executor.set_execution_time(c_1_1->get_timer(), 10);
        // executor.set_execution_time(c_1_2->sub_, 40);
        // executor.set_execution_time(c_1_3->sub_, 40);
        // executor.set_execution_time(c_1_4->sub_, 40);
        // executor.set_execution_time(c_1_5->sub_, 40);

        // // //chain-2
        // executor.set_execution_time(c_2_1->timer_, 10);
        // executor.set_execution_time(c_2_2->sub_, 30);
        // executor.set_execution_time(c_2_3->sub_, 30);
        // executor.set_execution_time(c_2_4->sub_, 30);
        // executor.set_execution_time(c_2_5->sub_, 30);


        // //  //chain-3
        // executor.set_execution_time(c_3_1->timer_, 1);
        // executor.set_execution_time(c_3_2->sub_, 1);
        // executor.set_execution_time(c_3_3->sub_, 1);
        // executor.set_execution_time(c_3_4->sub_, 1);
        // executor.set_execution_time(c_3_4->sub_, 1);

      #endif
    #endif


    executor.spin();

    // fclose(file);
    rclcpp::shutdown();

    return 0;
}
