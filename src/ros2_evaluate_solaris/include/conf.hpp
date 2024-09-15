#ifndef CONF_H
#define CONF_H

#include <time.h>
#include <stdint.h>
#include <stdlib.h>

#define LOCAL_SCREEN_PRINT      1

#define ROS2_MULTI              0
#define BIND_MULTI              1

// #define CFG_EXECUTOR            ROS2_MULTI 
#define CFG_EXECUTOR            BIND_MULTI
 
#define CFG_STRATEGY            2         


// Chain1 : 1-->2
// Chain2 : 1-->3-->4-->5
// Chain3 : 6-->7-->8
// Chain4 : 9-->10-->11-->12
// Strategy 0 : {1,2}{3,4,5}{6,7,8}{9,10,11,12}
// Strategy 1 : {1,6,9}{2,3,7,10}{4,8,11}{5,12}
// Strategy 2 : {1,6,9}{2,3,10}{4,7,11}{5,8,12}
// Strategy 3 : {1,6,9}{2,3,4,5}{7,8}{10,11,12}
// Strategy 4 : {1,6,9}{2,3,4}{5,7,8}{10,11,12}
// Strategy 5 : {1,2,3}{4,5,6}{7,8,9}{10,11,12}
// Strategy 6 : {1,7,8}{6,10,11,12}{9,2}{3,4,5}
// Strategy 7 : {1}{6}{9}{2,3,4,5,7,8,10,11,12}

// 1. CFG_EXECUTOR : ROS2_MULTI-->BIND_MULTI
// 2. CFG_EXECUTOR == BIND_MULTI, CFG_STRATEGY : 0 --> 7
// 3. cd /home/neu/Desktop/ROS2_mt


// 4. . ~/ros2_humble/install/local_setup.zsh
// 5. colcon build --allow-overriding rclcpp rcl rmw
// 6. source ./install/local_setup.zsh
// 7. taskset -c 0-5 ros2 run intra_process_demo two_node_pipeline
// 8. pidstat -w -p two_node_pipeline 1 300 > /home/neu/Desktop/ROS2_mt/src/result/ROS2_CS.log(change)
// 9. mpstat -P 0-5 1 300 > /home/neu/Desktop/ROS2_mt/src/result/ROS2_CPU.log(change)

#endif