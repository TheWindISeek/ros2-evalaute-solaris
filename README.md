# ros2_evaluate_solaris

实验一：测试动态节点变化对于静态任务链端到端延迟的影响

1.关闭超线程、隔离CPU核心（0-1），固定CPU核心频率（2.4GHz）；
2.在隔离CPU核心上，运行ROS 2应用（记做ROS_App1)，在未隔离CPU核心上，运行ROS 2应用（记作ROS_App2）；
3.ROS_App1在隔离CPU核心上稳定运行一条任务链：
（1）隔离CPU核心只有一个（暂定）；
（2）该任务链包含三个回调函数（一个定时器两个订阅者），使得定时器的周期远大于三个回调函数的执行时间之和，即隔离CPU核心的利用率在10%左右；
（3）任务链内的回调通过DDS进行通信；
（4）ROS_App1采用异步通信方式发布数据。
4.ROS_App1稳定运行后，测试下列结果：
（1）任务链的端到端延迟；
（2）任务链端到端延迟的抖动；
（3）隔离CPU核心的利用率。
5.ROS_App2在未隔离CPU核心上运行，ROS_App2不运行实际回调，只创建节点和摧毁节点，节点类型分下列情况：
 （1）每个节点内没有发布者和订阅者；
 （2）每个节点内存在一个发布者和一个订阅者，但是发布者与订阅者的主题与ROS_App1无关；
 （3）每个节点内存在两个发布者，并且发布者的主题与ROS_App1相关，但是发布者不实际发布数据。

6.ROS_App2频繁创建节点和摧毁节点，创建和摧毁节点的频率是：一秒钟创建节点10次、100次、1000次。
7.在ROS_App2运行起来后，测试ROS_App1上的任务链的端到端延迟、任务链端到端延迟的抖动、隔离CPU核心的利用率是否发生变化。
8.隔离CPU核心存在多个（两个或者三个），并且ROS_App1线程可以在隔离CPU核心上任意迁移（具体做法需要查询），重复上列实验，观察ROS_App2是否对ROS_App1存在影响和干扰。



# build
```shell
colcon build --packages-up-to ros_evaluate_solaris --allow-overriding rmw_fastrtps_cpp
```
# check rmw
run 
```
ros2 run ros_evaluate_solaris self_node 
```
to check

# rmw_fastrtos_cpp

src/rmw_fastrtps_cpp/src/publisher.cpp/create_publisher

```cpp
if (!participant_info->leave_middleware_default_qos) {
    writer_qos.publish_mode().kind = eprosima::fastrtps::ASYNCHRONOUS_PUBLISH_MODE;
}
```
# use multi ssh in one single host
https://www.jianshu.com/p/12badb7e6c10

you can test
```shell
echo "test git@github.com"
ssh -T git@github.com
echo ""test github-lucifer"
ssh -T github-lucifer
```

# how to use caret in current project

```shell
# Environment settings (keep the order as below)
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash

# Enable tracepoints which are defined hooked functions.
export LD_PRELOAD=$(readlink -f ~/Codes/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

# (Optional) Exclude nodes and topics which you are not concerned with
export CARET_IGNORE_NODES="/rviz*"
export CARET_IGNORE_TOPICS="/clock:/parameter_events"

# Launch the target application, demos_end_to_end_sample
ros2 launch caret_demos end_to_end_sample.launch.py

```

```shell

source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash

# set a destination directory. ~/.ros/tracing is default.
mkdir -p ~/ros2_caret/evaluate
export ROS_TRACE_DIR=~/ros2_caret_evaluate

ros2 caret record -s end_to_end_sample

# Start recording with pressing Enter key
# > All process tarted recording.
# > press enter to stop...

```

```shell
# 检查记录数据的有效性
ros2 caret check_ctf ~/ros2_caret_evaluate/end_to_end_sample

```

or 

use launch file

```shell
./export_caret_env.sh
ros2 launch 
```