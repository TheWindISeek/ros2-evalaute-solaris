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

# rmw_fastrtos_cpp

src/rmw_fastrtps_cpp/src/publisher.cpp/create_publisher

```cpp
if (!participant_info->leave_middleware_default_qos) {
    writer_qos.publish_mode().kind = eprosima::fastrtps::ASYNCHRONOUS_PUBLISH_MODE;
}
```
