import launch
import launch_ros.actions
from tracetools_launch.action import Trace
from launch.actions import TimerAction, Shutdown

import sys
import datetime
from distutils.util import strtobool


def generate_launch_description():
  # 定时器，5分钟后自动关闭节点
  shutdown_timer = TimerAction(
        period=30.0*1,  # 4分钟
        actions=[
            Shutdown(reason='4 minutes passed, shutting down.')
        ]
  )
    
  caret_session = ""
  caret_event = ["ros2*"]
  caret_light = True

  for arg in sys.argv:
    if arg.startswith("caret_session:="):
      caret_session = arg.split(":=")[1]
    elif arg.startswith("caret_light:="):
      try:
        caret_light = strtobool(arg.split(":=")[1]) # 0 or 1
      except:
        print("Invalid arguments 'caret_light'.")
        print("Start tracing with 'ros2*'.")

  if caret_light:
    caret_event = [
            "ros2:*callback*",
            "ros2_caret:*callback*",
            "ros2:dispatch*",
            "ros2_caret:dispatch*",
            "ros2:rclcpp*" ,
            "ros2_caret:rclcpp*" ,
            "ros2_caret:rmw*",
            "ros2:rmw_take",
            "*callback_group*",
            "ros2_caret:*executor",
            "ros2_caret:dds_bind*",
            "ros2:rcl_*init",
            "ros2_caret:rcl_*init",
            "ros2_caret:caret_init"]

  if caret_session == "":
    dt_now = datetime.datetime.now()
    caret_session = "launch_trace_" + dt_now.strftime("%Y%m%d-%H%M%S")

  return launch.LaunchDescription([
    Trace(
      session_name=caret_session,
      events_kernel=[],
      events_ust=caret_event
    ),
    # launch_ros.actions.Node(
        # package='caret_demos', executable='end_to_end_sample', output='screen'),
    launch_ros.actions.Node(
        package='ros_evaluate_solaris', executable='ros_app1', output='screen'),
    shutdown_timer,
  ])