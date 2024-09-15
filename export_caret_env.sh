# 找到caret.so
export LD_PRELOAD=$(readlink -f ~/Codes/ros2_caret_ws/install/caret_trace/lib/libcaret.so)
echo $LD_PRELOAD
# 记录的结果在哪个文件夹
export ROS_TRACE_DIR=~/ros2_caret_evaluate
echo $ROS_TRACE_DIR