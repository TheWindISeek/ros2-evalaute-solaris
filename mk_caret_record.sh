#!/bin/bash
#判断目录是否存在，不存在创建
dir=~/ros2_caret_evaluate
if [ ! -d  $dir ]
then
	mkdir -p $dir
    # echo -e "\033[32m this is $dir success ! \033[0m"
fi

mv -fb jupyter-lab/* $dir
