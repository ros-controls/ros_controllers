#!/bin/bash

# include path を作成する
include_path=$(pwd)/include
gazebo_path=/usr/include/gazebo-2.2/
sdf_path=/usr/include/sdformat-1.4/

# CPATHに追加
export CPATH=${CPATH}:${include_path}:${gazebo_path}:${sdf_path}
