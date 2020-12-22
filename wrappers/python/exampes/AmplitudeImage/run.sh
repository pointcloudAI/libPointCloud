#!/bin/bash
export POINTCLOUD_SDK_PATH="./../../../../libs/macos"
export DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH:$POINTCLOUD_SDK_PATH/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$POINTCLOUD_SDK_PATH/lib"
export PYTHONPATH=$POINTCLOUD_SDK_PATH/lib/python3:$PYTHONPATH
echo "$PYTHONPATH"
python3 graph.py
