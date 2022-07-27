#!/bin/bash
set -m

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &

python3 Server/robot_server.py &

pushd Camera
python3 tracking_aruco_markers.py &
popd

python3 Experiments/Hello_World/hello_world_central_program.py &

read

kill %1
kill %2
kill %3
kill %4

