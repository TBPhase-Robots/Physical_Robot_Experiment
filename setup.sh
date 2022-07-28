#!/bin/bash
set -m

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &

pushd Server
python3 robot_server.py &
popd

pushd Camera
python3 tracking_aruco_markers.py $1 &
popd

# python3 Experiments/Hello_World/hello_world_central_program.py &

read

ros2 topic pub /global/origin -1 geometry_msgs/msg/Vector3 '{x: -0.87, y: 0.85, z: 1.0}'

read

kill %1
kill %2
kill %3
kill %4

