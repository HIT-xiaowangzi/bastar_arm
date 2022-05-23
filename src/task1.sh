#!/bin/bash
gnome-terminal -t "yolo" -x bash -c "python /home/wangzirui/HIT-DLR/src/Yolov5_ros/yolov5_ros/yolov5_ros/yolov5/detect_task_one.py;exec bash;"
sleep 10

gnome-terminal -t "server" -x bash -c "rosrun task1 server;exec bash;"
sleep 3

gnome-terminal -t "main" -x bash -c "rosrun task1 main;exec bash;"
sleep 3

gnome-terminal -t "left_hand_camera" -x bash -c "rosrun task1 left_hand_camera.py;exec bash;"
sleep 3

gnome-terminal -t "state_machine" -x bash -c "rosrun task1 state_machine;exec bash;"
sleep 3